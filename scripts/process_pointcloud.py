"""WebODM LAZ -> assets/pointcloud/{points.bin, meta.json}

    .venv/Scripts/python.exe scripts/process_pointcloud.py [--max-points N]

Viewer world coordinates (three.js, Y-up, meters):
    X = easting - E0,  Y = elevation - Z0,  Z = -(northing - N0)
with (E0, N0) from the LAZ header offsets and Z0 = ground level (10th
percentile elevation), recorded in meta.json as origin_utm.

points.bin layout: points are shuffled (so any prefix is a uniform subsample,
letting the viewer render progressively while downloading), then written in
chunks of `chunk_size` points. Each chunk is
    uint16[n*3] positions (quantized to bbox_min..bbox_max), then uint8[n*3] RGB
The last chunk may be shorter.
"""
import argparse
import json
import os

import laspy
import numpy as np

LAZ = 'data/raw/webodm/all/odm_georeferencing/odm_georeferenced_model.laz'
OUT_DIR = 'assets/pointcloud'
CHUNK = 65536


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--max-points', type=int, default=0, help='randomly subsample to at most N points (0 = keep all)')
    args = ap.parse_args()

    las = laspy.read(LAZ)
    E0, N0 = float(las.header.offsets[0]), float(las.header.offsets[1])
    z = np.asarray(las.z)
    Z0 = round(float(np.percentile(z, 10)), 1)

    xyz = np.stack([np.asarray(las.x) - E0, z - Z0, -(np.asarray(las.y) - N0)], 1).astype(np.float64)
    rgb = np.stack([las.red, las.green, las.blue], 1)
    if rgb.max() > 255:  # some exporters use 16-bit color
        rgb = rgb >> 8
    rgb = rgb.astype(np.uint8)

    rng = np.random.default_rng(42)
    order = rng.permutation(len(xyz))
    if args.max_points and len(order) > args.max_points:
        order = order[: args.max_points]
    xyz, rgb = xyz[order], rgb[order]

    lo, hi = xyz.min(0), xyz.max(0)
    q = np.round((xyz - lo) / (hi - lo) * 65535).astype('<u2')

    os.makedirs(OUT_DIR, exist_ok=True)
    with open(f'{OUT_DIR}/points.bin', 'wb') as f:
        for s in range(0, len(q), CHUNK):
            f.write(q[s : s + CHUNK].tobytes())
            f.write(rgb[s : s + CHUNK].tobytes())

    meta = {
        'count': int(len(q)),
        'chunk_size': CHUNK,
        'bbox_min': lo.round(4).tolist(),
        'bbox_max': hi.round(4).tolist(),
        'origin_utm': [E0, N0, Z0],
        'crs': 'EPSG:32616 (WGS 84 / UTM zone 16N)',
        'axes': 'X=east, Y=up, Z=-north (meters, relative to origin_utm)',
    }
    json.dump(meta, open(f'{OUT_DIR}/meta.json', 'w'), indent=2)
    size = os.path.getsize(f'{OUT_DIR}/points.bin') / 1e6
    print(f'wrote {len(q):,} points ({size:.1f} MB), origin {E0}, {N0}, {Z0}')


if __name__ == '__main__':
    main()
