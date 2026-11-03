"""South-facade measurements shared by align.py (to lock the fit) and its own CLI (to check it).

    .venv/Scripts/python.exe scripts/facade.py [--plot OUT_DIR]

The south facade is the best-captured vertical surface in the drone scan and has a
precise along-wall reference: the brick panels / stucco pilaster strips are sharp
color edges in the scan and painted-face edges in the model. All coordinates here
are Y-up (viewer world): x/z horizontal, y up.

measure() reports, in the facade's own frame (x along the wall to the east,
z out of the wall to the south):
  along_shift   scan strip edges minus model strip edges (m, median)
  depth_off     scan wall surface minus model wall plane (m; + = scan in front)
  depth_slope   trend of that difference along the wall (rad) -> yaw error
"""
import argparse
import json

import laspy
import numpy as np

LAZ = 'data/raw/webodm/all/odm_georeferencing/odm_georeferenced_model.laz'
TRIS = 'data/work/model_tris_envelope__{}.bin'
P = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float)  # Z-up (E, N, U) -> Y-up (x=E, y=U, z=-N)


def normals(t):
    n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    area = np.linalg.norm(n, axis=1) / 2
    return n / (2 * area[:, None] + 1e-12), area


def measure(pts, rgb, brick, stucco):
    """pts (n,3) and model triangles (m,3,3) in the same Y-up frame; rgb 0..255."""
    nb, ab = normals(brick)
    south = nb[:, 2] > 0.95
    N = (nb[south] * ab[south, None]).sum(0)
    N[1] = 0
    N /= np.linalg.norm(N)
    U = np.cross([0.0, 1.0, 0.0], N)  # along the wall, pointing east
    frame = lambda q: np.stack([q @ U, q[..., 1], q @ N], -1)
    fp, fbk, fst = frame(pts), frame(brick), frame(stucco)

    # facade = the biggest south-facing brick plane by area
    nb, ab = normals(fbk)
    d = fbk[:, :, 2].mean(1)
    cand = nb[:, 2] > 0.99
    planes = {}
    for di, ai in zip(np.round(d[cand], 1), ab[cand]):
        planes[di] = planes.get(di, 0) + ai
    zp = max(planes, key=planes.get)
    on = lambda t: np.abs(t[:, :, 2].mean(1) - zp) < 0.1
    fb = fbk[cand & on(fbk)]
    z0 = fb[:, :, 2].mean()
    ns, _ = normals(fst)
    fs = fst[(ns[:, 2] > 0.99) & on(fst)]
    x_lo, x_hi = fb[:, :, 0].min(), fb[:, :, 0].max()
    y_lo, y_hi = fb[:, :, 1].min(), fb[:, :, 1].max()

    strips = []
    for lo, hi in sorted((t[:, 0].min(), t[:, 0].max()) for t in fs):
        if strips and lo <= strips[-1][1] + 0.02:
            strips[-1][1] = max(strips[-1][1], hi)
        else:
            strips.append([lo, hi])
    strips = [s for s in strips if s[1] - s[0] > 0.3]

    sel = (np.abs(fp[:, 2] - z0) < 0.5) & (fp[:, 0] > x_lo - 0.6) & (fp[:, 0] < x_hi + 0.6) & \
          (fp[:, 1] > y_lo + 0.8) & (fp[:, 1] < y_hi - 0.8)
    p, col = fp[sel], rgb[sel]
    red = col[:, 0] - col[:, 2]
    bins = np.arange(x_lo - 0.6, x_hi + 0.6, 0.05)
    k = np.digitize(p[:, 0], bins) - 1
    prof = np.full(len(bins), np.nan)
    depth = np.full(len(bins), np.nan)
    cnt = np.bincount(k[(k >= 0) & (k < len(bins))], minlength=len(bins))
    order = np.argsort(k)
    ks, splits = k[order], np.searchsorted(k[order], np.arange(len(bins) + 1))
    for i in range(len(bins)):
        if cnt[i] >= 15:
            idx = order[splits[i]:splits[i + 1]]
            prof[i] = np.median(red[idx])
            depth[i] = np.median(p[idx, 2] - z0)
    xc = bins + 0.025

    lab = np.ones(len(bins))
    for lo, hi in strips:
        lab[(xc > lo) & (xc < hi)] = 0
    good = ~np.isnan(prof) & (xc > x_lo) & (xc < x_hi)
    thr = (np.nanmedian(prof[good & (lab == 1)]) + np.nanmedian(prof[good & (lab == 0)])) / 2

    sm = prof.copy()
    for i in range(len(sm)):
        w = prof[max(0, i - 2):i + 3]
        w = w[~np.isnan(w)]
        sm[i] = np.median(w) if len(w) else np.nan
    scan_edges = [xc[i] + 0.025 for i in range(len(sm) - 1)
                  if not np.isnan(sm[i]) and not np.isnan(sm[i + 1]) and (sm[i] > thr) != (sm[i + 1] > thr)]
    pairs = []
    for e in sorted(v for s in strips for v in s):
        dd = [s - e for s in scan_edges]
        if dd and min(abs(v) for v in dd) < 0.6:
            pairs.append((e, min(dd, key=abs)))
    pairs = np.array(pairs)
    A = np.c_[pairs[:, 0] - pairs[:, 0].mean(), np.ones(len(pairs))]
    (scale_slope, _), *_ = np.linalg.lstsq(A, pairs[:, 1], rcond=None)

    g = good & (cnt > 30)
    xm = xc[g].mean()
    A = np.c_[xc[g] - xm, np.ones(g.sum())]
    (dslope, doff), *_ = np.linalg.lstsq(A, depth[g], rcond=None)

    return {
        'U': U, 'N': N, 'z0': z0, 'x_mean': xm, 'x_range': (x_lo, x_hi), 'y_range': (y_lo, y_hi),
        'strips': strips, 'pairs': pairs,
        'along_shift': float(np.median(pairs[:, 1])),
        'along_spread': float(pairs[:, 1].std()),
        'scale_slope': float(scale_slope),
        'depth_off': float(doff), 'depth_slope': float(np.arctan(dslope)),
        'profile': (xc, prof, depth, lab),
    }


def correction(m):
    """Rigid move (about vertical) of the MODEL that cancels the measured errors.
    Returns (theta, pivot, shift): rotate by theta about +Y around pivot, then add shift
    (all Y-up)."""
    U, N = m['U'], m['N']
    pivot = U * m['x_mean'] + N * m['z0']
    U2 = U + np.tan(m['depth_slope']) * N  # direction of the scan's wall
    U2 /= np.linalg.norm(U2)
    theta = np.arctan2(np.cross(U, U2)[1], U @ U2)
    shift = U * m['along_shift'] + N * m['depth_off']
    return theta, pivot, shift


def report(m):
    x_lo, x_hi = m['x_range']
    print(f'south facade {x_hi - x_lo:.1f} m long, {len(m["strips"])} stucco strips, {len(m["pairs"])} matched edges')
    print(f'  along the wall: scan - model = {m["along_shift"] * 100:+.1f} cm (edge spread {m["along_spread"] * 100:.1f} cm, '
          f'linear trend {m["scale_slope"] * (x_hi - x_lo) * 100:+.0f} cm end to end)')
    print(f'  across the wall: scan {m["depth_off"] * 100:+.1f} cm (+ = in front of model), '
          f'yaw {np.degrees(m["depth_slope"]):+.3f} deg ({np.tan(m["depth_slope"]) * (x_hi - x_lo) * 100:+.0f} cm end to end)')


def load_viewer_frame():
    """Scan (leveled) + model brick/stucco, in viewer world coords, as the page shows them."""
    a = json.load(open('assets/alignment.json'))
    meta = json.load(open('assets/pointcloud/meta.json'))
    E0, N0, Z0 = meta['origin_utm']
    las = laspy.read(LAZ)
    c = np.stack([np.asarray(las.x) - E0, np.asarray(las.y) - N0, np.asarray(las.z) - Z0], 1) @ P.T
    M = np.array(a['scan']['matrix']).reshape(4, 4).T
    rgb = np.stack([las.red, las.green, las.blue], 1).astype(np.float32)
    yaw = a['model']['rotationY_rad']
    Ry = np.array([[np.cos(yaw), 0, np.sin(yaw)], [0, 1, 0], [-np.sin(yaw), 0, np.cos(yaw)]])
    tris = lambda name: (np.fromfile(TRIS.format(name), np.float32).reshape(-1, 3, 3) @ P.T) @ Ry.T + a['model']['position']
    return c @ M[:3, :3].T + M[:3, 3], rgb, tris('brick'), tris('stucco')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--plot', help='write a profile PNG here')
    args = ap.parse_args()
    pts, rgb, brick, stucco = load_viewer_frame()
    m = measure(pts, rgb, brick, stucco)
    report(m)
    for e, d in m['pairs']:
        print(f'    edge x={e:7.2f}  {d * 100:+.1f} cm')
    if args.plot:
        from PIL import Image, ImageDraw
        xc, prof, depth, lab = m['profile']
        im = Image.new('RGB', (len(xc) * 4, 200), 'black')
        dr = ImageDraw.Draw(im)
        for i in range(len(xc)):
            x = i * 4
            dr.rectangle([x, 0, x + 3, 40], fill=(180, 80, 60) if lab[i] else (230, 225, 210))
            if not np.isnan(prof[i]):
                v = int(np.clip(prof[i] * 2, 0, 255))
                dr.rectangle([x, 50, x + 3, 90], fill=(v, v // 3, v // 4))
                y = int(150 - np.clip(depth[i], -0.4, 0.4) * 100)
                dr.rectangle([x, y, x + 3, y + 2], fill=(255, 220, 0))
        dr.line([0, 150, im.width, 150], fill=(90, 90, 90))
        im.save(f'{args.plot}/facade_profile.png')
        print('plot:', f'{args.plot}/facade_profile.png  (top: model, middle: scan color, bottom: scan depth)')


if __name__ == '__main__':
    main()
