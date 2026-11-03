"""Align the Revit model to the WebODM point cloud -> assets/alignment.json

    .venv/Scripts/python.exe scripts/align.py [--debug OUT_DIR] [--yaw DEG]

Inputs:
  data/raw/webodm/all/odm_georeferencing/odm_georeferenced_model.laz  (UTM, meters)
  data/work/model_tris_{envelope,site}.bin  (from convert_model.mjs: Revit coords, m, Z-up)
  assets/pointcloud/meta.json               (from process_pointcloud.py: world origin)

Scale is known (Revit feet -> meters happens upstream), so we solve 4 DOF:
yaw about vertical + a 3D offset.

  1. Coarse: for every yaw (1 deg steps), rasterize the model's walls/roof as
     height-above-local-ground and find the offset that best matches the scan's
     height-above-ground (sum of squared differences via FFT). Using heights,
     not just a footprint mask, keeps a rectangular roof from matching at 180 deg.
  2. Fine: trimmed point-to-point ICP on walls/roof, yaw + translation only.

--yaw forces the coarse yaw (degrees), if the automatic pick is ever wrong.
--debug writes top-down overlay images for checking the fit by eye.
"""
import argparse
import json

import laspy
import numpy as np
from numpy.fft import irfft2, rfft2
from scipy.ndimage import minimum_filter, uniform_filter
from scipy.spatial import cKDTree

import facade  # scripts/facade.py

LAZ = 'data/raw/webodm/all/odm_georeferencing/odm_georeferenced_model.laz'
TRIS = 'data/work/model_tris_{}.bin'
META = 'assets/pointcloud/meta.json'
OUT = 'assets/alignment.json'
RES = 0.5          # coarse grid cell, m
GROUND_WIN = 20.0  # min-filter window for "local ground", m (wider than the house is narrow)
rng = np.random.default_rng(0)


def load_tris(layer):
    return np.fromfile(TRIS.format(layer), np.float32).reshape(-1, 3, 3).astype(np.float64)


def sample_tris(t, n):
    a = 0.5 * np.linalg.norm(np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0]), axis=1)
    idx = rng.choice(len(t), n, p=a / a.sum())
    r1 = np.sqrt(rng.random((n, 1)))
    r2 = rng.random((n, 1))
    return (1 - r1) * t[idx, 0] + r1 * (1 - r2) * t[idx, 1] + r1 * r2 * t[idx, 2]


def rodrigues(w):
    th = np.linalg.norm(w)
    if th < 1e-12:
        return np.eye(3)
    k = w / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def yaw_matrix(yaw):
    R = np.eye(3)
    R[:2, :2] = rot(yaw)
    return R


def roof_samples(tris, n=150_000):
    """Points on the model's roof: up-facing triangles near the top of the building."""
    cr = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    nz = cr[:, 2] / (np.linalg.norm(cr, axis=1) + 1e-12)
    top = tris[:, :, 2].mean(1) > tris[:, :, 2].max() - 1.5
    return sample_tris(tris[(nz > 0.95) & top], n)


def roof_tilt(cloud, roof_w):
    """Fit a plane to (scan z - model z) over the roof. Returns (slope_x, slope_y, offset,
    residual rms). The model's roof has real drainage slopes, so we compare against
    the model surface rather than assuming the roof is flat."""
    lo, hi = roof_w.min(0) + 1.0, roof_w.max(0) - 1.0  # stay off the edges/parapets
    s = np.all((cloud[:, :2] > lo[:2]) & (cloud[:, :2] < hi[:2]), axis=1)
    p = cloud[s]
    dist, i = cKDTree(roof_w[:, :2]).query(p[:, :2])
    dz = p[:, 2] - roof_w[i, 2]
    keep = (dist < 0.3) & (np.abs(dz - np.median(dz)) < 0.5)  # drop rooftop units, overhanging trees
    p, dz = p[keep], dz[keep]
    A = np.c_[p[:, 0] - p[:, 0].mean(), p[:, 1] - p[:, 1].mean(), np.ones(len(p))]
    k = np.linalg.lstsq(A, dz, rcond=None)[0]
    return k[0], k[1], k[2], float((dz - A @ k).std())


def tilt_fix(sx, sy):
    """Rotation that levels a plane z = sx*x + sy*y (takes its normal to +Z)."""
    nrm = np.array([-sx, -sy, 1.0])
    nrm /= np.linalg.norm(nrm)
    axis = np.cross(nrm, [0.0, 0.0, 1.0])
    s = np.linalg.norm(axis)
    return np.eye(3) if s < 1e-12 else rodrigues(axis / s * np.arcsin(s))


def rot(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s], [s, c]])


def apply(p, yaw, t):
    q = p.copy()
    q[:, :2] = p[:, :2] @ rot(yaw).T
    return q + t


def grid_minmax(p, x0, y0, shape):
    """Per-cell max z and min z (NaN / inf where empty)."""
    i = ((p[:, 1] - y0) / RES).astype(int)
    j = ((p[:, 0] - x0) / RES).astype(int)
    ok = (i >= 0) & (i < shape[0]) & (j >= 0) & (j < shape[1])
    i, j, z = i[ok], j[ok], p[ok, 2]
    zmax = np.full(shape, -np.inf)
    zmin = np.full(shape, np.inf)
    np.maximum.at(zmax, (i, j), z)
    np.minimum.at(zmin, (i, j), z)
    return zmax, zmin


def height_above_ground(zmax, zmin):
    win = int(GROUND_WIN / RES) | 1
    ground = minimum_filter(zmin, size=win, mode='nearest')
    h = zmax - ground
    valid = np.isfinite(h)
    h[~valid] = 0
    return np.maximum(h, 0), valid


def coarse(cloud, env, ground_pts, forced_yaw=None):
    """Return (yaw, t) taking model coords -> cloud coords (Z-up, relative to origin)."""
    pad = 40.0
    x0, y0 = cloud[:, 0].min() - pad, cloud[:, 1].min() - pad
    shape = (int((cloud[:, 1].max() + pad - y0) / RES), int((cloud[:, 0].max() + pad - x0) / RES))
    C, V = height_above_ground(*grid_minmax(cloud, x0, y0, shape))

    # model rasterized on a small grid centered on its own centroid
    center = env[:, :2].mean(0)
    allm = np.vstack([env, ground_pts])
    half = np.abs(allm[:, :2] - center).max() + 2
    mshape = (int(2 * half / RES),) * 2
    Cmax = C.copy()
    Vf = V.astype(np.float64)

    results = []
    yaws = [forced_yaw] if forced_yaw is not None else np.arange(-180, 180, 1.0)
    for yaw_deg in yaws:
        R = rot(np.radians(yaw_deg))
        e = env.copy(); e[:, :2] = (env[:, :2] - center) @ R.T
        g = ground_pts.copy(); g[:, :2] = (ground_pts[:, :2] - center) @ R.T
        # ground from walls+roof+site; heights only where walls/roof are
        _, gmin = grid_minmax(np.vstack([e, g]), -half, -half, mshape)
        emax, emin = grid_minmax(e, -half, -half, mshape)
        win = int(GROUND_WIN / RES) | 1
        ground = minimum_filter(np.minimum(gmin, emin), size=win, mode='nearest')
        W = np.isfinite(emax) & np.isfinite(ground)
        M = np.where(W, np.maximum(emax - ground, 0), 0)
        W = W & (M > 1.0)  # ignore low bits (steps, curbs) - roof and walls carry the shape

        # embed in the big grid, top-left aligned; correlation index = shift of the top-left corner
        Mb = np.zeros(shape); Mb[: mshape[0], : mshape[1]] = M
        Wb = np.zeros(shape); Wb[: mshape[0], : mshape[1]] = W
        Cc = np.minimum(Cmax, M.max() + 3)  # trees taller than the roof don't dominate
        FW, FM, FM2 = rfft2(Wb), rfft2(Mb), rfft2(Wb * Mb * Mb)
        corr = lambda A, FB: irfft2(rfft2(A) * np.conj(FB), s=shape)
        cov = corr(Vf, FW)
        ssd = corr(Vf * Cc * Cc, FW) - 2 * corr(Vf * Cc, FM) + corr(Vf, FM2)
        score = np.where(cov > 0.6 * W.sum(), ssd / np.maximum(cov, 1), np.inf)
        di, dj = np.unravel_index(np.argmin(score), shape)
        # model point at (u,v) relative to center lands at (x0 + dj*RES + half + u, ...)
        yaw = np.radians(yaw_deg)
        txy = np.array([x0 + dj * RES + half, y0 + di * RES + half]) - rot(yaw) @ center
        results.append((float(score[di, dj]), float(yaw_deg), np.array([txy[0], txy[1], 0.0])))
    return results


def icp(src, target, tree, yaw, t, schedule, verbose=False):
    for thresh in schedule:
        for _ in range(20):
            p = apply(src, yaw, t)
            d, idx = tree.query(p, distance_upper_bound=thresh)
            ok = np.isfinite(d)
            if ok.sum() < 50:
                return yaw, t
            a, b = p[ok], target[idx[ok]]
            ca, cb = a.mean(0), b.mean(0)
            H = (a[:, :2] - ca[:2]).T @ (b[:, :2] - cb[:2])
            dyaw = np.arctan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
            yaw += dyaw
            t = (b - apply(src[ok], yaw, np.zeros(3))).mean(0)
            if abs(dyaw) < 1e-5:
                break
        if verbose:
            print(f'  icp thresh={thresh:4.2f}  inliers={ok.mean():.0%}  rms={np.sqrt((d[ok] ** 2).mean()):.3f} m  '
                  f'yaw={np.degrees(yaw):.2f}', flush=True)
    return yaw, t


def fit_quality(src, tree, yaw, t):
    d, _ = tree.query(apply(src, yaw, t))
    return d


def voxel_down(p, size):
    k = np.floor(p / size).astype(np.int64)
    _, idx = np.unique(k, axis=0, return_index=True)
    return p[idx]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--debug', help='write overlay images to this directory')
    ap.add_argument('--yaw', type=float, help='force the coarse yaw, degrees')
    args = ap.parse_args()

    meta = json.load(open(META))
    E0, N0, Z0 = meta['origin_utm']
    las = laspy.read(LAZ)
    cloud = np.stack([np.asarray(las.x) - E0, np.asarray(las.y) - N0, np.asarray(las.z) - Z0], 1)

    env = sample_tris(load_tris('envelope'), 200_000)
    site = sample_tris(load_tris('site'), 100_000)

    print('coarse search...', flush=True)
    results = coarse(cloud, env, site, args.yaw)

    # The height match alone can be fooled (trees along one side of the house), so
    # take the best candidate in each 30-degree bin and let a quick ICP decide.
    starts = {}
    for score, yaw_deg, t in results:
        b = int((yaw_deg + 180) // 30)
        if b not in starts or score < starts[b][0]:
            starts[b] = (score, yaw_deg, t)

    target = voxel_down(cloud, 0.08)
    tree = cKDTree(target)
    tree2d = cKDTree(target[:, :2])
    src = env[rng.choice(len(env), 30_000, replace=False)]
    quick = src[:8000]

    best = None
    for score, yaw_deg, t in sorted(starts.values(), key=lambda r: r[1]):
        yaw, t = np.radians(yaw_deg), t.copy()
        # vertical start: median (scan z - model z) over the roof (top 20% of model points);
        # a median shrugs off the trees that overhang part of it
        p = apply(env, yaw, t)
        roof = p[p[:, 2] > np.percentile(p[:, 2], 80)]
        _, idx = tree2d.query(roof[:, :2], k=8)
        t[2] = np.median(np.median(target[idx, 2], axis=1) - roof[:, 2])
        yaw, t = icp(quick, target, tree, yaw, t, [2.0, 1.0, 0.5, 0.3])
        q = (fit_quality(quick, tree, yaw, t) < 0.25).mean()
        print(f'  start {yaw_deg:5.0f} deg -> {np.degrees(yaw):7.2f} deg  within 25 cm: {q:.0%}', flush=True)
        if best is None or q > best[0]:
            best = (q, yaw, t)

    _, yaw, t = best
    print(f'refining from yaw={np.degrees(yaw):.2f}', flush=True)
    yaw, t = icp(src, target, tree, yaw, t, [1.0, 0.7, 0.5, 0.35, 0.25], verbose=True)
    d = fit_quality(src, tree, yaw, t)
    print(f'4-dof: yaw={np.degrees(yaw):.3f} deg  median dist={np.median(d):.3f} m  within 25 cm: {(d < 0.25).mean():.0%}')

    # --- tilt: drone-GPS georeferencing can leave the whole scan slightly tipped ---
    # The Revit model is plumb, so the correction goes on the scan: rotate it about
    # the roof center so its roof matches the model's roof, then re-fit yaw/offset.
    # The roof is the surface an overhead drone captures best (walls are patchy and
    # tree-covered), so it's the reference for tilt.
    roof = roof_samples(load_tris('envelope'))
    pivot = apply(roof, yaw, t).mean(0)
    R_lev = np.eye(3)
    level = lambda c: (c - pivot) @ R_lev.T + pivot
    sx, sy, _, rms = roof_tilt(cloud, apply(roof, yaw, t))
    print(f'raw scan vs model roof: tilt E-W {np.degrees(sx):+.2f} deg, N-S {np.degrees(sy):+.2f} deg', flush=True)
    for it in range(4):
        R_lev = tilt_fix(sx, sy) @ R_lev  # remove the remaining relative slope
        target = voxel_down(level(cloud), 0.08)
        tree = cKDTree(target)
        yaw, t = icp(src, target, tree, yaw, t, [0.5, 0.35, 0.25])
        sx, sy, off, rms = roof_tilt(level(cloud), apply(roof, yaw, t))
        print(f'  pass {it + 1}: remaining roof tilt E-W {np.degrees(sx):+.3f} deg, N-S {np.degrees(sy):+.3f} deg, '
              f'roof residual {rms:.3f} m', flush=True)
        if max(abs(sx), abs(sy)) < np.radians(0.02):
            break
    # --- facade lock --------------------------------------------------------------
    # ICP is dominated by the big flat roof, which pins tilt and height but lets the
    # model slide sideways, and the patchy, tree-covered walls don't hold it well.
    # The south facade is the best wall in the scan and has 16 sharp brick/stucco
    # color edges, so use it for yaw (wall direction), depth (wall plane), and the
    # along-wall position (strip edges). Horizontal moves only; tilt/height stay.
    rgb = np.stack([las.red, las.green, las.blue], 1).astype(np.float32)
    scan_v = level(cloud) @ facade.P.T
    brick_z, stucco_z = load_tris('envelope__brick'), load_tris('envelope__stucco')
    model_v = lambda tr: apply(tr.reshape(-1, 3), yaw, t).reshape(-1, 3, 3) @ facade.P.T
    for it in range(5):
        fm = facade.measure(scan_v, rgb, model_v(brick_z), model_v(stucco_z))
        theta, pivot_v, shift_v = facade.correction(fm)
        print(f'  facade pass {it + 1}: along {fm["along_shift"] * 100:+.1f} cm, depth {fm["depth_off"] * 100:+.1f} cm, '
              f'yaw {np.degrees(fm["depth_slope"]):+.3f} deg', flush=True)
        if abs(theta) < np.radians(0.005) and np.linalg.norm(shift_v) < 0.005:
            break
        # rotating about viewer +Y == rotating about Z-up +Z (same sign); see P
        pz, sz = facade.P.T @ pivot_v, facade.P.T @ shift_v
        t = yaw_matrix(theta) @ (t - pz) + pz + sz
        yaw += theta
    facade.report(fm)

    tilt_deg = float(np.degrees(np.arccos(np.clip(R_lev[2, 2], -1, 1))))
    if tilt_deg > 5:
        print(f'WARNING: tilt of {tilt_deg:.1f} deg is suspiciously large - check the debug images')
    t_model = t

    d, _ = tree.query(apply(src, yaw, t))
    within = d < 0.25
    print(f'final: yaw={np.degrees(yaw):.3f} deg  scan tilt correction={tilt_deg:.3f} deg  median dist={np.median(d):.3f} m  '
          f'within 25 cm: {within.mean():.0%}  rms(visible)={np.sqrt((d[within] ** 2).mean()):.3f} m')

    # Viewer world is Y-up: X=east, Y=up, Z=-north, i.e. P @ (x, y, z)_zup with
    # P: (x, y, z) -> (x, z, -y). The GLB is Revit meters in the same Y-up
    # convention, so rotating Revit about +Z by yaw == rotating the GLB about +Y.
    P = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float)
    S = np.eye(4)
    S[:3, :3] = P @ R_lev @ P.T
    S[:3, 3] = P @ (pivot - R_lev @ pivot)
    out = {
        'description': 'model: transform from model.glb (Revit coords, meters, Y-up) into viewer world - apply '
                       'rotation.y then position. scan: 4x4 leveling matrix (column-major, three.js '
                       'Matrix4.fromArray) applied to the point cloud and photo mesh. World origin is in '
                       'pointcloud/meta.json.',
        'model': {
            'rotationY_rad': float(yaw),
            'rotationY_deg': float(np.degrees(yaw)),
            'position': (P @ t_model).tolist(),
        },
        'scan': {
            'matrix': S.T.flatten().tolist(),
            'tilt_deg': float(tilt_deg),
        },
        'fit': {
            'median_dist_m': float(np.median(d)),
            'within_25cm': float(within.mean()),
            # RMS over model surfaces the scan actually captured (excludes hidden walls)
            'rms_visible_m': float(np.sqrt((d[within] ** 2).mean())),
            'facade_along_m': fm['along_shift'],
            'facade_edge_spread_m': fm['along_spread'],
            'facade_depth_m': fm['depth_off'],
            'facade_yaw_deg': float(np.degrees(fm['depth_slope'])),
        },
    }
    json.dump(out, open(OUT, 'w'), indent=2)
    print('wrote', OUT)

    if args.debug:
        model_world = lambda m: m @ yaw_matrix(yaw).T + t_model
        rgb = np.stack([las.red, las.green, las.blue], 1).astype(np.uint8)
        debug_images(args.debug, level(cloud), rgb, model_world(env), model_world(site))


def debug_images(out_dir, cloud, rgb, env, site):
    from PIL import Image
    res = 0.1
    x, y, z = cloud[:, 0], cloud[:, 1], cloud[:, 2]
    m = (x > env[:, 0].min() - 15) & (x < env[:, 0].max() + 15) & (y > env[:, 1].min() - 15) & (y < env[:, 1].max() + 15)
    x, y, z = x[m], y[m], z[m]
    rgb = rgb[m]
    x0, y1 = x.min(), y.max()
    W, H = int((x.max() - x0) / res) + 1, int((y1 - y.min()) / res) + 1
    img = np.zeros((H, W, 3), np.uint8)
    order = np.argsort(z)
    img[((y1 - y[order]) / res).astype(int), ((x[order] - x0) / res).astype(int)] = rgb[order]
    Image.fromarray(img).save(f'{out_dir}/align_cloud.png')
    for name, pts, color in [('env', env, (255, 40, 200)), ('site', site, (40, 220, 255))]:
        i = ((y1 - pts[:, 1]) / res).astype(int); j = ((pts[:, 0] - x0) / res).astype(int)
        ok = (i >= 0) & (i < H) & (j >= 0) & (j < W)
        if name == 'env':
            ov = img.copy()
            ov[i[ok], j[ok]] = (0.4 * ov[i[ok], j[ok]] + 0.6 * np.array(color)).astype(np.uint8)
            Image.fromarray(ov).save(f'{out_dir}/align_overlay.png')
    # 1 m thick sections through the house center: scan in true color, model magenta.
    # A tilted scan shows up as a sloping roof line against the model's flat one.
    res, zt, zb = 0.05, env[:, 2].max() + 3, env[:, 2].min() - 3
    cx, cy = env[:, 0].mean(), env[:, 1].mean()
    allc = np.stack([x, y, z], 1)
    for name, axis, across in [('long', 0, 1), ('cross', 1, 0)]:
        c_mid = (cx, cy)[across]
        lo, hi = env[:, axis].min() - 6, env[:, axis].max() + 6
        img = np.zeros((int((zt - zb) / res) + 1, int((hi - lo) / res) + 1, 3), np.uint8)
        for pts, col in [(allc, None), (env, (255, 40, 200))]:
            s = (np.abs(pts[:, across] - c_mid) < 0.5) & (pts[:, axis] > lo) & (pts[:, axis] < hi) & (pts[:, 2] > zb) & (pts[:, 2] < zt)
            i = ((zt - pts[s, 2]) / res).astype(int); j = ((pts[s, axis] - lo) / res).astype(int)
            img[i, j] = rgb[s] if col is None else col
        Image.fromarray(img).save(f'{out_dir}/align_section_{name}.png')
    print('debug images in', out_dir)


if __name__ == '__main__':
    main()
