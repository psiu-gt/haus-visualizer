# Psi U House Visualizer

https://psiu-gt.github.io/haus-visualizer/

> [!CAUTION]
> Slop alert: the viewer is entirely vibe-coded.

A small static website to view the Psi Upsilon Gamma Tau fraternity house from a drone photogrammetry scan made in WebODM.

## Features

- **Flip** (`Space`) between the scan and the model. The camera stays where it is.
- **Blend:** a slider crossfades between the two, so differences stand out.
- **Swipe** (`S`): split screen with a draggable divider, scan on the left and model on the right.
- **Preset views:** Aerial (`1`), Top (`2`), Street (`3`).
- **Transparent walls** (`W`): the model's exterior walls become see-through, so you can see the interior.
- **Hide roof** (`R`): removes the model's roof, to look down into the building. The entry awning stays.
- A controls guide in the top-left corner (touch gestures on phones). It can be dismissed.
- The point cloud streams in progressively, so the house appears before the 27 MB download finishes.

## Running locally

Any static file server works. Opening `index.html` directly from disk won't, because browsers block loading the 3D files over `file://`.

```sh
python -m http.server 8000     # or: npm run serve
```

Then open http://localhost:8000.

## Deploying

The site is plain static files (`index.html`, `src/`, `assets/`), so it can go on GitHub Pages, Netlify, or Cloudflare Pages. three.js loads from the jsDelivr CDN. All the assets together are about 33 MB. The largest file is `assets/pointcloud/points.bin` at 27 MB, under GitHub Pages' 100 MB limit.

## Layout

```
index.html              page + UI markup
src/main.js             viewer (three.js 0.170, no build step)
src/style.css
assets/                 web-ready data, deployed as-is
  pointcloud/points.bin   3.1M points, quantized + shuffled for progressive loading
  pointcloud/meta.json    point count, bounds, UTM origin of the viewer's world
  model.glb               Revit model, meshopt-compressed, grouped into layers
  alignment.json          model placement + scan leveling (see below)
scripts/                processing pipeline (below) + dev helpers
data/raw/               original exports, not committed (see docs/EXPORTING.md)
data/work/              intermediate files, not committed
```

## Rebuilding the assets

You only need to do this if the source exports change. One-time setup:

```sh
npm install
python -m venv .venv && .venv/Scripts/pip install -r requirements.txt   # Windows path; use .venv/bin/ elsewhere
```

Then:

```sh
# 1. unzip WebODM's "All Assets" zip into data/raw/webodm/all/
# 2. point cloud -> assets/pointcloud/
.venv/Scripts/python scripts/process_pointcloud.py
# 3. Revit FBX -> assets/model.glb (+ data/work/ triangles for alignment)
npm run model
# 4. align model and scan -> assets/alignment.json   (~1 min)
.venv/Scripts/python scripts/align.py --debug some/dir   # --debug writes overlay/section PNGs to check by eye
```

After a Revit re-export, steps 3 and 4 are enough.

### How the model is colored

Revit's FBX export didn't include material definitions, so `convert_model.mjs` colors each element by its Revit family/type name (see the `RULES` table there). Faces painted in Revit, such as the stucco pilaster strips on the brick walls, still carry a per-face material slot. The script finds them as a second slot on each wall's exterior face and colors them stucco. The insides of exterior walls get the interior wall color.

The script also tags meshes with a `role` that the viewer's toggles use: exterior walls are `wall` (Transparent walls) and the main roof is `roof` (Hide roof).

### How alignment works

`scripts/align.py`, all automatic:

1. **Yaw + offset.** Scale is known (Revit feet → meters), so the script searches every 1° yaw for the offset where the model's height-above-ground best matches the scan's. The best candidate in each 30° range is then refined with a quick ICP, and the tightest fit wins. Trees along one side of the house fool the height match alone.
2. **Tilt.** Drone GPS georeferencing left the whole scan tipped about **1.6°**, mostly north–south. The Revit model is plumb, so the correction goes on the *scan*: the script fits a plane to the scan-minus-model height difference over the roof, the surface an overhead drone captures best. It then rotates the scan about the roof center until that plane is flat, and re-runs ICP. The model's roof has real drainage slopes, so the comparison is against the model's surface, not against "flat".

3. **Facade lock.** ICP is dominated by the big flat roof, which pins tilt and height but lets the model slide sideways. The walls that should hold it are patchy, and the north wall is buried in trees. So the final horizontal fit comes from the south facade, the best wall in the scan:
   - its plane sets yaw and depth;
   - its 16 brick/stucco color edges set the position along the building.

   This step fixed a 33 cm along-wall offset and a 0.85° yaw error that ICP alone had left.

Current fit:
- **Tilt:** roof residual under 0.01°.
- **South facade:** 0 cm depth and 0 cm along-wall offset. Individual strip edges are within ±12 cm. Two pilaster pairs sit about 10 cm off their designed positions, which looks like a real as-built difference.
- **Other walls:** the recessed stucco entry and the west wall are within 6 cm. The tree-covered north wall and the sparsely scanned east wall read 15–18 cm inside the model.

`alignment.json` holds the model's yaw and position plus a 4×4 leveling matrix that the viewer applies to the point cloud. To re-check the current fit at any time, run `.venv/Scripts/python scripts/facade.py`. It measures in the viewer's frame, using `alignment.json` exactly as the page does.

### Dev helpers

- `scripts/inspect_fbx.mjs FILE`: bounds, triangle counts, and element names in an FBX.
- `npm run screenshot -- OUT_DIR [--mobile]`: drives headless Chrome through each mode and saves screenshots. Needs the local server running.
- In the browser console, `window.viewer` exposes the camera, controls, and scene groups.
