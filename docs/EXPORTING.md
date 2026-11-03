# Exporting the source data

Put everything in `data/raw/` in this folder. Those files are only the raw inputs. Processing turns them into the smaller web-ready files in `assets/`.

```
data/raw/
├── webodm/   ← WebODM downloads go here
└── revit/    ← Revit exports go here
```

---

## 1. WebODM (point cloud)

1. Open your project in WebODM and find the finished task.
2. Click **Download Assets**.
3. **Easiest option:** choose **All Assets** (a `.zip`). It includes the point cloud, the textured mesh, and the georeferencing info. Put the zip in `data/raw/webodm/`. You don't need to unzip it.
4. **If All Assets is too large**, download these individually instead:
   - **Point Cloud → LAZ** (required)
   - **Textured Model**: GLB if offered, otherwise the OBJ zip (optional, but nice to have; the site could show the photo-textured mesh as a third view)
   - **Report** (the PDF). It tells me the coordinate system and point density.
5. Optional: if the point cloud includes a lot of neighboring buildings, trees, or street, that's fine. I can crop it to the house. If you'd like to crop it yourself, [CloudCompare](https://www.cloudcompare.org/) (free) works well: *Edit → Segment*, then save as LAZ.

## 2. Revit (model)

### Main export: FBX

1. Open the model and switch to a **3D view**. The default `{3D}` view is fine, but duplicating it first (*right-click → Duplicate View → Duplicate*) gives you a clean view to adjust.
2. In that view:
   - Turn off **Section Box** if it's on (*Properties → Extents*).
   - Open **Visibility/Graphics** (`VV`) and hide things that shouldn't show up as 3D geometry: levels, grids, scope boxes, cameras, reference planes, and anything else in the Annotation tab.
   - Decide about **Topography/Site**. If the model has terrain, leave it on and I can toggle it in the viewer. If it's huge (a whole campus), turn it off.
   - Set the visual style to **Realistic** so materials and colors carry over.
3. **File → Export → FBX**.
   - Leave *Export rooms, areas and spaces as 3D volumes* **unchecked**.
   - Leave *Use Levels of Detail* at the default.
4. Save as `data/raw/revit/house.fbx`. If Revit writes a folder of textures next to it, copy that folder too.

### Also helpful: IFC

**File → Export → IFC**, default settings, saved to `data/raw/revit/house.ifc`.

IFC keeps element information (wall, door, roof, room names), which would make "click a part to see what it is" possible later. It also often includes the model's real-world location, which helps with alignment.

## 3. A few details that make alignment easier

Matching the Revit model to the scan is the trickiest step, so the more of these you can give me, the better. None of them is required.

- **Revit units:** feet/inches or metric? (*Manage → Project Units*)
- **Shared coordinates:** has the model been placed in real-world coordinates (survey point / shared site set up)? If you don't know, that's fine.
- **Which way is "front"**, e.g. "the front door faces the street to the south-east".
- **A few matching points (best case):** 3–4 easy-to-identify spots, such as building corners at ground level. For each one, give its coordinates in Revit (hover or use a spot coordinate) and in the point cloud (click it in WebODM's 3D viewer or in CloudCompare). With these I can compute an exact alignment. Without them, I'll do a rough manual fit and then refine it automatically with ICP.

## 4. Anything else for the page

- The chapter name / school and any text you want on the page (title, a blurb, credits for who flew the drone and who built the model).
- A logo or crest, if you want one. Make sure you're allowed to use it.
- Where you plan to host it (GitHub Pages, Netlify, ...), if you know.

## Checklist

- [ ] `data/raw/webodm/all.zip` (or at least the `.laz` + report)
- [ ] `data/raw/revit/house.fbx` (+ texture folder, if any)
- [ ] `data/raw/revit/house.ifc` (optional)
- [ ] Units, coordinates, and front-facing direction (in chat is fine)
- [ ] Page text / credits (in chat is fine)

When the files are in place, tell me and I'll start processing them and building the viewer.
