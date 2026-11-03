// Revit FBX -> assets/model.glb
//
//   node scripts/convert_model.mjs [data/raw/revit/house.fbx]
//
// Revit's FBX export carried no materials, so every element is colored here by
// its Revit family/type name (see RULES). Elements are also grouped into layers
// (site, envelope, structure, interior, mep) that the viewer can toggle.
//
// Output coordinates: Revit project coordinates converted to meters, Y-up
// (three.js / glTF convention). Alignment to the scan is applied at runtime from
// assets/alignment.json, so the model never needs re-exporting to tweak it.
//
// Also writes data/work/model_tris_{envelope,site}.bin (Float32 triangles,
// meters, Z-up Revit axes) for scripts/align.py.

import fs from 'node:fs';
import path from 'node:path';
import * as THREE from 'three';
import { FBXLoader } from 'three/examples/jsm/loaders/FBXLoader.js';
import { mergeGeometries } from 'three/examples/jsm/utils/BufferGeometryUtils.js';
import { Document, NodeIO } from '@gltf-transform/core';
import { EXTMeshoptCompression, KHRMeshQuantization } from '@gltf-transform/extensions';
import { weld, quantize, reorder, meshopt, dedup, prune } from '@gltf-transform/functions';
import { MeshoptEncoder } from 'meshoptimizer';

globalThis.window ??= { innerWidth: 1920, innerHeight: 1080 }; // FBXLoader reads this for cameras

const FT = 0.3048;
const input = process.argv[2] || 'data/raw/revit/house.fbx';
const outGlb = 'assets/model.glb';
const outTris = 'data/work/model_tris.bin';

// --- Materials ---------------------------------------------------------------
const MATERIALS = {
  grass:     { color: '#6f8a4f' },
  asphalt:   { color: '#45474b' },
  concrete:  { color: '#bdb9b0' },
  brick:     { color: '#a0573f' },
  stucco:    { color: '#e0d8c8' },
  stone:     { color: '#8e8a80' },
  roof:      { color: '#55565a' },
  glass:     { color: '#9cc3d8', alpha: 0.45, metallic: 0.1, roughness: 0.15 },
  door:      { color: '#8c6a48' },
  metal:     { color: '#8f959c', metallic: 0.6, roughness: 0.45 },
  redmetal:  { color: '#b03a30', metallic: 0.4, roughness: 0.5 },
  wallInt:   { color: '#ece8df' },
  porcelain: { color: '#f4f4f2', roughness: 0.3 },
  pipe:      { color: '#6f8da3', metallic: 0.3, roughness: 0.5 },
  equipment: { color: '#a9b0b6', metallic: 0.4, roughness: 0.5 },
  wood:      { color: '#a77b4f' },
  paint:     { color: '#f3efe3' },
  generic:   { color: '#cbc6bc' },
};

// First match wins. [regex on family/type name, layer, material]
const RULES = [
  [/^Toposolid_Grassland/, 'site', 'grass'],
  [/^Toposolid_Path_.*Asphalt/, 'site', 'asphalt'],
  [/^Toposolid/, 'site', 'concrete'],
  [/Parking_Stripe/, 'site', 'paint'],
  [/^Dumpster/, 'site', 'equipment'],
  [/Retaining_-_Stone/, 'site', 'stone'],
  [/^Model_Text/, 'site', 'paint'],

  [/^Basic_Wall_Brick/, 'envelope', 'brick'],
  [/^Basic_Wall_.*Stucco/, 'envelope', 'stucco'],
  [/^Basic_Wall_Concrete/, 'envelope', 'concrete'],
  [/^Basic_Wall_Interior/, 'interior', 'wallInt'],
  [/^Basic_Wall_/, 'envelope', 'generic'],
  [/^Basic_Roof/, 'envelope', 'roof'],
  [/Window/, 'envelope', 'glass'],
  [/LOUVER|Gutter|Downspout|Gable_wall_end_plate/, 'envelope', 'metal'],

  [/^Door/, 'interior', 'door'],
  [/Shower|stall|sink|Sink|Water_Closet|Urinal|Countertop|Floor_Drain/, 'interior', 'porcelain'],
  [/Ceiling/, 'interior', 'paint'],
  [/^Floor_Wood/, 'interior', 'wood'],

  [/Red$/, 'structure', 'redmetal'],
  [/Joist|HSS|W_Shapes|Railing|Handrail|Top_Rail|Picket|Stringer|Carriage/, 'structure', 'metal'],
  [/Stair|Non-Monolithic|^Floor_/, 'structure', 'concrete'],

  [/^Pipe|Bend|Tee|Elbow|Transition|Cap_/, 'mep', 'pipe'],
  [/Fan|Air_|Condensing/, 'mep', 'equipment'],
];
const LAYERS = ['site', 'envelope', 'structure', 'interior', 'mep'];

function classify(name) {
  for (const [re, layer, mat] of RULES) if (re.test(name)) return { layer, mat };
  return { layer: 'structure', mat: 'generic' };
}

// --- Painted faces on exterior walls -----------------------------------------
// Revit's FBX lost the material *definitions*, but each face still carries a
// material slot index (geometry.groups). Faces the architect painted (the stucco
// pilaster strips between the brick panels) show up as a second slot on the
// wall's exterior face. Slot numbers aren't consistent between walls, so per wall:
// on the outermost exterior plane, the biggest slot is the wall's own finish and
// any other slot is paint (stucco). Faces pointing into the building get the
// interior wall color. Positions here are world-space Revit units (feet, Z-up).
function splitWall(g, baseMat, center) {
  const pos = g.attributes.position.array;
  const nTri = pos.length / 9;
  const slot = new Int32Array(nTri).fill(-1);
  for (const gr of g.groups) for (let v = gr.start; v < gr.start + gr.count; v += 3) slot[v / 3] = gr.materialIndex;

  const N = [], C = [], A = new Float64Array(nTri);
  const a = new THREE.Vector3(), b = new THREE.Vector3(), c = new THREE.Vector3();
  for (let t = 0; t < nTri; t++) {
    a.fromArray(pos, t * 9); b.fromArray(pos, t * 9 + 3); c.fromArray(pos, t * 9 + 6);
    const n = b.clone().sub(a).cross(c.clone().sub(a));
    A[t] = n.length() / 2;
    N.push(n.normalize());
    C.push(a.clone().add(b).add(c).divideScalar(3));
  }

  // wall's outward direction: the dominant vertical normal that points away from the building
  const dirs = new Map();
  for (let t = 0; t < nTri; t++) {
    const n = N[t];
    if (Math.abs(n.z) > 0.1 || A[t] === 0) continue;
    if (n.x * (C[t].x - center.x) + n.y * (C[t].y - center.y) <= 0) continue;
    const k = `${Math.round(n.x * 20)},${Math.round(n.y * 20)}`;
    const d = dirs.get(k) || { area: 0, n: new THREE.Vector3() };
    d.area += A[t]; d.n.addScaledVector(n, A[t]); dirs.set(k, d);
  }
  if (!dirs.size) return null;
  const out = [...dirs.values()].sort((p, q) => q.area - p.area)[0].n.normalize();
  let dmax = -Infinity;
  for (let t = 0; t < nTri; t++) if (N[t].dot(out) > 0.99) dmax = Math.max(dmax, N[t].dot(C[t]));
  const isExterior = (t) => N[t].dot(out) > 0.99 && Math.abs(N[t].dot(C[t]) - dmax) < 0.15; // within ~5 cm

  const slotArea = new Map();
  for (let t = 0; t < nTri; t++) if (isExterior(t)) slotArea.set(slot[t], (slotArea.get(slot[t]) || 0) + A[t]);
  const baseSlot = [...slotArea.entries()].sort((p, q) => q[1] - p[1])[0]?.[0];

  const buckets = new Map();
  for (let t = 0; t < nTri; t++) {
    let m = baseMat;
    if (isExterior(t) && slot[t] !== baseSlot) m = 'stucco';
    else if (N[t].dot(out) < -0.99) m = 'wallInt';
    if (!buckets.has(m)) buckets.set(m, []);
    buckets.get(m).push(t);
  }
  const res = new Map();
  for (const [m, tris] of buckets) {
    const arr = new Float32Array(tris.length * 9);
    tris.forEach((t, i) => arr.set(pos.subarray(t * 9, t * 9 + 9), i * 9));
    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.BufferAttribute(arr, 3));
    res.set(m, geom);
  }
  return res;
}

// --- Load FBX ----------------------------------------------------------------
const buf = fs.readFileSync(input);
const root = new FBXLoader().parse(buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength), '');
root.updateMatrixWorld(true);

const familyOf = (o) => {
  let top = o;
  while (top.parent && top.parent !== root) top = top.parent;
  return top.name.replace(/(_\d+)+$/, '');
};
const isExteriorWall = (family) => /^Basic_Wall_/.test(family) && classify(family).layer === 'envelope';

// plan-view center of the building, to tell the outside of a wall from the inside
const buildingCenter = new THREE.Vector3();
{
  const box = new THREE.Box3();
  root.traverse((o) => { if (o.isMesh && isExteriorWall(familyOf(o))) box.expandByObject(o); });
  box.getCenter(buildingCenter);
}

// Revit FBX: Z-up, feet. Convert to Y-up meters: (x, y, z)ft -> (x, z, -y)m.
const toYUpMeters = new THREE.Matrix4().makeRotationX(-Math.PI / 2).premultiply(new THREE.Matrix4().makeScale(FT, FT, FT));

// Roles the viewer toggles: 'wall' (exterior walls -> "Transparent walls") and
// 'roof' (the main roof -> "Hide roof"; the entry awning is left alone).
function roleOf(family) {
  if (isExteriorWall(family)) return 'wall';
  if (/^Basic_Roof/.test(family) && !/Awning/.test(family)) return 'roof';
  return null;
}

const groups = new Map(); // "layer/mat" or "layer/mat#role" -> BufferGeometry[]
const parseKey = (key) => {
  const [lm, role = null] = key.split('#');
  const [layer, mat] = lm.split('/');
  return { layer, mat, role };
};
const unmatched = new Set();
const add = (key, g) => {
  if (!groups.has(key)) groups.set(key, []);
  groups.get(key).push(g);
};
let paintedWalls = 0;
root.traverse((o) => {
  if (!o.isMesh) return;
  const family = familyOf(o);
  const { layer, mat } = classify(family);
  if (mat === 'generic' && layer === 'structure') unmatched.add(family);
  const role = roleOf(family);
  const tag = role ? `#${role}` : '';

  let g = o.geometry.index ? o.geometry.toNonIndexed() : o.geometry.clone();
  for (const k of Object.keys(g.attributes)) if (k !== 'position') g.deleteAttribute(k);
  g.applyMatrix4(o.matrixWorld);

  const parts = isExteriorWall(family) && g.groups.length > 1 ? splitWall(g, mat, buildingCenter) : null;
  if (parts) {
    if (parts.has('stucco') && mat !== 'stucco') paintedWalls++;
    for (const [m, pg] of parts) add(`${layer}/${m}${tag}`, pg);
  } else {
    add(`${layer}/${mat}${tag}`, g);
  }
});
if (unmatched.size) console.warn('unclassified (structure/generic):', [...unmatched]);
console.log(`exterior walls with painted (stucco) faces: ${paintedWalls}`);

// --- Alignment helper output (Z-up meters, exterior-visible stuff only) -------
fs.mkdirSync(path.dirname(outTris), { recursive: true });
for (const layer of ['envelope', 'site']) {
  const parts = [];
  for (const [key, geoms] of groups) {
    if (!key.startsWith(layer + '/')) continue;
    for (const g of geoms) {
      const p = g.attributes.position.array;
      const out = new Float32Array(p.length);
      for (let i = 0; i < p.length; i++) out[i] = p[i] * FT;
      parts.push(Buffer.from(out.buffer));
    }
  }
  fs.writeFileSync(outTris.replace('.bin', `_${layer}.bin`), Buffer.concat(parts));
}
// per-material envelope triangles, for the facade lock (scripts/facade.py)
const byMat = new Map();
for (const [key, geoms] of groups) {
  const { layer, mat } = parseKey(key);
  if (layer !== 'envelope') continue;
  if (!byMat.has(mat)) byMat.set(mat, []);
  for (const g of geoms) byMat.get(mat).push(Buffer.from(Float32Array.from(g.attributes.position.array, (v) => v * FT).buffer));
}
for (const [mat, parts] of byMat) fs.writeFileSync(outTris.replace('.bin', `_envelope__${mat}.bin`), Buffer.concat(parts));

// --- Build glTF ----------------------------------------------------------------
const doc = new Document();
const buffer = doc.createBuffer();
const scene = doc.createScene('house');
const gltfMats = {};
for (const [name, m] of Object.entries(MATERIALS)) {
  const c = new THREE.Color(m.color); // sRGB hex -> linear working space
  const mat = doc.createMaterial(name)
    .setBaseColorFactor([c.r, c.g, c.b, m.alpha ?? 1])
    .setMetallicFactor(m.metallic ?? 0)
    .setRoughnessFactor(m.roughness ?? 0.85)
    .setDoubleSided(true);
  if (m.alpha !== undefined) mat.setAlphaMode('BLEND');
  gltfMats[name] = mat;
}

const layerNodes = {};
for (const layer of LAYERS) {
  layerNodes[layer] = doc.createNode(layer).setExtras({ layer });
  scene.addChild(layerNodes[layer]);
}

const stats = {};
for (const [key, geoms] of groups) {
  const { layer, mat: matName, role } = parseKey(key);
  const merged = mergeGeometries(geoms, false);
  merged.applyMatrix4(toYUpMeters);
  merged.computeVertexNormals(); // flat normals: soup is non-indexed, so each face keeps its own
  const pos = merged.attributes.position.array;
  const nor = merged.attributes.normal.array;
  const prim = doc.createPrimitive()
    .setAttribute('POSITION', doc.createAccessor().setType('VEC3').setArray(new Float32Array(pos)).setBuffer(buffer))
    .setAttribute('NORMAL', doc.createAccessor().setType('VEC3').setArray(new Float32Array(nor)).setBuffer(buffer))
    .setMaterial(gltfMats[matName]);
  const mesh = doc.createMesh(key).addPrimitive(prim);
  const node = doc.createNode(key).setMesh(mesh);
  if (role) node.setExtras({ role }); // three.js GLTFLoader puts this in object.userData
  layerNodes[layer].addChild(node);
  stats[key] = Math.round(pos.length / 9);
}

await MeshoptEncoder.ready;
doc.createExtension(EXTMeshoptCompression).setRequired(true).setEncoderOptions({ method: EXTMeshoptCompression.EncoderMethod.QUANTIZE });
doc.createExtension(KHRMeshQuantization).setRequired(true);
await doc.transform(
  dedup(),
  weld(), // merges identical vertices (same position AND normal), keeps hard edges
  prune(),
  reorder({ encoder: MeshoptEncoder }),
  quantize({ quantizePosition: 16, quantizeNormal: 8 }),
  meshopt({ encoder: MeshoptEncoder, level: 'high' }),
);

fs.mkdirSync('assets', { recursive: true });
await new NodeIO().registerExtensions([EXTMeshoptCompression, KHRMeshQuantization]).registerDependencies({ 'meshopt.encoder': MeshoptEncoder }).write(outGlb, doc);

console.table(stats);
console.log(`wrote ${outGlb} (${(fs.statSync(outGlb).size / 1e6).toFixed(2)} MB) and data/work/ triangles`);
