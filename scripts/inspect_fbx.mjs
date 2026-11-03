// Quick look at the Revit FBX: units, bounds, object/material names.
import fs from 'node:fs';
import * as THREE from 'three';
import { FBXLoader } from 'three/examples/jsm/loaders/FBXLoader.js';

globalThis.window ??= { innerWidth: 1920, innerHeight: 1080 }; // FBXLoader reads this for cameras

const file = process.argv[2];
const buf = fs.readFileSync(file);
const ab = buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength);
const root = new FBXLoader().parse(ab, '');

root.updateMatrixWorld(true);
const box = new THREE.Box3().setFromObject(root);
console.log('root scale', root.scale.toArray(), 'rotation', root.rotation.toArray());
console.log('bounds min', box.min.toArray().map((v) => v.toFixed(2)), 'max', box.max.toArray().map((v) => v.toFixed(2)));
console.log('size', box.getSize(new THREE.Vector3()).toArray().map((v) => v.toFixed(2)));

let meshes = 0, tris = 0;
const mats = new Map(), names = new Map();
root.traverse((o) => {
  if (!o.isMesh) return;
  meshes++;
  const g = o.geometry;
  tris += (g.index ? g.index.count : g.attributes.position.count) / 3;
  for (const m of [].concat(o.material)) mats.set(m.name, (mats.get(m.name) || 0) + 1);
  const key = (o.parent?.name || o.name).replace(/\s*\[\d+\]$/, '');
  names.set(key, (names.get(key) || 0) + 1);
});
console.log({ meshes, tris: Math.round(tris) });
console.log('materials', [...mats.entries()].slice(0, 60));
console.log('object name prefixes', [...names.entries()].sort((a, b) => b[1] - a[1]).slice(0, 40));
