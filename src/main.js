import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { MeshoptDecoder } from 'three/addons/libs/meshopt_decoder.module.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';

const ASSETS = 'assets/';
const BASE_POINT_SIZE = 0.06; // meters; scan density is ~5 cm
const WALL_OPACITY = 0.15; // "Transparent walls"

const $ = (id) => document.getElementById(id);

// --- renderer / scene ---------------------------------------------------------
const canvas = $('view');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, powerPreference: 'high-performance' });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));

const scene = new THREE.Scene();
scene.background = new THREE.Color('#15181e');
const pmrem = new THREE.PMREMGenerator(renderer);
scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
scene.environmentIntensity = 0.6;
const sun = new THREE.DirectionalLight('#fff4e0', 2.2);
sun.position.set(-40, 80, 30);
scene.add(sun, new THREE.HemisphereLight('#cfe0ff', '#4a4036', 0.6));

const camera = new THREE.PerspectiveCamera(50, 1, 0.1, 3000);
camera.position.set(60, 60, 90);
const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.maxPolarAngle = Math.PI * 0.495;
controls.minDistance = 1;
controls.maxDistance = 500;
controls.screenSpacePanning = false; // pan along the ground

// scanRoot carries the leveling correction from alignment.json (the drone scan's
// georeferencing is slightly tilted).
const scanRoot = new THREE.Group();
scanRoot.matrixAutoUpdate = false;
const model = new THREE.Group();
scene.add(scanRoot, model);

// --- compositing (blend) ----------------------------------------------------------
const rtOpts = { type: THREE.HalfFloatType, samples: 4 };
const rtScan = new THREE.WebGLRenderTarget(1, 1, rtOpts);
const rtModel = new THREE.WebGLRenderTarget(1, 1, rtOpts);
const compMat = new THREE.ShaderMaterial({
  uniforms: { tA: { value: rtScan.texture }, tB: { value: rtModel.texture }, uMix: { value: 0 } },
  vertexShader: /* glsl */ `
    varying vec2 vUv;
    void main() { vUv = uv; gl_Position = vec4(position.xy, 0.0, 1.0); }`,
  fragmentShader: /* glsl */ `
    uniform sampler2D tA, tB;
    uniform float uMix;
    varying vec2 vUv;
    void main() {
      gl_FragColor = mix(texture2D(tA, vUv), texture2D(tB, vUv), uMix);
      #include <colorspace_fragment>
    }`,
  depthTest: false,
  depthWrite: false,
});
const compQuad = new THREE.Mesh(new THREE.PlaneGeometry(2, 2), compMat);
compQuad.frustumCulled = false;
const compScene = new THREE.Scene().add(compQuad);
const compCam = new THREE.OrthographicCamera(-1, 1, 1, -1, 0, 1);

// --- state -------------------------------------------------------------------------
const state = {
  mix: 0, // 0 = scan, 1 = model
  mixTarget: null,
  swipe: false,
  split: 0.5,
  wallsTransparent: false,
  roofHidden: false,
  dirty: true,
  fly: null,
};
const requestRender = () => (state.dirty = true);
controls.addEventListener('change', requestRender);

function show(which) {
  scanRoot.visible = which === 'scan';
  model.visible = which === 'model';
}

function render() {
  const w = renderer.domElement.width, h = renderer.domElement.height;
  if (state.swipe) {
    const x = Math.round(state.split * w);
    renderer.setScissorTest(true);
    renderer.setScissor(0, 0, x, h); show('scan'); renderer.render(scene, camera);
    renderer.setScissor(x, 0, w - x, h); show('model'); renderer.render(scene, camera);
    renderer.setScissorTest(false);
  } else if (state.mix <= 0.001 || state.mix >= 0.999) {
    show(state.mix < 0.5 ? 'scan' : 'model');
    renderer.render(scene, camera);
  } else {
    show('scan'); renderer.setRenderTarget(rtScan); renderer.render(scene, camera);
    show('model'); renderer.setRenderTarget(rtModel); renderer.render(scene, camera);
    renderer.setRenderTarget(null);
    compMat.uniforms.uMix.value = state.mix;
    renderer.render(compScene, compCam);
  }
}

// --- point cloud -------------------------------------------------------------------
const pointsMat = new THREE.ShaderMaterial({
  uniforms: { uSize: { value: BASE_POINT_SIZE }, uScale: { value: 1 } },
  vertexShader: /* glsl */ `
    attribute vec3 rgb;
    uniform float uSize, uScale;
    varying vec3 vColor;
    void main() {
      vColor = pow(rgb, vec3(2.2)); // stored as sRGB
      vec4 mv = modelViewMatrix * vec4(position, 1.0);
      gl_Position = projectionMatrix * mv;
      gl_PointSize = clamp(uSize * uScale / -mv.z, 1.0, 14.0);
    }`,
  fragmentShader: /* glsl */ `
    varying vec3 vColor;
    void main() {
      vec2 c = gl_PointCoord - 0.5;
      if (dot(c, c) > 0.25) discard;
      gl_FragColor = vec4(vColor, 1.0);
      #include <colorspace_fragment>
    }`,
});

async function loadPoints(meta, onProgress) {
  const N = meta.count, C = meta.chunk_size;
  const pos = new Uint16Array(N * 3), col = new Uint8Array(N * 3);
  const geom = new THREE.BufferGeometry();
  const posAttr = new THREE.BufferAttribute(pos, 3, true);
  const colAttr = new THREE.BufferAttribute(col, 3, true);
  geom.setAttribute('position', posAttr);
  geom.setAttribute('rgb', colAttr);
  geom.setDrawRange(0, 0);

  const pts = new THREE.Points(geom, pointsMat);
  pts.frustumCulled = false;
  const lo = new THREE.Vector3(...meta.bbox_min), hi = new THREE.Vector3(...meta.bbox_max);
  pts.position.copy(lo); // positions are quantized to [0,1] within the bbox
  pts.scale.copy(hi.clone().sub(lo));
  scanRoot.add(pts);

  const res = await fetch(ASSETS + 'pointcloud/points.bin');
  if (!res.ok) throw new Error(`points.bin: HTTP ${res.status}`);
  const total = N * 9;
  const buf = new Uint8Array(total);
  const reader = res.body.getReader();
  let got = 0, parsed = 0;
  for (;;) {
    const { done, value } = await reader.read();
    if (done) break;
    buf.set(value.subarray(0, Math.max(0, total - got)), got);
    got = Math.min(total, got + value.length);
    const before = parsed;
    while (parsed < N) {
      const n = Math.min(C, N - parsed), start = parsed * 9;
      if (got < start + n * 9) break;
      pos.set(new Uint16Array(buf.buffer, start, n * 3), parsed * 3);
      col.set(buf.subarray(start + n * 6, start + n * 9), parsed * 3);
      parsed += n;
    }
    if (parsed > before) {
      posAttr.addUpdateRange(before * 3, (parsed - before) * 3);
      colAttr.addUpdateRange(before * 3, (parsed - before) * 3);
      posAttr.needsUpdate = colAttr.needsUpdate = true;
      geom.setDrawRange(0, parsed);
      requestRender();
    }
    onProgress(got / total);
  }
  return pts;
}

// --- glTF ------------------------------------------------------------------------
const gltfLoader = new GLTFLoader().setMeshoptDecoder(MeshoptDecoder);
const loadGltf = (url, onProgress) =>
  new Promise((resolve, reject) =>
    gltfLoader.load(url, resolve, (e) => e.total && onProgress?.(e.loaded / e.total), reject));

const layerNodes = {};
const wallMeshes = [], roofMeshes = []; // tagged by scripts/convert_model.mjs (node extras "role")
async function loadModel(alignment, onProgress) {
  const gltf = await loadGltf(ASSETS + 'model.glb', onProgress);
  model.add(gltf.scene);
  model.rotation.y = alignment.model.rotationY_rad;
  model.position.fromArray(alignment.model.position);
  for (const child of gltf.scene.children) if (child.userData.layer) layerNodes[child.userData.layer] = child;
  gltf.scene.traverse((o) => {
    const role = o.userData.role;
    if (!role) return;
    o.traverse((m) => {
      if (!m.isMesh) return;
      if (role === 'wall') {
        m.material = m.material.clone(); // materials are shared with non-wall parts (e.g. concrete)
        wallMeshes.push(m);
      } else if (role === 'roof') {
        roofMeshes.push(m);
      }
    });
  });
  applyModelToggles();
}

function applyModelToggles() {
  for (const m of wallMeshes) {
    const t = state.wallsTransparent;
    m.material.transparent = t;
    m.material.opacity = t ? WALL_OPACITY : 1;
    m.material.depthWrite = !t; // let the interior show through
    m.material.needsUpdate = true;
  }
  for (const m of roofMeshes) m.visible = !state.roofHidden;
  $('wallsBtn').setAttribute('aria-pressed', state.wallsTransparent);
  $('roofBtn').setAttribute('aria-pressed', state.roofHidden);
  requestRender();
}

function toggleModel(key) {
  state[key] = !state[key];
  applyModelToggles();
  if (state.mix < 0.5 && !state.swipe) animateMix(1); // these only affect the model, so show it
}

// --- camera views -----------------------------------------------------------------
const views = {};
function computeViews() {
  const box = new THREE.Box3();
  if (layerNodes.envelope) {
    model.updateMatrixWorld(true);
    box.setFromObject(layerNodes.envelope);
  } else {
    box.set(new THREE.Vector3(-20, 0, -15), new THREE.Vector3(20, 12, 15));
  }
  const c = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3());
  // back off on narrow (portrait) screens so the whole house still fits
  const r = Math.max(size.x, size.z) * Math.max(1, 0.9 / camera.aspect);
  const ground = box.min.y;
  // world +Z is south, +X is east
  views.aerial = { pos: c.clone().add(new THREE.Vector3(0.55 * r, 0.6 * r, 0.95 * r)), target: c.clone() };
  views.top = { pos: c.clone().add(new THREE.Vector3(0, 1.7 * r, 0.01)), target: c.clone() };
  views.street = {
    pos: new THREE.Vector3(c.x - 0.12 * r, ground + 3, c.z + 0.8 * r),
    target: new THREE.Vector3(c.x, ground + size.y * 0.45, c.z),
  };
}

function flyTo(name, instant = false) {
  computeViews(); // depends on aspect ratio, which may have changed
  const v = views[name];
  if (!v) return;
  if (instant) {
    camera.position.copy(v.pos);
    controls.target.copy(v.target);
    controls.update();
    requestRender();
    return;
  }
  state.fly = { t0: performance.now(), dur: 1100, fromPos: camera.position.clone(), fromTarget: controls.target.clone(), ...v };
}

const ease = (t) => (t < 0.5 ? 4 * t * t * t : 1 - (-2 * t + 2) ** 3 / 2);

// --- UI ----------------------------------------------------------------------------
const mixInput = $('mix');
const ends = document.querySelectorAll('.blend .end');

function setMix(v) {
  state.mix = v;
  mixInput.value = v;
  ends[0].classList.toggle('active', v < 0.5);
  ends[1].classList.toggle('active', v >= 0.5);
  requestRender();
}
const animateMix = (to) => (state.mixTarget = { from: state.mix, to, t0: performance.now(), dur: 650 });
const flip = () => {
  if (state.swipe) setSwipe(false);
  animateMix((state.mixTarget?.to ?? state.mix) < 0.5 ? 1 : 0);
};

mixInput.addEventListener('input', () => { state.mixTarget = null; setMix(+mixInput.value); });
ends.forEach((b) => b.addEventListener('click', () => { if (state.swipe) setSwipe(false); animateMix(+b.dataset.to); }));
$('flip').addEventListener('click', flip);

const swipeEl = $('swipe');
function setSwipe(on) {
  state.swipe = on;
  swipeEl.hidden = !on;
  $('swipeBtn').setAttribute('aria-pressed', on);
  $('blendRow').style.opacity = on ? 0.4 : 1;
  requestRender();
}
function setSplit(x) {
  state.split = Math.min(0.98, Math.max(0.02, x));
  swipeEl.style.setProperty('--split', `${state.split * 100}%`);
  requestRender();
}
$('swipeBtn').addEventListener('click', () => setSwipe(!state.swipe));
const handle = swipeEl.querySelector('.swipe-handle');
handle.addEventListener('pointerdown', (e) => {
  handle.setPointerCapture(e.pointerId);
  const move = (ev) => setSplit(ev.clientX / innerWidth);
  const up = () => { handle.removeEventListener('pointermove', move); handle.removeEventListener('pointerup', up); };
  handle.addEventListener('pointermove', move);
  handle.addEventListener('pointerup', up);
});

document.querySelectorAll('.view').forEach((b) => b.addEventListener('click', () => flyTo(b.dataset.view)));

// controls guide: dismissible, remembered per browser
const help = $('help'), helpOpen = $('helpOpen');
function setHelp(visible) {
  help.hidden = !visible;
  helpOpen.hidden = visible;
  try { localStorage.setItem('psiu.help', visible ? '1' : '0'); } catch {}
}
try { if (localStorage.getItem('psiu.help') === '0') setHelp(false); } catch {}
$('helpClose').addEventListener('click', () => setHelp(false));
helpOpen.addEventListener('click', () => setHelp(true));

$('wallsBtn').addEventListener('click', () => toggleModel('wallsTransparent'));
$('roofBtn').addEventListener('click', () => toggleModel('roofHidden'));

addEventListener('keydown', (e) => {
  if (e.target instanceof HTMLInputElement && e.code !== 'Space') return;
  if (e.metaKey || e.ctrlKey || e.altKey) return;
  const actions = {
    Space: flip,
    KeyS: () => setSwipe(!state.swipe),
    Digit1: () => flyTo('aerial'),
    Digit2: () => flyTo('top'),
    Digit3: () => flyTo('street'),
    KeyW: () => toggleModel('wallsTransparent'),
    KeyR: () => toggleModel('roofHidden'),
  };
  if (actions[e.code]) { e.preventDefault(); actions[e.code](); }
});
// stop Space from also "clicking" whichever button has focus
addEventListener('keyup', (e) => { if (e.code === 'Space') e.preventDefault(); });

// --- loading UI ----------------------------------------------------------------------
const loadingEl = $('loading');
function setLoading(text, p) {
  loadingEl.classList.remove('done');
  $('loadText').textContent = p == null ? text : `${text} ${Math.round(p * 100)}%`;
  $('loadBar').style.width = `${(p ?? 0) * 100}%`;
}
const doneLoading = () => loadingEl.classList.add('done');
function showError(err) {
  console.error(err);
  const hint = location.protocol === 'file:' ? ' Open this page through a local web server (see README).' : '';
  setLoading(`Couldn't load: ${err.message}.${hint}`);
}

// --- resize / loop ---------------------------------------------------------------------
function resize() {
  renderer.setSize(innerWidth, innerHeight, false);
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
  const w = renderer.domElement.width, h = renderer.domElement.height;
  rtScan.setSize(w, h);
  rtModel.setSize(w, h);
  pointsMat.uniforms.uScale.value = h / (2 * Math.tan(THREE.MathUtils.degToRad(camera.fov) / 2));
  requestRender();
}
addEventListener('resize', resize);
resize();

renderer.setAnimationLoop((now) => {
  if (state.mixTarget) {
    const { from, to, t0, dur } = state.mixTarget;
    const t = Math.min(1, (now - t0) / dur);
    setMix(from + (to - from) * ease(t));
    if (t === 1) state.mixTarget = null;
  }
  if (state.fly) {
    const f = state.fly;
    const t = Math.min(1, (now - f.t0) / f.dur), k = ease(t);
    camera.position.lerpVectors(f.fromPos, f.pos, k);
    controls.target.lerpVectors(f.fromTarget, f.target, k);
    if (t === 1) state.fly = null;
    requestRender();
  }
  controls.update();
  if (state.dirty) {
    state.dirty = false;
    render();
  }
});

// handy from the browser console (and used by scripts/screenshot.mjs)
window.viewer = { THREE, camera, controls, scanRoot, model, state, requestRender, setMix, toggleModel };

// --- boot --------------------------------------------------------------------------
(async () => {
  try {
    setLoading('Loading', 0);
    const [m, alignment] = await Promise.all([
      fetch(ASSETS + 'pointcloud/meta.json').then((r) => r.json()),
      fetch(ASSETS + 'alignment.json').then((r) => r.json()),
    ]);
    const meta = m;
    if (alignment.scan?.matrix) {
      scanRoot.matrix.fromArray(alignment.scan.matrix);
      scanRoot.matrixWorldNeedsUpdate = true;
    }

    let pP = 0, pM = 0;
    const progress = () => setLoading('Loading scan & model', 0.8 * pP + 0.2 * pM);
    const modelP = loadModel(alignment, (p) => { pM = p; progress(); }).then(() => flyTo('aerial', true));
    const pointsP = loadPoints(meta, (p) => { pP = p; progress(); });
    flyTo('aerial', true);
    await Promise.all([modelP, pointsP]);
    doneLoading();
  } catch (err) {
    showError(err);
  }
})();
