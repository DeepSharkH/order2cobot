// Calibration dashboard frontend: one websocket, a three.js scene in the world
// frame (calibration-board centre, z up), camera feeds and calibration cards. See
// dashboard/server.py for the frames and the wire format.

import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';
import { CSS2DRenderer, CSS2DObject } from './vendor/CSS2DRenderer.js';
import { TransformControls } from './vendor/TransformControls.js';
import { GLTFLoader } from './vendor/GLTFLoader.js';
import { UR3eModel } from './ur3e_model.js';
import { HandEModel } from './hande_model.js';
import { SwitchView } from './switch3d.js';
import { ObjectsView } from './objects3d.js';

const C = { bg: 0x111111, cream: 0xfef5da, brass: 0xc9a15d, grey: 0x8c8577, teal: 0x6fb3a8 };
const JN = ['Base', 'Shoulder', 'Elbow', 'Wrist 1', 'Wrist 2', 'Wrist 3'];

const $ = (sel) => document.querySelector(sel);
const esc = (s) => String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
const deg = (r) => `${((r * 180) / Math.PI).toFixed(1)}°`;
const mat4 = (m) => new THREE.Matrix4().set(...m[0], ...m[1], ...m[2], ...m[3]);
const fmt = (v, d = 2) => (v == null ? '—' : Number(v).toFixed(d));

// ── 3D scene ────────────────────────────────────────────────────────────────
const viewport = $('#viewport');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setClearColor(C.bg);
viewport.appendChild(renderer.domElement);
const labels = new CSS2DRenderer();
Object.assign(labels.domElement.style, { position: 'absolute', top: '0', pointerEvents: 'none' });
viewport.appendChild(labels.domElement);

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 50);
camera.up.set(0, 0, 1);
const HOME = { pos: new THREE.Vector3(0.5, -1.0, 0.8), target: new THREE.Vector3(-0.2, 0, 0.1) };
camera.position.copy(HOME.pos);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.target.copy(HOME.target);

scene.add(new THREE.HemisphereLight(0xffffff, 0x2a2622, 1.8));
const sun = new THREE.DirectionalLight(0xffffff, 1.4);
sun.position.set(1, -1.5, 3);
scene.add(sun);
const grid = new THREE.GridHelper(2, 20, 0x2f2f2f, 0x1d1d1d);
grid.rotation.x = Math.PI / 2;
scene.add(grid);
scene.add(new THREE.AxesHelper(0.1));       // the world (board centre): the only axes drawn

function label(text, cls = 'label3d') {
  const el = document.createElement('div');
  el.className = cls;
  el.innerHTML = text;
  return { el, obj: new CSS2DObject(el) };
}
function placed(parent) {
  const g = new THREE.Group();
  g.matrixAutoUpdate = false;
  parent.add(g);
  return g;
}

// robot: baseGroup = T_world_base (on the board plane), the arm in its base frame
const baseGroup = placed(scene);
const robot = new UR3eModel();
baseGroup.add(robot.root);
robot.load().catch((e) => console.error('UR3e meshes failed to load', e));
const armLabel = label('UR3e');
armLabel.obj.position.set(0, 0, 0.62);
baseGroup.add(armLabel.obj);
const flangeG = placed(baseGroup);
const hande = new HandEModel();       // Robotiq Hand-E on the flange
flangeG.add(hande.root);
hande.load().catch((e) => console.error('Hand-E meshes failed to load', e));

// CAD models of the targets (assets/models/cad/*.py) at their cadguide poses T_base_obj
// (state.cad: python -m cadguide register / anchor, or the manual calibration below),
// semi-transparent so the camera clouds and the arm show through: misalignment is visible at
// a glance. g is the rigid pose, its child s the pitch scales about the model's scale origin
// (cadguide.model.CadModel.place), so the drawn grid is the one the targets come from.
const cad = {};
const cadLoader = new GLTFLoader();
async function loadCadModel(name) {
  const g = placed(baseGroup);
  g.visible = false;
  const s = placed(g);
  const gltf = await cadLoader.loadAsync(`./models/cad/${name}.glb`);
  gltf.scene.traverse((o) => {
    if (!o.isMesh) return;
    o.material = o.material.clone();
    Object.assign(o.material, { transparent: true, opacity: 0.45, depthWrite: false });
  });
  s.add(gltf.scene);
  const l = label(name);
  l.obj.position.set(0, 0, { ls108g: 0.04, rack: 0.02 }[name] ?? 0.015);
  g.add(l.obj);
  cad[name] = { g, s, l, key: '' };
  applyCad(lastState?.cad);
}
Promise.all(['ls108g', 'breadboard', 'rack'].map(loadCadModel)).catch((e) => console.error('CAD models', e));     // rack: assets/models/cad/rack.py
let cadOn = true;
// Tr(o) · S · Tr(−o): the pitch scales stretch the model about its scale origin o (object frame)
function setCadScale(m, scale, o) {
  m.s.matrix.makeScale(scale[0], scale[1], 1).setPosition((1 - scale[0]) * o[0], (1 - scale[1]) * o[1], 0);
}
function applyCad(c) {
  for (const [name, m] of Object.entries(cad)) {
    const p = c?.objects?.[name];
    const edited = editor.active && editor.items[name];     // the editor drives the models while open
    m.g.visible = cadOn && !!(p?.T || edited);
    if (edited || !p?.T) continue;
    const key = JSON.stringify([p.T, p.scale, p.kind, p.anchors]);
    if (key === m.key) continue;
    m.key = key;
    m.g.matrix.copy(mat4(p.T));
    setCadScale(m, p.scale, p.scale_origin);
    m.l.el.textContent = name;     // the pose source (manual, cloud, anchors) stays off the label
  }
}

// cameras, drawn like the Hestia dashboard: a short pyramid from the
// intrinsics (optical frame: x right, y down, z forward), an "up" triangle
// above the image's top edge, a faint face and a label.
function frustum(color, name, depth) {
  const g = placed(scene);
  const lines = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color }));
  const face = new THREE.Mesh(new THREE.BufferGeometry(), new THREE.MeshBasicMaterial({
    color, transparent: true, opacity: 0.08, side: THREE.DoubleSide, depthWrite: false }));
  const glyph = new THREE.Group();      // hidden while looking through this camera (the cloud stays)
  glyph.add(lines, face);
  g.add(glyph);
  const l = label(name, 'label3d cam');
  glyph.add(l.obj);
  const canvas = document.createElement('canvas');   // live feed, the background when looking through it
  const tex = new THREE.CanvasTexture(canvas);
  tex.colorSpace = THREE.SRGBColorSpace;
  return { g, glyph, lines, face, canvas, tex, l, depth, K: null, size: null, key: '' };
}
function setFrustum(f, K, size, ortho) {
  const key = JSON.stringify([K, size, ortho]);
  if ((!K && !ortho) || !size || key === f.key) return;
  f.key = key;
  f.K = K;
  f.size = size;
  f.ortho = ortho || null;
  if (ortho) {
    // no mipmaps: they would blend the feed's black edge (see frustumImage) back into a smear
    f.tex.generateMipmaps = false;
    f.tex.minFilter = THREE.LinearFilter;
    return setOrthoGlyph(f);
  }
  const d = f.depth;
  const [w, h] = size;
  const c = (u, v) => new THREE.Vector3(((u - K[0][2]) / K[0][0]) * d, ((v - K[1][2]) / K[1][1]) * d, d);
  const o = new THREE.Vector3();
  const tl = c(0, 0), tr = c(w, 0), br = c(w, h), bl = c(0, h);
  const mid = tl.clone().add(tr).multiplyScalar(0.5);
  const up = mid.clone().add(new THREE.Vector3(0, -0.35 * (br.y - tr.y), 0));
  f.lines.geometry.setFromPoints([o, tl, o, tr, o, br, o, bl, tl, tr, tr, br, br, bl, bl, tl,
    tl.clone().lerp(tr, 0.3), up, up, tl.clone().lerp(tr, 0.7)]);
  f.face.geometry.setFromPoints([tl, tr, br, tl, br, bl]);
  // the geometries started empty: their cached (empty) bounding spheres would cull them forever
  f.lines.geometry.computeBoundingSphere();
  f.face.geometry.computeBoundingSphere();
  f.l.obj.position.copy(up).add(new THREE.Vector3(0, -0.2 * d, 0));
}
// orthographic heightmap (SICK T Mini through Nova): not a pinhole camera and Nova does not
// report where the sensor is, so no glyph and no label; its cloud and its view stay
function setOrthoGlyph(f) {
  f.lines.visible = f.face.visible = f.l.obj.visible = false;
}
function frustumImage(f, img) {
  // a feed wider than the image (T Mini: intensity | depth side by side) keeps its left part
  const w = f.size ? Math.min(img.naturalWidth, Math.round((img.naturalHeight * f.size[0]) / f.size[1])) : img.naturalWidth;
  if (f.canvas.width !== w || f.canvas.height !== img.naturalHeight) {
    f.canvas.width = w;
    f.canvas.height = img.naturalHeight;
    f.tex.dispose();          // new size: re-upload
  }
  const g = f.canvas.getContext('2d');
  g.drawImage(img, 0, 0);
  if (f.ortho) {             // black edge: the letterboxed view clamps to it instead of smearing the image
    g.strokeStyle = '#000';
    g.strokeRect(0.5, 0.5, f.canvas.width - 1, f.canvas.height - 1);
  }
  f.tex.needsUpdate = true;
}
const fcam = frustum(C.grey, 'Fixed D435', 0.07);
const gcam = frustum(C.brass, 'Gripper cam', 0.04);
const tcam = frustum(C.teal, 'SICK ToF', 0.05);      // shown once it has a pose (T_world_tcam)
const cams = [fcam, gcam, tcam];
const CAM_SW = ['sw-cam', 'sw-gcam', 'sw-tcam'];     // legend swatches, same colours as the frustums

// point clouds, each in its camera's optical frame
const clouds = cams.map((f, i) => {
  const c = new THREE.Points(new THREE.BufferGeometry(),
    new THREE.PointsMaterial({ size: i === 0 ? 0.004 : 0.002, vertexColors: true, sizeAttenuation: true }));
  c.frustumCulled = false;
  f.g.add(c);
  return c;
});

function resize() {
  const w = viewport.clientWidth, h = viewport.clientHeight;
  renderer.setSize(w, h);
  labels.setSize(w, h);
  camera.aspect = w / Math.max(h, 1);
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(viewport);

// ── look from a camera ──────────────────────────────────────────────────────
// The 3D camera takes the real camera's pose and field of view, and the live
// feed becomes the background: robot, cloud and board should line up with it.
const view = { cam: null, saved: null };
function showGlyph(f, on) {
  f.glyph.visible = on;
  f.l.obj.visible = on && f.g.visible && !f.ortho;    // CSS2D labels ignore their parents' visibility; no T Mini label
}
const OPT_TO_GL = new THREE.Matrix4().makeRotationX(Math.PI);   // optical (z fwd, y down) -> three (−z fwd, y up)
function setView(which) {       // camera index, or null for the free view
  const f = which == null ? null : cams[which] ?? null;
  if (f && !view.cam) view.saved = { pos: camera.position.clone(), target: controls.target.clone(), up: camera.up.clone() };
  if (view.cam) showGlyph(view.cam, true);
  view.cam = f;
  controls.enabled = !f;
  if (!f && view.saved) {
    camera.up.copy(view.saved.up);
    camera.position.copy(view.saved.pos);
    controls.target.copy(view.saved.target);
    camera.clearViewOffset();
    scene.background = null;
    resize();
  }
  document.querySelectorAll('#view-seg button').forEach((b) => b.classList.toggle('on', b.dataset.view === String(which ?? 'free')));
}
// looking through the heightmap: straight down from far above with a long lens (near
// orthographic). Its rows run along +y, so seen from above the feed is flipped top-bottom;
// the background is its intensity image (frustumImage keeps the left part of the feed).
const ORTHO_D = 10;
function followOrtho(f) {
  const { origin: [ox, oy], pixel: [px, py] } = f.ortho;
  const [w, h] = f.size;
  const look = new THREE.Matrix4().makeBasis(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, -1, 0),
    new THREE.Vector3(0, 0, -1)).setPosition(ox + (w * px) / 2, oy + (h * py) / 2, ORTHO_D);
  f.g.updateWorldMatrix(true, false);
  f.g.matrixWorld.clone().multiply(look).multiply(OPT_TO_GL).decompose(camera.position, camera.quaternion, new THREE.Vector3());
  const vw = viewport.clientWidth, vh = Math.max(viewport.clientHeight, 1);
  const scale = Math.min(vw / w, vh / h);             // screen px per image px
  const tall = (vh / scale) * py;                    // metres seen top to bottom
  camera.fov = (2 * Math.atan(tall / 2 / ORTHO_D) * 180) / Math.PI;
  camera.aspect = vw / vh;
  camera.clearViewOffset();
  camera.updateProjectionMatrix();
  showGlyph(f, false);
  if (f.canvas.width) {
    scene.background = f.tex;
    const sx = (tall * vw) / vh / px / w, sy = (vh / scale) / h;
    f.tex.repeat.set(sx, -sy);
    f.tex.offset.set((1 - sx) / 2, (1 + sy) / 2);
  }
}
function followCamera() {
  const f = view.cam;
  if (f && f.ortho && f.g.visible) return followOrtho(f);
  if (!f || !f.K || !f.g.visible) return;
  f.g.updateWorldMatrix(true, false);
  const m = f.g.matrixWorld.clone().multiply(OPT_TO_GL);
  m.decompose(camera.position, camera.quaternion, new THREE.Vector3());
  const [w, h] = f.size;
  const vw = viewport.clientWidth, vh = Math.max(viewport.clientHeight, 1);
  // vertical FOV from fy; the image is letterboxed into the viewport
  const scale = Math.min(vw / w, vh / h);
  camera.fov = (2 * Math.atan((vh / scale) / 2 / f.K[1][1]) * 180) / Math.PI;
  camera.aspect = vw / vh;
  // principal point offset (pixels in the viewport)
  const ox = (w / 2 - f.K[0][2]) * scale, oy = (h / 2 - f.K[1][2]) * scale;
  camera.setViewOffset(vw, vh, ox, oy, vw, vh);
  camera.updateProjectionMatrix();
  showGlyph(f, false);                    // don't draw the glyph we sit in
  if (f.canvas.width) {
    scene.background = f.tex;
    // letterbox the background like the projection
    const sx = (vw / scale) / w, sy = (vh / scale) / h;
    f.tex.repeat.set(sx, sy);
    f.tex.offset.set((1 - sx) / 2, (1 - sy) / 2);
  }
}
$('#view-seg').onclick = (e) => {
  const b = e.target.closest('button[data-view]');
  if (b && !b.disabled) setView(b.dataset.view === 'free' ? null : Number(b.dataset.view));
};
document.addEventListener('keydown', (e) => {
  if (['INPUT', 'SELECT', 'TEXTAREA'].includes(document.activeElement.tagName)) return;
  const b = /^[1-9]$/.test(e.key) && $(`#view-seg button[data-view="${Number(e.key) - 1}"]`);
  if (b && !b.disabled) setView(Number(e.key) - 1);
  if (e.key === '0' || (e.key === 'Escape' && view.cam)) setView(null);
});
// one view button and legend entry per camera, named after its model, tagged RGB-D / Depth;
// a camera can be looked through once it has a pose and intrinsics
const kindTag = (c) => (c.kind ? `<em class="kind">${esc(c.kind)}</em>` : '');
function ensureViews(list) {
  list.forEach((c, i) => {
    let b = $(`#view-seg button[data-view="${i}"]`);
    if (!b) {
      b = document.createElement('button');
      b.dataset.view = String(i);
      b.innerHTML = `<i class="sw ${CAM_SW[i] || 'sw-cam'}"></i>${esc(c.model)}${kindTag(c)}`;     // its frustum's colour
      $('#view-seg').appendChild(b);
    }
    const why = !c.T_world_cam ? 'not placed in the scene yet' : !c.K && !c.ortho ? 'no pinhole intrinsics' : '';
    b.disabled = !!why || !cams[i];
    b.classList.toggle('na', b.disabled);
    b.title = why ? `${c.name}: ${why}` : `Look through the ${c.name} (${i + 1})`;
    if (b.disabled && view.cam && view.cam === cams[i]) setView(null);
  });
}

(function loop() {
  requestAnimationFrame(loop);
  if (view.cam) followCamera();
  else controls.update();
  renderer.render(scene, camera);
  labels.render(scene, camera);
})();

$('#btn-view').onclick = () => { setView(null); camera.position.copy(HOME.pos); controls.target.copy(HOME.target); };
function toggle(id, fn) {
  const b = $(id);
  b.onclick = () => { b.classList.toggle('on'); fn(b.classList.contains('on')); };
}
toggle('#btn-cloud', (on) => { fuse.cloudOn = on; showClouds(); });
toggle('#btn-cad', (on) => { cadOn = on; applyCad(lastState?.cad); });
toggle('#btn-ovl', (on) => send({ type: 'set_overlays', on }));

// ── camera feeds ────────────────────────────────────────────────────────────
const feeds = [];
function ensureFeeds(list) {
  list.forEach((c, i) => {
    if (feeds[i]) return;
    const el = document.createElement('div');
    el.className = 'feed';
    el.innerHTML = `<div class="zoom"><img alt=""></div><div class="nosig">No signal</div><div class="cap">${esc(c.name)}${kindTag(c)}<span></span></div>`;
    $('#feeds').appendChild(el);
    feeds[i] = { el, img: el.querySelector('img'), nosig: el.querySelector('.nosig'), cap: el.querySelector('.cap span'), url: null };
    zoomable(el, el.querySelector('.zoom'));
  });
}
// wheel to zoom about the pointer, drag to move, double click back to the whole frame
function zoomable(el, inner) {
  let s = 1, x = 0, y = 0, drag = null;
  const apply = () => {
    const w = el.clientWidth, h = el.clientHeight;
    x = Math.min(0, Math.max(w - w * s, x));
    y = Math.min(0, Math.max(h - h * s, y));
    inner.style.transform = s === 1 ? '' : `translate(${x}px, ${y}px) scale(${s})`;
    el.classList.toggle('zoomed', s > 1);
  };
  el.addEventListener('wheel', (e) => {
    e.preventDefault();
    const r = el.getBoundingClientRect(), mx = e.clientX - r.left, my = e.clientY - r.top;
    const n = Math.min(8, Math.max(1, s * Math.exp(-e.deltaY * 0.0015)));
    x = mx - (mx - x) * (n / s);
    y = my - (my - y) * (n / s);
    s = n;
    apply();
  }, { passive: false });
  el.addEventListener('pointerdown', (e) => {
    if (s === 1 || e.button !== 0) return;
    drag = { px: e.clientX, py: e.clientY, x, y };
    el.setPointerCapture(e.pointerId);
  });
  el.addEventListener('pointermove', (e) => {
    if (!drag) return;
    x = drag.x + e.clientX - drag.px;
    y = drag.y + e.clientY - drag.py;
    apply();
  });
  el.addEventListener('pointerup', () => { drag = null; });
  el.addEventListener('dblclick', () => { s = 1; x = y = 0; apply(); });
}
function showFrame(i, blob) {
  const f = feeds[i];
  if (!f) return;
  const url = URL.createObjectURL(blob);
  f.img.onload = () => {
    if (f.url) URL.revokeObjectURL(f.url);
    f.url = url;
    f.el.style.setProperty('--ar', `${f.img.naturalWidth} / ${f.img.naturalHeight}`);
    f.el.style.setProperty('--arn', f.img.naturalWidth / f.img.naturalHeight);
    if (view.cam === cams[i]) frustumImage(cams[i], f.img);
  };
  f.img.src = url;
  f.nosig.classList.add('hidden');
}

// ── cards ───────────────────────────────────────────────────────────────────
// Robot, then one card per calibration step: fixed cam (board PnP), gripper
// cam (base · FK · mount, checked against its board view), robot base.
function buildCards() {
  $('#cards').innerHTML = `
    <div class="card dark" id="c-robot"></div>
    <div class="card hidden" id="c-base">
      <div class="card-head"><div><h3>Robot base</h3><p>Upright, under the board plane</p></div></div>
      <div class="actions"><button id="b-edit">Edit base</button></div>
      <div class="rows" id="base-rows"></div>
      <div class="msg" id="msg"></div>
    </div>
    <div class="card dark" id="c-fuse">
      <div class="card-head"><div><h3>Fused cloud</h3><p>1 cm voxels within 1 m of the board</p></div></div>
      <div class="actions"><button id="b-tint" class="toggle" title="Paint each camera's points in its colour">Colour by camera</button></div>
      <table class="fuse" id="fuse-rows"></table>
      <p class="note" id="fuse-sum"></p>
    </div>`;
  $('#b-edit').onclick = () => (editor.active && editor.sel === 'base' ? editor.close() : editor.open('base'));
  $('#b-tint').onclick = (e) => { fuse.tint = !fuse.tint; e.target.classList.toggle('on', fuse.tint); clouds.forEach((_, i) => paintCloud(i)); };
  $('#fuse-rows').onchange = (e) => {
    const i = Number(e.target.dataset.cam);
    if (Number.isNaN(i)) return;
    fuse.on[i] = e.target.checked;
    showClouds();
    renderFuse(true);
  };
}
buildCards();

const mm = (v) => fmt(v * 1000, 0);

function renderCards(s) {
  const r = s.robot;
  const q = r.q || [];
  $('#c-robot').innerHTML = `
    <div class="card-head"><div><h3>UR3e</h3><p>${esc(r.mode ?? '—')} · ${esc(r.control ?? '—')} · ${esc(r.dh)} DH</p></div>
      <span class="chip ${r.connected ? '' : 'bad'}">${r.connected ? `${fmt(r.rate, 0)} Hz` : 'offline'}</span></div>
    <div class="joints">${JN.map((n, i) => {
      const v = q[i];
      const pct = v == null ? 50 : ((v + 2 * Math.PI) / (4 * Math.PI)) * 100;
      return `<div class="joint"><span class="n">${n}</span><span class="track"><i style="left:${pct}%"></i></span><span class="val">${v == null ? '—' : deg(v)}</span></div>`;
    }).join('')}</div>
    ${r.error ? `<p class="note" style="color:#d2483c">${esc(r.error)}</p>` : ''}`;

  const b = s.base;
  $('#base-rows').innerHTML = `
    <b>Position</b><span>${mm(b.x)}, ${mm(b.y)} mm</span>
    <b>Yaw</b><span>${fmt(b.yaw_deg, 1)}°</span>
    <b>Height</b><span>${fmt((b.z ?? 0) * 1000, 1)} mm vs board</span>`;
  $('#msg').textContent = s.busy ? `${s.busy}…` : s.message;
}

// ── state → scene ───────────────────────────────────────────────────────────
function applyState(s) {
  lastState = s;
  ensureFeeds(s.cams);
  ensureViews(s.cams);
  if (!editor.active) baseGroup.matrix.copy(mat4(s.T_world_base));
  applyCad(s.cad);
  if (s.robot.q) robot.setJoints(s.robot.q);
  if (s.robot.flange) flangeG.matrix.copy(mat4(s.robot.flange));

  s.cams.forEach((c, i) => {
    const f = cams[i];
    if (f) {
      f.g.visible = !!c.T_world_cam;
      if (c.T_world_cam) f.g.matrix.copy(mat4(c.T_world_cam));
      showGlyph(f, view.cam !== f);
      setFrustum(f, c.K, c.size, c.ortho);
    }
    const fd = feeds[i];
    if (fd) {
      fd.cap.textContent = c.ok ? `${c.size?.[0]}×${c.size?.[1]} · ${c.fps} fps` : (c.error || 'offline');
      if (!c.ok) fd.nosig.classList.remove('hidden');
    }
  });
  s.cams.forEach((c, i) => {
    if (cams[i]) cams[i].l.el.textContent = c.model + (i === 0 && !c.located ? ' (not located)' : '');
  });

  const st = [
    ['Robot', s.robot.connected ? 'ok' : 'bad'],
    ['Fixed cam', s.cams[0].ok ? 'ok' : 'bad'],
    ['Gripper cam', s.cams[1].ok ? 'ok' : 'bad'],
    ...(s.cams[2] ? [['ToF cam', s.cams[2].ok ? 'ok' : 'bad']] : []),
    ['Switch link', s.qa?.live?.up ? 'ok' : 'bad'],
  ];
  $('#status').innerHTML = st.map(([n, c]) => `<span class="st ${c}"><i></i>${n}</span>`).join('');
  if (s.mirror) $('#status').insertAdjacentHTML('afterbegin', `<span class="st ok" title="cameras and robot from ${esc(s.mirror)}; calibration is read-only"><i></i>Mirror of ${esc(s.mirror.replace(/^https?:\/\//, ''))}</span>`);
  renderCards(s);
  if (s.qa) renderQA(s.qa);
  if (s.objects) renderObjects(s.objects, s);
}

function applyCloud(buf) {
  const idx = new Uint8Array(buf, 1, 1)[0];
  const cloud = clouds[idx];
  if (!cloud) return;
  objView.setCloud(idx, buf, lastState?.cams[idx]?.T_world_cam);
  const n = new DataView(buf).getUint32(4, true);
  const xyz = new Float32Array(buf, 8, n * 3);
  const rgb = new Uint8Array(buf, 8 + n * 12, n * 3);
  const col = new Float32Array(n * 3);
  for (let i = 0; i < n * 3; i++) col[i] = rgb[i] / 255;
  cloud.userData.rgb = col;
  cloud.geometry.setAttribute('position', new THREE.BufferAttribute(xyz.slice(), 3));
  cloud.geometry.setAttribute('color', new THREE.BufferAttribute(col.slice(), 3));
  paintCloud(idx);
  fuse.vox[idx] = voxels(xyz, lastState?.cams[idx]?.T_world_cam);
  renderFuse();
}

// ── fused cloud: which cameras go in, and what each one adds ────────────────
// Each camera's cloud is placed by its own pose; together they are the fused cloud.
// Coverage is counted in 1 cm world voxels near the board, so the point density of a
// stream (stride, resolution) does not count, only the space it fills.
const CAM_RGB = [C.grey, C.brass, C.teal].map((h) => new THREE.Color(h));
const fuse = { on: [true, true, true], tint: false, cloudOn: true, vox: [], t: 0 };
const VOX = 0.01, REACH = 1.0;
function showClouds() {
  clouds.forEach((c, i) => { c.visible = fuse.cloudOn && fuse.on[i] !== false; });
}
function paintCloud(i) {
  const c = clouds[i], a = c.geometry.getAttribute('color');
  if (!a || !c.userData.rgb) return;
  if (fuse.tint) {
    const k = CAM_RGB[i] || CAM_RGB[0];
    for (let j = 0; j < a.array.length; j += 3) { a.array[j] = k.r; a.array[j + 1] = k.g; a.array[j + 2] = k.b; }
  } else a.array.set(c.userData.rgb);
  a.needsUpdate = true;
}
function voxels(xyz, T) {
  if (!T) return null;
  const s = new Set();
  for (let k = 0; k < xyz.length; k += 3) {
    const x = T[0][0] * xyz[k] + T[0][1] * xyz[k + 1] + T[0][2] * xyz[k + 2] + T[0][3];
    const y = T[1][0] * xyz[k] + T[1][1] * xyz[k + 1] + T[1][2] * xyz[k + 2] + T[1][3];
    const z = T[2][0] * xyz[k] + T[2][1] * xyz[k + 1] + T[2][2] * xyz[k + 2] + T[2][3];
    if (Math.abs(x) > REACH || Math.abs(y) > REACH || z < -0.05 || z > REACH) continue;
    s.add((Math.round(x / VOX) + 2048) * 16777216 + (Math.round(y / VOX) + 2048) * 4096 + (Math.round(z / VOX) + 2048));
  }
  return s;
}
function renderFuse(now = false) {
  if (!lastState || (!now && performance.now() - fuse.t < 1000)) return;
  fuse.t = performance.now();
  const sel = lastState.cams.map((c, i) => fuse.on[i] !== false && fuse.vox[i] && c.T_world_cam ? i : -1).filter((i) => i >= 0);
  const union = new Set();
  sel.forEach((i) => fuse.vox[i].forEach((k) => union.add(k)));
  const only = (i) => {
    let n = 0;
    fuse.vox[i].forEach((k) => { if (!sel.some((j) => j !== i && fuse.vox[j].has(k))) n++; });
    return n;
  };
  $('#fuse-rows').innerHTML = '<tr><th></th><th>voxels</th><th>only it</th></tr>' + lastState.cams.map((c, i) => {
    const v = fuse.vox[i], on = fuse.on[i] !== false;
    const why = !c.T_world_cam ? 'not placed' : !v ? 'no cloud' : '';
    const share = on && v && union.size ? v.size / union.size : 0;
    const col = '#' + (CAM_RGB[i] || CAM_RGB[0]).getHexString();
    return `<tr class="${on && !why ? '' : 'off'}">
      <td><label><input type="checkbox" data-cam="${i}" ${on ? 'checked' : ''}><i class="sw" style="background:${col}"></i>${esc(c.model)}</label>
        <span class="bar"><i style="width:${(share * 100).toFixed(1)}%;background:${col}"></i></span></td>
      <td class="n">${why || v.size.toLocaleString('en-GB')}</td>
      <td class="n">${why || !on ? '—' : `${only(i).toLocaleString('en-GB')}`}</td></tr>`;
  }).join('');
  $('#fuse-sum').textContent = sel.length
    ? `${union.size.toLocaleString('en-GB')} voxels from ${sel.length} camera${sel.length > 1 ? 's' : ''}; the bar is each camera's share of them.`
    : 'No camera selected.';
}

// ── manual calibration ──────────────────────────────────────────────────────
// The robot base (x, y, yaw: upright on the board plane at its calibrated height) and the CAD
// targets (origin x, y, z in the base frame, yaw, pitch; tilted like the plank and, while "On
// plank", at its height) are dragged with the gizmo, nudged by the step or typed in. While the
// editor is open its unsaved poses go to the server (edit_preview), which draws them on the
// feeds: line each box up with its part in every camera. Saving a CAD target makes the pose its
// manual cadguide registration (set_cad_pose; its anchors are dropped).
const gizmo = new TransformControls(camera, renderer.domElement);
gizmo.addEventListener('dragging-changed', (e) => { controls.enabled = !e.value && !view.cam; });
gizmo.setSize(0.8);
scene.add(gizmo.getHelper());

const ZUP = new THREE.Vector3(0, 0, 1);
const R2D = 180 / Math.PI;
const wrapPi = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const yawOf = (q) => {          // atan2(R10, R00), as cadguide.model.yaw_of
  const e = new THREE.Matrix4().makeRotationFromQuaternion(q).elements;
  return Math.atan2(e[1], e[0]);
};
const rows = (m) => [0, 1, 2, 3].map((i) => [0, 4, 8, 12].map((j) => m.elements[i + j]));
const sg = (v, d = 2) => `${v < -0.5 * 10 ** -d ? '−' : '+'}${Math.abs(v).toFixed(d)}`;
const STEPS = [{ mm: 0.1, deg: 0.05, pitch: 0.001 }, { mm: 0.5, deg: 0.2, pitch: 0.005 }, { mm: 2, deg: 1, pitch: 0.02 }];
const DEC = { x: 2, y: 2, z: 2, yaw: 3, px: 4, py: 4 };
const ORIGIN = { ls108g: 'the case bottom centre', breadboard: 'the top face centre', rack: 'the footprint centre on the table' };

// the plank (state.cad.plank: z = a x + b y + c in the world) in the base frame as drawn now
function plankInBase() {
  const p = lastState?.cad?.plank;
  if (!p) return null;
  if (baseGroup.matrixAutoUpdate) baseGroup.updateMatrix();
  const inv = baseGroup.matrix.clone().invert();
  return { c: new THREE.Vector3(0, 0, p.c).applyMatrix4(inv),
    n: new THREE.Vector3(-p.a, -p.b, 1).normalize().transformDirection(inv) };
}

const editor = {
  active: false,
  sel: null,
  items: {},          // base | ls108g | breadboard -> what is being placed and its saved pose
  step: STEPS[1],
  saving: null,
  open(sel = this.sel || 'ls108g') {
    if (!lastState) return;
    this.active = true;
    baseGroup.matrix.decompose(baseGroup.position, baseGroup.quaternion, baseGroup.scale);
    baseGroup.matrixAutoUpdate = true;
    // the base height is fixed (calibrated), only x, y, yaw move
    const b = { x: baseGroup.position.x, y: baseGroup.position.y, yaw: yawOf(baseGroup.quaternion) };
    this.items = { base: { kind: 'base', obj: baseGroup, z: baseGroup.position.z, saved: b, touched: false } };
    for (const [name, m] of Object.entries(cad)) this.items[name] = this.cadItem(name, m);
    document.querySelectorAll('#ed-items button').forEach((bt) => { bt.disabled = !this.items[bt.dataset.item]; });
    $('#editor').classList.remove('hidden');
    $('#b-edit').classList.add('on');
    $('#btn-edit').classList.add('on');
    this.msg('');
    this.select(this.items[sel] ? sel : 'base');
    this.preview(true);
    this.beat = setInterval(() => this.preview(true), 1000);     // the server drops a silent preview
  },
  cadItem(name, m) {
    const p = lastState.cad?.objects?.[name];
    if (!p) return null;
    const g = m.g;
    const T = p.T ? mat4(p.T) : null;
    if (T) T.decompose(g.position, g.quaternion, new THREE.Vector3());
    else { g.position.set(-0.3, 0, 0); g.quaternion.identity(); }   // not registered: in front of the robot
    g.matrixAutoUpdate = true;
    // its rotation = tilt (base z onto the part's z) · Rz(psi), as cadguide places it on the plank
    const n = ZUP.clone().applyQuaternion(g.quaternion);
    const rz = new THREE.Quaternion().setFromUnitVectors(ZUP, n).invert().multiply(g.quaternion);
    const it = { kind: 'cad', name, obj: g, m, info: p, n, psi: 2 * Math.atan2(rz.z, rz.w), yaw: yawOf(g.quaternion),
      z: g.position.z, scale: [...(p.scale || [1, 1])], onPlank: false, touched: false,
      saved: T && { T, scale: [...p.scale] } };
    const pl = plankInBase();
    if (!T) {
      it.onPlank = !!pl;
      this.build(it);
    } else if (pl) {        // a saved pose that stands on the plank stays on it as it moves
      it.onPlank = n.angleTo(pl.n) < 1e-3 && Math.abs(this.plankZ(it, g.quaternion, pl) - it.z) < 2e-4;
    }
    setCadScale(m, it.scale, p.scale_origin);
    m.l.el.innerHTML = `${name} <small>editing</small>`;
    return it;
  },
  close() {
    this.active = false;
    clearInterval(this.beat);
    clearTimeout(this.timer);
    this.timer = null;
    this.saving = null;
    gizmo.detach();
    baseGroup.matrixAutoUpdate = false;
    for (const m of Object.values(cad)) { m.g.matrixAutoUpdate = false; m.key = ''; }
    $('#editor').classList.add('hidden');
    $('#b-edit').classList.remove('on');
    $('#btn-edit').classList.remove('on');
    send({ type: 'edit_preview' });
    if (lastState) {             // back to the saved poses
      baseGroup.matrix.copy(mat4(lastState.T_world_base));
      applyCad(lastState.cad);
    }
  },
  select(id) {
    const it = this.items[id];
    if (!it) return;
    this.sel = id;
    gizmo.attach(it.obj);
    this.mode(gizmo.mode);
    const isCad = it.kind === 'cad';
    document.querySelectorAll('#editor .cad-only').forEach((e) => e.classList.toggle('hidden', !isCad));
    $('#ed-py').classList.toggle('hidden', !isCad || !it.info.pitch_mm[1]);
    $('#editor [data-k="px"]').closest('label').classList.toggle('hidden', !isCad || !it.info.pitch_mm[0]);     // the rack has no pitch
    $('#ed-items').querySelectorAll('button').forEach((b) => b.classList.toggle('on', b.dataset.item === id));
    if (isCad) {
      const p = it.info;
      $('#ed-src').textContent = p.kind ? `${p.kind} registration ${(p.stamp || '').slice(5, 16).replace('T', ' ')} · ${p.anchors} anchor${p.anchors === 1 ? '' : 's'}` : 'not registered';
      $('#ed-hint').textContent = `X / Y / Z: ${ORIGIN[id] || 'the CAD origin'}, robot base frame; pitch in mm. Line the box up with the part in the camera feeds. Saving makes this its manual registration${p.anchors ? ` and drops its ${p.anchors} anchor${p.anchors > 1 ? 's' : ''}` : ''}.`;
    } else {
      $('#ed-src').textContent = `world frame · height ${fmt(it.z * 1000, 1)} mm`;
      $('#ed-hint').textContent = 'Upright at its calibrated height; world origin is the board centre. '
        + 'Line the drawn arm up with the real one in the point clouds and the fixed-cam overlay.';
    }
    this.fields();
  },
  mode(m) {
    gizmo.setMode(m);
    gizmo.showX = gizmo.showY = m === 'translate';     // turns about z only
    gizmo.showZ = m === 'rotate';                      // z by the field, the nudge or the plank
    document.querySelectorAll('#editor [data-mode]').forEach((b) => b.classList.toggle('on', b.dataset.mode === m));
  },
  // the part's rotation and height from its tilt, psi and z (or the plank)
  build(it) {
    if (it.kind === 'base') {
      it.obj.position.z = it.z;
      return;
    }
    const pl = it.onPlank ? plankInBase() : null;
    if (pl) it.n.copy(pl.n);
    const q = new THREE.Quaternion().setFromUnitVectors(ZUP, it.n).multiply(new THREE.Quaternion().setFromAxisAngle(ZUP, it.psi));
    it.obj.quaternion.copy(q);
    if (pl) it.z = this.plankZ(it, q, pl);
    it.obj.position.z = it.z;
    it.yaw = yawOf(q);
  },
  // origin z that puts the part's bottom centre on the plank (cadguide.register.from_manual)
  plankZ(it, q, pl) {
    const b = new THREE.Vector3(0, 0, it.info.bottom_z).applyQuaternion(q);
    const fx = it.obj.position.x + b.x, fy = it.obj.position.y + b.y;
    return pl.c.z - (pl.n.x * (fx - pl.c.x) + pl.n.y * (fy - pl.c.y)) / pl.n.z - b.z;
  },
  get(it, k) {        // shown values: mm, degrees
    const o = it.obj;
    if (k === 'x' || k === 'y' || k === 'z') return o.position[k] * 1000;
    if (k === 'yaw') return yawOf(o.quaternion) * R2D;
    const i = k === 'px' ? 0 : 1;
    return it.kind === 'cad' && it.info.pitch_mm[i] ? it.info.pitch_mm[i] * it.scale[i] : null;
  },
  set(it, k, v) {
    if (!Number.isFinite(v) || (it.kind === 'base' && !['x', 'y', 'yaw'].includes(k))) return;
    const o = it.obj;
    if (k === 'x' || k === 'y') o.position[k] = v / 1000;
    else if (k === 'z') { it.z = v / 1000; it.onPlank = false; }
    else if (k === 'yaw') {
      const d = wrapPi(v / R2D - yawOf(o.quaternion));
      if (it.kind === 'base') o.quaternion.setFromAxisAngle(ZUP, yawOf(o.quaternion) + d);
      else it.psi += d;
    } else {
      const i = k === 'px' ? 0 : 1;
      const s = it.info.pitch_mm[i] ? v / it.info.pitch_mm[i] : NaN;
      if (!(Math.abs(s - 1) <= 0.05)) return;           // the server takes ±5 %
      it.scale[i] = s;
      setCadScale(it.m, it.scale, it.info.scale_origin);
    }
    this.build(it);
    this.changed(it);
  },
  nudge(k, d) {
    const it = this.items[this.sel];
    if (!it) return;
    const v = this.get(it, k);
    if (v != null) this.set(it, k, v + d * (k === 'yaw' ? this.step.deg : k[0] === 'p' ? this.step.pitch : this.step.mm));
  },
  onGizmo() {
    const it = this.items[this.sel];
    if (!it) return;
    if (it.kind === 'cad') it.psi += wrapPi(yawOf(it.obj.quaternion) - it.yaw);   // the turn about z
    else it.obj.quaternion.setFromAxisAngle(ZUP, yawOf(it.obj.quaternion));
    this.build(it);
    this.changed(it);
  },
  togglePlank() {
    const it = this.items[this.sel];
    if (it?.kind !== 'cad') return;
    if (!it.onPlank && !plankInBase()) { this.msg('No plank yet: python -m cadguide plank', true); return; }
    it.onPlank = !it.onPlank;
    this.build(it);
    this.changed(it);
  },
  changed(it) {
    it.touched = true;
    this.fields();
    this.preview();
  },
  isChanged(it) {
    if (!it?.touched) return false;
    if (!it.saved) return true;
    const o = it.obj;
    if (it.kind === 'base') {
      return Math.abs(o.position.x - it.saved.x) > 1e-7 || Math.abs(o.position.y - it.saved.y) > 1e-7
        || Math.abs(wrapPi(yawOf(o.quaternion) - it.saved.yaw)) > 1e-7;
    }
    o.updateMatrix();
    const a = o.matrix.elements, b = it.saved.T.elements;
    return a.some((v, i) => Math.abs(v - b[i]) > 1e-7) || it.scale.some((v, i) => Math.abs(v - it.saved.scale[i]) > 1e-9);
  },
  fields(all = false) {
    const it = this.items[this.sel];
    if (!it) return;
    document.querySelectorAll('#editor input[data-k]').forEach((i) => {
      if (!all && document.activeElement === i) return;
      const v = it.kind === 'base' && !['x', 'y', 'yaw'].includes(i.dataset.k) ? null : this.get(it, i.dataset.k);
      i.value = v == null ? '' : v.toFixed(DEC[i.dataset.k]);
    });
    $('#ed-plank').classList.toggle('on', !!it.onPlank);
    $('#ed-items').querySelectorAll('button').forEach((b) => b.classList.toggle('dirty', this.isChanged(this.items[b.dataset.item])));
    $('#ed-delta').textContent = this.delta(it);
  },
  delta(it) {
    if (!it.saved) return 'Not registered yet: place it, then save';
    const o = it.obj;
    let p, yaw, out;
    if (it.kind === 'base') {
      p = new THREE.Vector3(it.saved.x, it.saved.y, it.z);
      yaw = it.saved.yaw;
    } else {
      p = new THREE.Vector3().setFromMatrixPosition(it.saved.T);
      yaw = yawOf(new THREE.Quaternion().setFromRotationMatrix(it.saved.T));
    }
    const d = o.position.clone().sub(p).multiplyScalar(1000);
    out = `Δ saved  x ${sg(d.x)}  y ${sg(d.y)}${it.kind === 'cad' ? `  z ${sg(d.z)}` : ''} mm  yaw ${sg(wrapPi(yawOf(o.quaternion) - yaw) * R2D, 3)}°`;
    if (it.kind === 'cad' && it.scale.some((v, i) => Math.abs(v - it.saved.scale[i]) > 1e-9)) {
      out += `  pitch ${it.scale.map((v, i) => sg((v / it.saved.scale[i] - 1) * 100, 3)).slice(0, it.info.pitch_mm[1] ? 2 : 1).join(' / ')} %`;
    }
    return out;
  },
  // the unsaved poses to the server, which draws them on the feeds (at most ~15 Hz)
  preview(now = false) {
    if (!this.active) return;
    if (!now) {
      this.timer ??= setTimeout(() => { this.timer = null; this.preview(true); }, 60);
      return;
    }
    const out = { type: 'edit_preview', cad: {} };
    const b = this.items.base;
    if (this.isChanged(b)) out.base = { x: b.obj.position.x, y: b.obj.position.y, yaw_deg: yawOf(b.obj.quaternion) * R2D };
    for (const it of Object.values(this.items)) {
      if (it?.kind !== 'cad' || !this.isChanged(it)) continue;
      it.obj.updateMatrix();
      out.cad[it.name] = { T: rows(it.obj.matrix), scale: it.scale };
    }
    send(out);
  },
  revert() {
    const it = this.items[this.sel];
    if (!it) return;
    if (it.kind === 'base') {
      it.obj.position.set(it.saved.x, it.saved.y, it.z);
      it.obj.quaternion.setFromAxisAngle(ZUP, it.saved.yaw);
      it.touched = false;
    } else this.items[it.name] = this.cadItem(it.name, it.m);   // the saved pose, as the server has it now
    this.select(this.sel);
    this.preview(true);
  },
  save() {
    const todo = Object.values(this.items).filter((it) => this.isChanged(it));
    if (!todo.length) { this.msg('Nothing changed'); return; }
    if (ws?.readyState !== 1) { this.msg('Not connected', true); return; }
    const drop = todo.filter((it) => it.kind === 'cad' && it.info.anchors);
    if (drop.length && !confirm(`Saving makes the hand-placed pose the registration and drops the anchors of ${drop.map((it) => `${it.info.label} (${it.info.anchors})`).join(', ')}. Save?`)) return;
    this.saving = { left: todo.length, msgs: [], ok: true };
    for (const it of todo) {
      if (it.kind === 'base') {
        send({ type: 'set_base', x: it.obj.position.x, y: it.obj.position.y, yaw: yawOf(it.obj.quaternion) * R2D });
      } else {
        it.obj.updateMatrix();
        send({ type: 'set_cad_pose', name: it.name, T: rows(it.obj.matrix), scale: it.scale });
      }
    }
    this.msg('Saving…');
  },
  onSaved(m) {
    const sv = this.saving;
    if (!sv) return;
    sv.left -= 1;
    sv.msgs.push(m.message);
    sv.ok &&= m.ok;
    if (sv.left > 0) return;
    this.saving = null;
    this.msg(sv.msgs.join('\n'), !sv.ok);
    if (sv.ok) setTimeout(() => { if (this.active && !this.saving) this.close(); }, 1500);
  },
  msg(text, err = false) {
    const el = $('#ed-msg');
    el.textContent = text;
    el.classList.toggle('err', err);
  },
};
gizmo.addEventListener('objectChange', () => editor.onGizmo());
$('#btn-edit').onclick = () => (editor.active ? editor.close() : editor.open());
$('#ed-items').onclick = (e) => { const b = e.target.closest('button[data-item]'); if (b && !b.disabled) editor.select(b.dataset.item); };
document.querySelectorAll('#editor [data-mode]').forEach((b) => { b.onclick = () => editor.mode(b.dataset.mode); });
$('#ed-plank').onclick = () => editor.togglePlank();
$('#ed-step').onclick = (e) => {
  const b = e.target.closest('button');
  if (!b) return;
  editor.step = STEPS[Number(b.dataset.step)];
  document.querySelectorAll('#ed-step button').forEach((x) => x.classList.toggle('on', x === b));
};
$('#ed-nudge').onclick = (e) => { const b = e.target.closest('button[data-k]'); if (b) editor.nudge(b.dataset.k, Number(b.dataset.d)); };
document.querySelectorAll('#editor input[data-k]').forEach((i) => {
  i.oninput = () => { const it = editor.items[editor.sel]; if (it) editor.set(it, i.dataset.k, parseFloat(i.value)); };
  i.onblur = () => editor.fields();
  i.onkeydown = (e) => {           // ↑ / ↓ nudge the field by the step (shift: ×10)
    if (e.key !== 'ArrowUp' && e.key !== 'ArrowDown') return;
    e.preventDefault();
    editor.nudge(i.dataset.k, (e.key === 'ArrowUp' ? 1 : -1) * (e.shiftKey ? 10 : 1));
    editor.fields(true);
  };
});
$('#ed-revert').onclick = () => editor.revert();
$('#ed-cancel').onclick = () => editor.close();
$('#ed-save').onclick = () => editor.save();
// keys: E opens it; then M / R gizmo mode, P on plank, arrows x / y, PgUp / PgDn z, [ / ] yaw
// (shift: ×10 the step), Esc closes
const NUDGE_KEYS = { ArrowLeft: ['x', -1], ArrowRight: ['x', 1], ArrowDown: ['y', -1], ArrowUp: ['y', 1],
  PageDown: ['z', -1], PageUp: ['z', 1], '[': ['yaw', -1], ']': ['yaw', 1], '{': ['yaw', -1], '}': ['yaw', 1] };
document.addEventListener('keydown', (e) => {
  if (['INPUT', 'SELECT', 'TEXTAREA'].includes(document.activeElement.tagName) || e.ctrlKey || e.metaKey || e.altKey) return;
  const k = e.key.toLowerCase();
  if (!editor.active) {
    if (k === 'e' && !$('#tab-calib').classList.contains('hidden')) editor.open();
    return;
  }
  if (k === 'm') editor.mode('translate');
  else if (k === 'r') editor.mode('rotate');
  else if (k === 'p') editor.togglePlank();
  else if (e.key === 'Escape') editor.close();
  else if (NUDGE_KEYS[e.key]) {
    e.preventDefault();
    editor.nudge(NUDGE_KEYS[e.key][0], NUDGE_KEYS[e.key][1] * (e.shiftKey ? 10 : 1));
  }
});
// click a part (or the arm, for the base) in the 3D view to select it
const ray = new THREE.Raycaster();
renderer.domElement.addEventListener('pointerdown', (e) => {
  if (!editor.active || gizmo.dragging || gizmo.axis) return;
  const r = renderer.domElement.getBoundingClientRect();
  ray.setFromCamera(new THREE.Vector2(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1), camera);
  const hits = [...Object.entries(cad).map(([n, m]) => [n, m.g]), ['base', robot.root]]
    .filter(([n, o]) => editor.items[n] && o.visible)
    .map(([n, o]) => [n, ray.intersectObject(o, true)[0]])
    .filter(([, h]) => h)
    .sort((a, b) => a[1].distance - b[1].distance);
  if (hits.length) editor.select(hits[0][0]);
});


// ── tabs ────────────────────────────────────────────────────────────────────
// one set of camera feeds, moved into whichever tab is showing
function showTab(name, project) {
  for (const t of ['calib', 'qa', 'cams', 'obj', 'circuit', 'exp', 'projects']) $(`#tab-${t}`).classList.toggle('hidden', t !== name);
  document.querySelectorAll('#tabs button').forEach((b) => b.classList.toggle('on', b.dataset.tab === name));
  const home = { calib: '#calib-cams', qa: '#qa-cams', cams: '#cams-only', obj: '#obj-cams', circuit: '#ca-feeds' }[name];
  if (home) $(home).appendChild($('#feeds'));
  if (name === 'projects') {      // the projects page (web/projects/), loaded on first open
    const f = $('#projects-frame');
    if (!f.src || project) f.src = `/projects?embed=1${project ? `&id=${encodeURIComponent(project)}` : ''}`;
  }
  if (name === 'exp') expShow(expOpen, expWithTrace);
  else if (name === 'projects' && location.hash.startsWith('#projects')) { /* keeps #projects:<id> */ }
  else if (!(name === 'circuit' && location.hash.startsWith('#circuit'))) {   // keeps #circuit:<design>
    history.replaceState(null, '', name === 'projects' ? location.pathname : `#${name === 'calib' ? 'environment' : name}`);     // Projects is the home tab
  }
}
$('#tabs').onclick = (e) => { const b = e.target.closest('button'); if (b) showTab(b.dataset.tab); };
// the open project, from the projects page, goes into the address: #projects:<id>
window.addEventListener('message', (e) => {
  if (e.origin !== location.origin || !('project' in (e.data || {}))) return;
  if (!$('#tab-projects').classList.contains('hidden')) history.replaceState(null, '', e.data.project ? `#projects:${e.data.project}` : '#projects');
});

// the simulated arm (python -m simulation): simulation/web/sim_tab.js, loaded on first open
let simMod = null;
function simTab(on) {
  if (on && !simMod) simMod = import('/simweb/sim_tab.js').then((m) => { m.mount($('#tab-sim')); return m; });
  simMod?.then((m) => (on ? m.show() : m.hide()));
}

// ── port QA ─────────────────────────────────────────────────────────────────
let qaSel = 1;
let qaLast = null;
let qaVlmT = null;                // follow the VLM's port each time it reports a new one
const VLM_FRESH_S = 5;
const sw3d = new SwitchView($('#qa-3d'), { onPick: (p) => { qaSel = p; if (qaLast) renderQA(qaLast); } });
const qaDet = (r) => {
  if (!r) return '—';
  if (r.want === 'down') return r.ok ? `down ${fmt(r.t_link)} s` : 'stays up';
  return r.speed ? `${r.speed} Mb/s${r.ok ? ` · ${fmt(r.t_link)} s` : ''}` : 'no link';
};
// rebuild only on change: state arrives at 10 Hz and replacing a button mid-click drops the click
function setHTML(sel, html) {
  const el = $(sel);
  if (el.dataset.h !== html) { el.innerHTML = html; el.dataset.h = html; }
}
function renderQA(q) {
  qaLast = q;
  const L = q.live || {};
  const busyPort = q.busy ? +q.busy.split(' ')[1] : null;
  const vlm = q.vlm ? { ...q.vlm, fresh: q.vlm.age < VLM_FRESH_S } : null;
  if (vlm && vlm.t !== qaVlmT) { qaVlmT = vlm.t; qaSel = vlm.port; }
  sw3d.update({ ports: q.ports, sel: qaSel, active: busyPort ?? (vlm?.fresh ? vlm.port : null),
    testing: !!q.busy, link: L, vlm });
  setHTML('#qa-link', `
    <div class="v ${L.up ? '' : 'alert'}">${L.error ? 'No interface' : L.up ? `Link up · ${L.speed ?? '?'} Mb/s` : 'No link'}</div>`);
  // one row per port: a light (grey untested, green pass, red fail, brass while checking)
  setHTML('#qa-ports', q.ports.map((p) => `
    <button class="prow ${p.port === busyPort ? 'testing' : p.status} ${p.port === qaSel ? 'sel' : ''}" data-port="${p.port}">
      <i></i><span>Port ${p.port}</span><em>${p.port === busyPort ? 'checking' : p.status === 'untested' ? '' : esc(p.status)}</em>
    </button>`).join(''));

  const p = q.ports[qaSel - 1];
  const res = (name, r) => `<b>${name}</b><span class="${r && !r.ok ? 'alert' : ''}">${!r ? '—' : `${r.ok ? 'pass' : 'fail'} · ${qaDet(r)}`}</span>`;
  setHTML('#qa-c-port', `
    <div class="card-head"><div><h3>Port ${qaSel}</h3><p>${p.stamp ? esc(p.stamp.replace('T', ' ').slice(0, 16)) : 'not tested'}</p></div>
      <span class="chip ${p.status === 'fail' ? 'bad' : p.status === 'untested' ? 'quiet' : ''}">${esc(p.status)}</span></div>
    <div class="rows">
      ${res('Insert', p.insert)}${res('Extract', p.extract)}
      ${vlm ? `<b>Camera</b><span class="${vlm.fresh ? '' : 'alert'}">port ${vlm.port}${vlm.confidence != null ? ` · ${Math.round(vlm.confidence * 100)}%` : ''}</span>` : ''}
      ${p.insert?.flaps ? `<b>Flaps</b><span class="alert">${p.insert.flaps}</span>` : ''}</div>`);

  const busy = !!q.busy;
  for (const id of ['#qa-insert', '#qa-extract', '#qa-reset-port', '#qa-reset-all']) $(id).disabled = busy;
}
$('#qa-ports').onclick = (e) => {
  const b = e.target.closest('.prow');
  if (!b) return;
  qaSel = +b.dataset.port;
  if (qaLast) renderQA(qaLast);
};
$('#qa-insert').onclick = () => send({ type: 'qa_test', port: qaSel, phase: 'insert' });
$('#qa-extract').onclick = () => send({ type: 'qa_test', port: qaSel, phase: 'extract' });
$('#qa-reset-port').onclick = () => { if (confirm(`Clear the results of port ${qaSel}?`)) send({ type: 'qa_reset', port: qaSel }); };
$('#qa-reset-all').onclick = () => { if (confirm('Clear the results of all ports?')) send({ type: 'qa_reset' }); };

// ── objects ─────────────────────────────────────────────────────────────────
// bounding box of a configured object (configs/wire/objects.yaml) in the live clouds;
// the server runs the search (dashboard/objects.py) and sends the result with the state
let lastState = null;
const objView = new ObjectsView($('#obj-3d'));
let objSel = null;
let objFocused = null;
const objBusy = (s) => !!s.busy && s.busy.startsWith('locating');
const mmv = (v, d = 1) => fmt(v * 1000, d);

function renderObjects(o, s) {
  if (!objSel && o.specs.length) objSel = o.specs[0].name;
  setHTML('#obj-sel', o.specs.map((p) => `<option value="${esc(p.name)}" ${p.name === objSel ? 'selected' : ''}>${esc(p.label)}</option>`).join(''));
  const spec = o.specs.find((p) => p.name === objSel);
  const r = o.results[objSel] || null;
  objView.setMesh(spec?.mesh ?? null);
  objView.setResult(r);
  if (r && objFocused !== r.t) { objFocused = r.t; if (!$('#tab-obj').classList.contains('hidden')) objView.focus(); }

  const busy = objBusy(s);
  $('#obj-locate').disabled = !!s.busy || !spec;
  $('#obj-chip').textContent = busy ? 'searching' : r ? new Date(r.t * 1000).toLocaleTimeString('en-GB') : 'not located';
  const msg = $('#obj-msg');
  msg.textContent = busy ? `${s.busy}… (cloud search, then the camera frames if needed)` : (s.message.startsWith('locating') ? s.message : (r ? '' : 'Press Locate to search both depth clouds.'));
  msg.classList.toggle('alert', !busy && s.message.startsWith('locating') && s.message.includes('failed'));

  if (!spec) { setHTML('#obj-c-box', ''); setHTML('#obj-c-fit', ''); return; }
  if (!r) {
    setHTML('#obj-c-box', `
      <div class="card-head"><div><h3>${esc(spec.label)}</h3><p>${spec.size_mm.join(' × ')} mm per model${spec.instances ? ` · ${spec.instances} end to end` : ''}</p></div>
        <span class="chip quiet">no result</span></div>`);
    setHTML('#obj-c-fit', '');
    return;
  }
  const b = r.box, cf = r.cloud_fit, im = r.image_fit;
  const cov = Object.entries(cf.coverage).map(([k, v]) => `<b>${esc(k)}</b><span class="${v < 0.5 ? 'alert' : ''}">${Math.round(v * 100)}% seen</span>`).join('');
  setHTML('#obj-c-box', `
    <div class="card-head"><div><h3><i style="background:#c9a15d"></i>${esc(r.label)}</h3><p>world frame · ${esc(r.dof)} · ${b.instances[0]}×${b.instances[1]} model instances</p></div>
      <span class="chip">${esc(r.method.split(' (')[0])}</span></div>
    <div class="rows">
      <b>Centre</b><span>${b.centre.map((v) => mmv(v)).join(', ')} mm</span>
      <b>Yaw</b><span>${fmt(b.pose.yaw_deg, 2)}°</span>
      <b>Size</b><span>${b.size_mm.map((v) => fmt(v, 1)).join(' × ')} mm</span>
      <b>Base z</b><span>${mmv(r.plate_z.used, 2)} mm (cloud ${mmv(r.plate_z.cloud, 2)})</span>
      <b>Fit</b><span>${esc(r.method)}</span>
      <b>Search</b><span>${fmt(r.duration_s, 1)} s · ${r.points} points</span>
    </div>`);
  setHTML('#obj-c-fit', `
    <div class="card-head"><div><h3>Precision</h3><p>cloud fit: bootstrap 1σ · side coverage</p></div>
      <span class="chip ${r.images_needed.length ? 'bad' : ''}">${r.images_needed.length ? 'cloud not enough' : 'cloud ok'}</span></div>
    <div class="rows">
      <b>1σ x · y · yaw</b><span>${fmt(cf.std_mm_mm_deg[0], 2)} · ${fmt(cf.std_mm_mm_deg[1], 2)} mm · ${fmt(cf.std_mm_mm_deg[2], 3)}°</span>
      ${cov}
      <b>Points</b><span>${cf.points.top} top · ${cf.points.plate} plate</span>
      <b>Coarse</b><span>score ${fmt(r.coarse.score, 2)}</span>
      ${cf.method === 'icp' ? `<b>ICP to CAD</b><span>${fmt(cf.residual_mm.median, 2)} mm median · ${Math.round(cf.inliers * 100)}% within 2 mm</span>
      <b>Weakest</b><span class="${cf.weakest.sigma > 1 ? 'alert' : ''}">${cf.weakest.direction_mm_mm_deg.map((v) => fmt(v, 2)).join(', ')} · σ ${fmt(cf.weakest.sigma, 2)}</span>` : ''}
      ${r.images_needed.map((x) => `<b>Needs images</b><span class="alert">${esc(x)}</span>`).join('')}
      ${im ? `<b>Image fit</b><span>${im.restarts_agree} restarts agree · ±${im.restart_spread_mm_mm_deg.slice(0, 2).map((v) => fmt(v, 2)).join(', ')} mm</span>` : ''}
      ${im ? Object.entries(im.check_cameras).map(([n, c]) => `<b>${esc(n)}</b><span class="${Math.max(...c.delta_mm_mm_deg.slice(0, 2).map(Math.abs)) > 2 ? 'alert' : ''}">check ${c.delta_mm_mm_deg.slice(0, 2).map((v) => fmt(v, 1)).join(', ')} mm</span>`).join('') : ''}
      ${(r.cameras || []).map((c) => `<b>${esc(c.camera)}</b><span>${c.used ? 'used' : 'not used'} · ${esc(c.why)}</span>`).join('')}
    </div>`);
}
$('#obj-sel').onchange = (e) => { objSel = e.target.value; objFocused = null; if (lastState) renderObjects(lastState.objects, lastState); };
$('#obj-locate').onclick = () => send({ type: 'locate_object', name: objSel, images: $('#obj-img').value });
$('#obj-focus').onclick = () => objView.focus();
toggle('#obj-cad', (on) => objView.showCad(on));
$('#obj-view').onclick = () => objView.resetView();
$('#obj-colour').onclick = (e) => {
  const b = e.target.closest('button');
  if (!b) return;
  document.querySelectorAll('#obj-colour button').forEach((x) => x.classList.toggle('on', x === b));
  objView.showCloud(b.dataset.c !== 'off');
  if (b.dataset.c !== 'off') objView.setColour(b.dataset.c);
};

// ── websocket ───────────────────────────────────────────────────────────────
let ws;
function send(m) { if (ws?.readyState === 1) ws.send(JSON.stringify(m)); }
function connect() {
  ws = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`);
  ws.binaryType = 'arraybuffer';
  ws.onopen = () => $('#conn').classList.add('hidden');
  ws.onclose = () => { $('#conn').classList.remove('hidden'); setTimeout(connect, 1000); };
  ws.onmessage = (ev) => {
    if (typeof ev.data === 'string') {
      const m = JSON.parse(ev.data);
      if (m.type === 'state') applyState(m);
      else if (m.type === 'saved') editor.onSaved(m);
      return;
    }
    const u = new Uint8Array(ev.data, 0, 2);
    if (u[0] === 1) showFrame(u[1], new Blob([new Uint8Array(ev.data, 2)], { type: 'image/jpeg' }));
    else if (u[0] === 2) applyCloud(ev.data);
  };
}
connect();

// ── experiments ─────────────────────────────────────────────────────────────
// agent runs under results/: a grid of runs; a run shows its camera videos, its tags and
// comments, its final output and, on request, the whole trace (text, tool calls and their results)
let expOpen = null;               // run shown, or null for the grid
let expWithTrace = false;         // open the run with its whole trace (#exp/<run>/trace)
let expPoll = null;
let expRun = null;                // detail of the open run
let expTag = null;                // grid shows only the runs with this tag
let expSrc = null;                // results folder shown (config `experiments`), null = the server's default
try { expSrc = localStorage.getItem('expSrc'); } catch { /* storage blocked */ }
// experiments API url for the folder shown
const expApi = (path = '') => `/api/experiments${path}${expSrc ? `?src=${encodeURIComponent(expSrc)}` : ''}`;
const expHash = (run, trace) => `#exp${expSrc ? `:${encodeURIComponent(expSrc)}` : ''}${run ? `/${encodeURIComponent(run)}${trace ? '/trace' : ''}` : ''}`;

const when = (iso) => {
  const d = iso ? new Date(/[zZ]|[+-]\d\d:\d\d$/.test(iso) ? iso : `${iso}Z`) : null;
  return d && !isNaN(d) ? d.toLocaleString([], { dateStyle: 'medium', timeStyle: 'short' }) : '—';
};
const mmss = (s) => `${String(Math.floor(s / 60)).padStart(2, '0')}:${String(Math.floor(s % 60)).padStart(2, '0')}`;

// small Markdown subset: headings, lists, code blocks, bold, italics, inline code, paragraphs
function md(text) {
  const inline = (t) => esc(t)
    .replace(/`([^`]+)`/g, '<code>$1</code>')
    .replace(/\*\*([^*]+)\*\*/g, '<b>$1</b>')
    .replace(/(^|[^*])\*([^*\s][^*]*)\*/g, '$1<i>$2</i>');
  const out = [];
  let list = null, para = [], code = null;
  const flush = () => {
    if (para.length) out.push(`<p>${para.map(inline).join('<br>')}</p>`);
    para = [];
    if (list) out.push(`</${list}>`);
    list = null;
  };
  for (const line of String(text ?? '').split('\n')) {
    if (code !== null) {
      if (/^\s*```/.test(line)) { out.push(`<pre>${esc(code.join('\n'))}</pre>`); code = null; } else code.push(line);
      continue;
    }
    if (/^\s*```/.test(line)) { flush(); code = []; continue; }
    const h = line.match(/^(#{1,4})\s+(.*)/);
    const li = line.match(/^\s*(?:([-*])|(\d+)\.)\s+(.*)/);
    if (h) { flush(); out.push(`<h4>${inline(h[2])}</h4>`); } else if (li) {
      const tag = li[1] ? 'ul' : 'ol';
      if (para.length || list !== tag) flush();
      if (!list) { out.push(`<${tag}>`); list = tag; }
      out.push(`<li>${inline(li[3])}</li>`);
    } else if (!line.trim()) flush();
    else if (list && /^\s+/.test(line)) out[out.length - 1] = out[out.length - 1].replace(/<\/li>$/, ` ${inline(line.trim())}</li>`);
    else { if (list) flush(); para.push(line); }
  }
  if (code !== null) out.push(`<pre>${esc(code.join('\n'))}</pre>`);
  flush();
  return out.join('');
}

async function getJSON(url, opts) {
  const r = await fetch(url, opts);
  if (!r.ok) throw new Error(`${r.status} ${r.statusText}`);
  return r.json();
}
const sendJSON = (method, url, body) => getJSON(url, {
  method, headers: { 'Content-Type': 'application/json' }, body: body && JSON.stringify(body) });

function expShow(run, withTrace = false) {
  expOpen = run;
  expWithTrace = withTrace;
  clearTimeout(expPoll);
  $('#exp-list').classList.toggle('hidden', !!run);
  $('#exp-run').classList.toggle('hidden', !run);
  history.replaceState(null, '', expHash(run));
  if (run) expLoadRun(run); else expLoadList();
}

async function expLoadList() {
  const grid = $('#exp-grid');
  try {
    let r0;
    try {
      r0 = await getJSON(expApi());
    } catch (e) {                 // a folder that is no longer configured: back to the default
      if (!expSrc) throw e;
      expSrc = null;
      r0 = await getJSON(expApi());
    }
    const { dir, runs: all, src, sources } = r0;
    if (expSrc !== src) {
      expSrc = src;
      try { localStorage.setItem('expSrc', src); } catch { /* storage blocked */ }
    }
    $('#exp-src').classList.toggle('hidden', sources.length < 2);
    $('#exp-src').innerHTML = sources.map((o) => `<option value="${esc(o.name)}" title="${esc(o.dir)}"${o.name === src ? ' selected' : ''}>${esc(o.name)}</option>`).join('');
    const count = {};
    for (const r of all) for (const t of r.tags) count[t] = (count[t] || 0) + 1;
    const tags = Object.keys(count).sort((a, b) => a.localeCompare(b));
    if (!count[expTag]) expTag = null;
    const runs = expTag ? all.filter((r) => r.tags.includes(expTag)) : all;
    $('#exp-dir').textContent = `${expTag ? `${runs.length} of ` : ''}${all.length} runs · ${dir}`;
    $('#exp-tag-list').innerHTML = tags.map((t) => `<option value="${esc(t)}">`).join('');
    $('#exp-filter').classList.toggle('hidden', !tags.length);
    $('#exp-filter').innerHTML = `<span class="lbl">Tags</span><button class="${expTag ? '' : 'on'}" data-tag="">All</button>`
      + tags.map((t) => `<button class="${t === expTag ? 'on' : ''}" data-tag="${esc(t)}">${esc(t)} · ${count[t]}</button>`).join('');
    grid.innerHTML = runs.length ? runs.map((r) => {
      const bad = r.error || (r.status && r.status !== 'success');
      const chip = r.recording ? '<span class="chip quiet">Recording</span>'
        : r.status ? `<span class="chip${bad ? ' bad' : ''}">${esc(r.status)}</span>` : '';
      const has = [r.videos.length ? `${r.videos.length} videos` : null, r.transcript ? 'transcript' : null]
        .filter(Boolean).join(' · ');
      const nums = [r.duration_s ? `${mmss(r.duration_s)} min` : null, r.turns ? `${r.turns} turns` : null,
        r.cost != null ? `$${fmt(r.cost)}` : null].filter(Boolean).join(' · ');
      const notes = r.comments ? `<span>${r.comments} comment${r.comments > 1 ? 's' : ''}</span>` : '';
      return `<button class="exp-card" data-run="${esc(r.run)}">
        <div class="card-head"><h3>${esc(r.name || r.run)}</h3>${chip}</div>
        <p class="exp-prompt">${esc(r.prompt || 'No transcript')}</p>
        ${r.tags.length ? `<div class="exp-tags">${r.tags.map((t) => `<span class="exp-tag">${esc(t)}</span>`).join('')}</div>` : ''}
        <div class="meta"><span><b>${esc(when(r.started_at))}</b></span><span>${esc(has)}</span>${nums ? `<span>${esc(nums)}</span>` : ''}${notes}</div>
      </button>`;
    }).join('') : `<p class="note">${expTag ? 'No runs with this tag.' : 'No runs with a transcript or videos yet.'}</p>`;
  } catch (e) {
    grid.innerHTML = `<p class="msg err">Could not list the experiments: ${esc(e.message)}</p>`;
  }
}

async function expLoadRun(run) {
  $('#exp-title').textContent = run;
  $('#exp-meta').textContent = '';
  $('#exp-videos').innerHTML = '';
  $('#exp-notes').innerHTML = '';
  $('#exp-text').innerHTML = '<p class="note">Loading…</p>';
  try {
    expRun = await getJSON(expApi(`/${encodeURIComponent(run)}`));
  } catch (e) {
    $('#exp-text').innerHTML = `<p class="msg err">${esc(e.message)}</p>`;
    return;
  }
  if (expOpen !== run) return;
  const r = expRun;
  $('#exp-title').textContent = r.name || run;
  $('#exp-meta').textContent = [when(r.started_at), r.model, r.status,
    r.duration_s ? `${mmss(r.duration_s)} min` : null, r.turns ? `${r.turns} turns` : null,
    r.cost != null ? `$${fmt(r.cost)}` : null].filter(Boolean).join(' · ');
  $('#exp-videos').classList.toggle('hidden', !r.videos.length);
  $('#exp-videos').innerHTML = r.videos.map((v) => {
    const cam = (r.recording_info?.cameras || []).find((c) => c.file === v);
    return `<div class="exp-video" data-file="${esc(v)}">
      <div class="exp-vbox"><div class="nosig">Checking video</div></div>
      <div class="cap">${esc(cam?.name || v)}<span>${esc(v)}</span></div>
    </div>`;
  }).join('');
  if (r.videos.length) expVideos(run);
  expNotes(r);
  if (!r.transcript) {
    $('#exp-text').innerHTML = '<p class="note">No transcript for this run.</p>';
    return;
  }
  $('#exp-text').innerHTML = `
    <div class="exp-block">
      <div class="exp-k">Prompt</div><pre class="exp-prompt-full">${esc(r.prompt)}</pre>
      ${r.error ? `<div class="exp-k err">Error</div><pre>${esc(r.error)}</pre>` : ''}
      <div class="exp-k">Final output</div>
      <div class="exp-md">${r.final_output ? md(r.final_output) : '<p class="note">No final output.</p>'}</div>
    </div>
    <div class="exp-trace-head">
      <button id="exp-trace-btn" class="primary">Show whole trace · ${r.steps} steps</button>
    </div>
    <div class="exp-trace" id="exp-trace"></div>`;
  $('#exp-trace-btn').onclick = () => expTrace(run);
  if (expWithTrace) expTrace(run);
}

// tags and comments of the open run; every change is saved on the server at once
function expNotes(n) {
  const box = $('#exp-notes');
  const draft = $('#exp-comment-in')?.value || '';
  box.innerHTML = `<div class="exp-block">
    <div class="exp-k">Tags</div>
    <div class="exp-tags">
      ${n.tags.map((t) => `<span class="exp-tag">${esc(t)}<button data-untag="${esc(t)}" title="Remove this tag">×</button></span>`).join('')}
      <input id="exp-tag-in" list="exp-tag-list" maxlength="40" placeholder="Add tag, Enter" autocomplete="off">
    </div>
    <div class="exp-k">Comments</div>
    ${n.comments.map((c) => `<div class="exp-comment">
      <div class="exp-comment-head"><span>${esc(when(c.at))}</span><button data-del="${esc(c.id)}">Delete</button></div>
      <div class="exp-md">${md(c.text)}</div>
    </div>`).join('') || '<p class="note">No comments yet.</p>'}
    <form class="chat-in" id="exp-comment-form">
      <textarea id="exp-comment-in" rows="2" placeholder="Add a comment (Markdown, Shift+Enter for a new line)"></textarea>
      <button class="primary">Comment</button>
    </form>
    <p class="msg err hidden" id="exp-notes-err"></p>
  </div>`;
  $('#exp-comment-in').value = draft;
}

async function expSaveNotes(method, path, body) {
  const run = expOpen;
  try {
    const n = await sendJSON(method, expApi(`/${encodeURIComponent(run)}/${path}`), body);
    if (expOpen !== run) return false;
    Object.assign(expRun, n);
    expNotes(expRun);
    return true;
  } catch (e) {
    const err = $('#exp-notes-err');
    if (err) { err.textContent = `Could not save: ${e.message}`; err.classList.remove('hidden'); }
    return false;
  }
}

function expAddTag(input) {
  const t = input.value.replace(/,/g, ' ').trim();
  if (!t) return;
  expSaveNotes('PUT', 'tags', { tags: [...expRun.tags, t] }).then((ok) => { if (ok) $('#exp-tag-in').focus(); });
}

// videos are converted to H.264 on the server the first time a run is opened
async function expVideos(run) {
  let st;
  try {
    st = await getJSON(expApi(`/${encodeURIComponent(run)}/videos`));
  } catch (e) {
    st = {};
  }
  if (expOpen !== run) return;
  let pending = false;
  for (const box of document.querySelectorAll('#exp-videos .exp-video')) {
    const s = st[box.dataset.file] || { state: 'error', error: 'unknown' };
    const vb = box.querySelector('.exp-vbox');
    if (s.state === 'ready') {
      if (!vb.querySelector('video')) {
        vb.innerHTML = `<video controls preload="metadata" muted playsinline
          src="${expApi(`/${encodeURIComponent(run)}/video/${encodeURIComponent(box.dataset.file)}`)}"></video>`;
        expSync();
      }
      continue;
    }
    pending = pending || s.state !== 'error';
    const label = {
      queued: 'Queued for conversion',
      converting: `Converting for the browser · ${Math.round((s.progress || 0) * 100)} %`,
      recording: 'Still recording',
      error: `Cannot convert: ${s.error || ''}`,
    }[s.state] || s.state;
    vb.innerHTML = `<div class="nosig${s.state === 'error' ? ' err' : ''}">${esc(label)}</div>`;
  }
  if (pending) expPoll = setTimeout(() => expVideos(run), 2000);
}

// the cameras were recorded together: play, pause and seek them together
function expSync() {
  const vids = [...document.querySelectorAll('#exp-videos video')];
  let busy = false;
  const mirror = (src, fn) => {
    if (busy) return;
    busy = true;
    for (const v of vids) if (v !== src) fn(v);
    setTimeout(() => { busy = false; }, 50);
  };
  for (const v of vids) {
    v.onplay = () => mirror(v, (o) => { o.currentTime = v.currentTime; o.play().catch(() => {}); });
    v.onpause = () => mirror(v, (o) => o.pause());
    v.onseeked = () => mirror(v, (o) => { if (Math.abs(o.currentTime - v.currentTime) > 0.3) o.currentTime = v.currentTime; });
    v.onratechange = () => mirror(v, (o) => { o.playbackRate = v.playbackRate; });
  }
}

function expSeek(at) {
  const t = Date.parse(at);
  for (const box of document.querySelectorAll('#exp-videos .exp-video')) {
    const v = box.querySelector('video');
    const cam = (expRun?.recording_info?.cameras || []).find((c) => c.file === box.dataset.file);
    const t0 = Date.parse(cam?.started_at || expRun?.recording_info?.started_at || '');
    if (v && !isNaN(t) && !isNaN(t0)) v.currentTime = Math.max(0, (t - t0) / 1000);
  }
  $('#exp-videos').scrollIntoView({ behavior: 'smooth', block: 'nearest' });
}

const MAIN_ARG = { Bash: 'command', Read: 'file_path', Write: 'file_path', Edit: 'file_path', Glob: 'pattern',
  Grep: 'pattern', WebFetch: 'url', WebSearch: 'query', Agent: 'description', Task: 'description', Skill: 'skill' };
const resultText = (r) => (typeof r === 'string' ? r : Array.isArray(r)
  ? r.map((b) => (b?.type === 'text' ? b.text : `[${b?.type || '?'}]`)).join('\n')
  : r == null ? '' : JSON.stringify(r, null, 2));

async function expTrace(run) {
  const box = $('#exp-trace');
  const btn = $('#exp-trace-btn');
  if (box.childElementCount) {         // second click hides it again
    box.innerHTML = '';
    btn.textContent = `Show whole trace · ${expRun.steps} steps`;
    history.replaceState(null, '', expHash(run));
    return;
  }
  btn.textContent = 'Loading trace…';
  let tr;
  try {
    tr = await getJSON(expApi(`/${encodeURIComponent(run)}/trace`));
  } catch (e) {
    btn.textContent = `Trace failed: ${e.message}`;
    return;
  }
  if (expOpen !== run) return;
  const t0 = Date.parse(tr.started_at || '');
  const hasVideo = !!document.querySelector('#exp-videos video, #exp-videos .exp-video');
  const stamp = (at) => {
    const t = Date.parse(at || '');
    const label = isNaN(t) || isNaN(t0) ? '--:--' : mmss(Math.max(0, (t - t0) / 1000));
    return hasVideo && at ? `<button class="exp-t" data-at="${esc(at)}" title="Show this moment in the videos">${label}</button>`
      : `<span class="exp-t">${label}</span>`;
  };
  box.innerHTML = tr.steps.map((s) => {
    if (s.kind === 'text') {
      return `<div class="exp-step">${stamp(s.at)}<div class="exp-md bot">${md(s.text)}</div></div>`;
    }
    if (s.kind === 'thinking') {
      return s.text?.trim() ? `<div class="exp-step">${stamp(s.at)}<pre class="exp-think">${esc(s.text.trim())}</pre></div>` : '';
    }
    if (s.kind !== 'tool_use') return '';
    const args = { ...(s.input || {}) };
    const title = args.description;
    delete args.description;
    const key = MAIN_ARG[s.name];
    let input = '';
    if (key && typeof args[key] === 'string') { input = args[key]; delete args[key]; }
    const rest = Object.entries(args).map(([k, v]) => `${k}: ${typeof v === 'string' ? v : JSON.stringify(v)}`);
    input = [input, ...rest].filter(Boolean).join('\n');
    const res = resultText(s.result).replace(/\s+$/, '') || '(no output)';
    const lines = res.split('\n').length;
    return `<div class="exp-step">${stamp(s.at)}<div class="exp-tool${s.is_error ? ' err' : ''}">
      <div class="exp-tool-head"><b>${esc(s.name)}</b>${title ? `<span>${esc(title)}</span>` : ''}</div>
      ${input ? `<pre class="exp-in">${esc(input)}</pre>` : ''}
      <details${lines <= 6 ? ' open' : ''}><summary>${s.is_error ? 'Error' : 'Result'} · ${lines} lines</summary>
        <pre class="exp-out">${esc(res)}</pre></details>
    </div></div>`;
  }).join('');
  btn.textContent = 'Hide whole trace';
  history.replaceState(null, '', expHash(run, true));
}

{
  $('#exp-grid').onclick = (e) => { const c = e.target.closest('.exp-card'); if (c) expShow(c.dataset.run); };
  $('#exp-filter').onclick = (e) => {
    const b = e.target.closest('button[data-tag]');
    if (b) { expTag = b.dataset.tag || null; expLoadList(); }
  };
  const notes = $('#exp-notes');
  notes.onclick = (e) => {
    const un = e.target.closest('button[data-untag]');
    if (un) expSaveNotes('PUT', 'tags', { tags: expRun.tags.filter((t) => t !== un.dataset.untag) });
    const del = e.target.closest('button[data-del]');
    if (del && confirm('Delete this comment?')) expSaveNotes('DELETE', `comments/${encodeURIComponent(del.dataset.del)}`);
  };
  notes.onkeydown = (e) => {
    if (e.target.id === 'exp-tag-in' && (e.key === 'Enter' || e.key === ',')) { e.preventDefault(); expAddTag(e.target); }
    if (e.target.id === 'exp-comment-in' && e.key === 'Enter' && !e.shiftKey) { e.preventDefault(); $('#exp-comment-form').requestSubmit(); }
  };
  notes.onsubmit = async (e) => {
    e.preventDefault();
    const input = $('#exp-comment-in');
    const text = input.value.trim();
    if (!text) return;
    input.value = '';
    if (!await expSaveNotes('POST', 'comments', { text })) $('#exp-comment-in').value = text;
  };
  $('#exp-back').onclick = () => expShow(null);
  $('#exp-refresh').onclick = () => expLoadList();
  $('#exp-src').onchange = (e) => {
    expSrc = e.target.value;
    try { localStorage.setItem('expSrc', expSrc); } catch { /* storage blocked */ }
    expTag = null;
    expShow(null);
  };
  $('#exp-text').onclick = (e) => { const b = e.target.closest('button.exp-t'); if (b) expSeek(b.dataset.at); };
}

// open the tab named in the address: #qa, #cams, #obj, #projects[:<id>], #circuit[:<design>], #exp[:<folder>][/<run>[/trace]]
{
  const h = decodeURIComponent(location.hash.slice(1));
  const m = h.match(/^exp(?::([^/]+))?(?:\/([^/]+)(\/trace)?)?$/);
  if (['environment', 'calib'].includes(h)) showTab('calib');
  else if (['qa', 'cams', 'obj'].includes(h)) showTab(h);
  else if (/^(projects|circuit)(:|$)/.test(h)) showTab('projects', h.split(':')[1]);     // Projects replaced Circuit Assembly
  else if (h === 'claude') showTab('circuit');
  else if (m) {
    if (m[1]) expSrc = m[1];
    expOpen = m[2] || null;
    expWithTrace = !!m[3];
    showTab('exp');
  } else showTab('projects');
}

