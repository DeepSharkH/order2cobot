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

const C = { bg: 0x111111, cream: 0xfef5da, brass: 0xc9a15d, grey: 0x8c8577 };
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

// CAD models of the targets (assets/models/cad/*.py) at T_base_obj from
// models/cad/poses.json (python -m cadguide register / anchor), semi-transparent so
// the camera clouds and the arm show through: misalignment is visible at a glance.
const cad = {};
const cadLoader = new GLTFLoader();
async function loadCadModel(name) {
  const g = placed(baseGroup);
  g.visible = false;
  const gltf = await cadLoader.loadAsync(`./models/cad/${name}.glb`);
  gltf.scene.traverse((o) => {
    if (!o.isMesh) return;
    o.material = o.material.clone();
    Object.assign(o.material, { transparent: true, opacity: 0.45, depthWrite: false });
  });
  g.add(gltf.scene);
  const l = label(name);
  g.add(l.obj);
  cad[name] = { g, l, key: '' };
}
Promise.all(['ls108g', 'breadboard'].map(loadCadModel)).catch((e) => console.error('CAD models', e));
let cadOn = true;
async function pollCadPoses() {
  try {
    const poses = await (await fetch('./models/cad/poses.json', { cache: 'no-store' })).json();
    for (const [name, m] of Object.entries(cad)) {
      const p = poses[name];
      m.g.visible = cadOn && !!p;
      if (!p) continue;
      const key = JSON.stringify(p);
      if (key === m.key) continue;
      m.key = key;
      // object x / y pitch scales (fitted from anchors) stretch the model about its origin
      m.g.matrix.copy(mat4(p.T)).multiply(new THREE.Matrix4().makeScale(p.scale[0], p.scale[1], 1));
      m.l.el.innerHTML = `${name} <small>${p.source}${p.anchors ? ` · ${p.anchors} anchor${p.anchors > 1 ? 's' : ''}` : ''}</small>`;
      m.l.obj.position.set(0, 0, name === 'ls108g' ? 0.04 : 0.015);
    }
  } catch { /* not registered yet */ }
}
pollCadPoses();
setInterval(pollCadPoses, 2000);

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
function setFrustum(f, K, size) {
  const key = JSON.stringify([K, size]);
  if (!K || !size || key === f.key) return;
  f.key = key;
  f.K = K;
  f.size = size;
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
function frustumImage(f, img) {
  if (f.canvas.width !== img.naturalWidth || f.canvas.height !== img.naturalHeight) {
    f.canvas.width = img.naturalWidth;
    f.canvas.height = img.naturalHeight;
    f.tex.dispose();          // new size: re-upload
  }
  f.canvas.getContext('2d').drawImage(img, 0, 0);
  f.tex.needsUpdate = true;
}
const fcam = frustum(C.grey, 'Fixed D435', 0.07);
const gcam = frustum(C.brass, 'Gripper cam', 0.04);
const cams = [fcam, gcam];

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
  f.l.obj.visible = on && f.g.visible;    // CSS2D labels ignore their parents' visibility
}
const OPT_TO_GL = new THREE.Matrix4().makeRotationX(Math.PI);   // optical (z fwd, y down) -> three (−z fwd, y up)
function setView(which) {
  const f = { fixed: fcam, gripper: gcam }[which] ?? null;
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
  document.querySelectorAll('#view-seg button').forEach((b) => b.classList.toggle('on', b.dataset.view === (which || 'free')));
}
function followCamera() {
  const f = view.cam;
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
document.querySelectorAll('#view-seg button').forEach((b) => {
  b.onclick = () => setView(b.dataset.view === 'free' ? null : b.dataset.view);
});
document.addEventListener('keydown', (e) => {
  if (['INPUT', 'SELECT'].includes(document.activeElement.tagName)) return;
  if (e.key === '1') setView('fixed');
  if (e.key === '2') setView('gripper');
  if (e.key === '0' || (e.key === 'Escape' && view.cam)) setView(null);
});

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
toggle('#btn-cloud', (on) => { clouds.forEach((c) => { c.visible = on; }); });
toggle('#btn-cad', (on) => { cadOn = on; for (const m of Object.values(cad)) m.key = ''; pollCadPoses(); });
toggle('#btn-ovl', (on) => send({ type: 'set_overlays', on }));

// ── camera feeds ────────────────────────────────────────────────────────────
const feeds = [];
function ensureFeeds(list) {
  list.forEach((c, i) => {
    if (feeds[i]) return;
    const el = document.createElement('div');
    el.className = 'feed';
    el.innerHTML = `<img alt=""><div class="nosig">No signal</div><div class="cap">${esc(c.name)}<span></span></div>`;
    $('#feeds').appendChild(el);
    feeds[i] = { el, img: el.querySelector('img'), nosig: el.querySelector('.nosig'), cap: el.querySelector('.cap span'), url: null };
  });
}
function showFrame(i, blob) {
  const f = feeds[i];
  if (!f) return;
  const url = URL.createObjectURL(blob);
  f.img.onload = () => {
    if (f.url) URL.revokeObjectURL(f.url);
    f.url = url;
    f.el.style.setProperty('--ar', `${f.img.naturalWidth} / ${f.img.naturalHeight}`);
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
    <div class="card" id="c-fcam">
      <div class="card-head"><div><h3>Fixed cam</h3><p>From the board</p></div><span class="chip quiet" id="fcam-chip"></span></div>
      <div class="actions"><button class="primary" id="b-locate">Locate from board</button></div>
      <div class="rows" id="fcam-rows"></div>
    </div>
    <div class="card" id="c-gcam">
      <div class="card-head"><div><h3>Gripper cam</h3><p>Base · FK · mount</p></div><span class="chip quiet" id="gcam-chip"></span></div>
      <div class="actions"><button id="b-mount" title="Mount = (base · FK)⁻¹ · one board view. Replaces the multi-view solve (dashboard/scene_solve.py); only for a moved camera">Mount from one view</button></div>
      <div class="rows" id="gcam-rows"></div>
    </div>
    <div class="card" id="c-base">
      <div class="card-head"><div><h3>Robot base</h3><p>Upright, under the board plane</p></div></div>
      <div class="actions"><button id="b-edit">Edit base</button></div>
      <div class="rows" id="base-rows"></div>
      <div class="msg" id="msg"></div>
    </div>`;
  $('#b-locate').onclick = () => send({ type: 'locate_fixed' });
  $('#b-mount').onclick = () => {
    if (confirm('Replace the gripper-cam mount with one from this single view? (the multi-view solve is better)')) send({ type: 'mount_from_view' });
  };
  $('#b-edit').onclick = () => (editor.active ? editor.close() : editor.open());
}
buildCards();

const mm = (v) => fmt(v * 1000, 0);
const boardRow = (c, b) => {
  const s = c.board;
  const txt = s.seen ? `seen · ${fmt(s.dist, 2)} m · ${fmt(s.rms, 2)} px`
    : s.partial ? `partial ${s.partial[0]}×${s.partial[1]} (need ${b.cols}×${b.rows})` : 'not seen';
  return `<b>Board</b><span class="${s.seen ? '' : 'alert'}">${txt}</span>`;
};
const planeRow = (p) => !p ? '<b>Depth on board</b><span>—</span>'
  : `<b>Depth on board</b><span class="${Math.abs(p.z_mm) > 3 || p.tilt_deg > 1 ? 'alert' : ''}">z ${fmt(p.z_mm, 1)} mm · tilt ${fmt(p.tilt_deg, 2)}° · ±${fmt(p.rms_mm, 1)} mm</span>`;

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

  const [fc, gc] = s.cams;
  const loc = fc.located;
  const lv = fc.live_vs_saved;
  $('#fcam-chip').textContent = loc ? loc.stamp.slice(11, 16) : 'not located';
  $('#fcam-rows').innerHTML = `
    <b>Camera</b><span class="${fc.ok ? '' : 'alert'}">${fc.ok ? `${fc.size?.[0]}×${fc.size?.[1]} · K ${esc(fc.intrinsics)}` : esc(fc.error || 'offline')}</span>
    ${boardRow(fc, s.board)}
    ${loc ? `<b>Located</b><span>${fmt(loc.rms_px)} px · ${loc.frames} frames</span>` : ''}
    ${fc.T_world_cam ? `<b>Position</b><span>${fc.T_world_cam.slice(0, 3).map((row) => mm(row[3])).join(', ')} mm</span>` : ''}
    ${lv ? `<b>Live vs saved</b><span class="${lv.dpos_mm > 5 ? 'alert' : ''}">${fmt(lv.dpos_mm, 1)} mm · ${fmt(lv.drot_deg, 2)}°</span>` : ''}
    ${planeRow(fc.plane)}`;
  $('#b-locate').disabled = !!s.busy || !fc.ok || !fc.board.seen;

  const ck = gc.check;
  $('#gcam-chip').textContent = ck ? `${fmt(ck.dpos_mm, 0)} mm off` : 'board not seen';
  $('#gcam-rows').innerHTML = `
    <b>Camera</b><span class="${gc.ok ? '' : 'alert'}">${gc.ok ? `${gc.size?.[0]}×${gc.size?.[1]} · K ${esc(gc.intrinsics)}` : esc(gc.error || 'offline')}</span>
    <b>Mount</b><span>${gc.mount.slice(3).map(mm).join(', ')} mm · ${gc.mount_set ? (gc.mount_set.views ? `${gc.mount_set.views} views` : 'one view') : 'config'}</span>
    ${boardRow(gc, s.board)}
    ${ck ? `<b>FK vs board view</b><span class="${ck.dpos_mm > 10 ? 'alert' : ''}">${fmt(ck.dpos_mm, 1)} mm · ${fmt(ck.drot_deg, 2)}°</span>
      <b>Base from view</b><span>${mm(ck.base.x)}, ${mm(ck.base.y)} mm · yaw ${fmt(ck.base.yaw_deg, 1)}°</span>
      <b>Its z · tilt</b><span class="${Math.abs(ck.base.z_mm) > 5 || ck.base.tilt_deg > 1 ? 'alert' : ''}">${fmt(ck.base.z_mm, 1)} mm · ${fmt(ck.base.tilt_deg, 2)}°</span>` : ''}
    ${planeRow(gc.plane)}`;

  $('#b-mount').disabled = !!s.busy || !gc.ok || !gc.board.seen || !r.connected;
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
  if (!editor.active) baseGroup.matrix.copy(mat4(s.T_world_base));
  if (s.robot.q) robot.setJoints(s.robot.q);
  if (s.robot.flange) flangeG.matrix.copy(mat4(s.robot.flange));

  s.cams.forEach((c, i) => {
    const f = cams[i];
    f.g.visible = !!c.T_world_cam;
    if (c.T_world_cam) f.g.matrix.copy(mat4(c.T_world_cam));
    showGlyph(f, view.cam !== f);
    setFrustum(f, c.K, c.size);
    const fd = feeds[i];
    if (fd) {
      fd.cap.textContent = c.ok ? `${c.size?.[0]}×${c.size?.[1]} · ${c.fps} fps` : (c.error || 'offline');
      if (!c.ok) fd.nosig.classList.remove('hidden');
    }
  });
  fcam.l.el.textContent = s.cams[0].located ? 'Fixed D435' : 'Fixed D435 (not located)';

  const st = [
    ['Robot', s.robot.connected ? 'ok' : 'bad'],
    ['Fixed cam', s.cams[0].ok ? 'ok' : 'bad'],
    ['Gripper cam', s.cams[1].ok ? 'ok' : 'bad'],
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
  cloud.geometry.setAttribute('position', new THREE.BufferAttribute(xyz.slice(), 3));
  cloud.geometry.setAttribute('color', new THREE.BufferAttribute(col, 3));
}

// ── base editor: x, y and yaw only (the base sits on the board plane) ───────
const gizmo = new TransformControls(camera, renderer.domElement);
gizmo.addEventListener('dragging-changed', (e) => { controls.enabled = !e.value; });
gizmo.setSize(0.8);
scene.add(gizmo.getHelper());

const editor = {
  active: false,
  open() {
    this.active = true;
    baseGroup.matrix.decompose(baseGroup.position, baseGroup.quaternion, baseGroup.scale);
    baseGroup.matrixAutoUpdate = true;
    this.z = baseGroup.position.z;          // the base height is fixed (calibrated), only x, y, yaw move
    gizmo.attach(baseGroup);
    this.mode('translate');
    $('#editor').classList.remove('hidden');
    $('#b-edit').classList.add('on');
    this.fields();
  },
  close() {
    this.active = false;
    gizmo.detach();
    baseGroup.matrixAutoUpdate = false;
    $('#editor').classList.add('hidden');
    $('#b-edit').classList.remove('on');
  },
  mode(m) {
    gizmo.setMode(m);
    gizmo.showX = gizmo.showY = m === 'translate';     // turns about z only
    gizmo.showZ = m === 'rotate';                      // stays on z = 0
    document.querySelectorAll('#editor [data-mode]').forEach((b) => b.classList.toggle('on', b.dataset.mode === m));
  },
  values() {
    const yaw = new THREE.Euler().setFromQuaternion(baseGroup.quaternion, 'ZYX').z;
    return { x: baseGroup.position.x, y: baseGroup.position.y, yaw: (yaw * 180) / Math.PI };
  },
  fields() {
    const v = this.values();
    const shown = { x: v.x * 1000, y: v.y * 1000, yaw: v.yaw };
    document.querySelectorAll('#editor input[data-k]').forEach((i) => {
      if (document.activeElement !== i) i.value = shown[i.dataset.k].toFixed(i.dataset.k === 'yaw' ? 1 : 0);
    });
  },
  fromFields() {
    const v = {};
    document.querySelectorAll('#editor input[data-k]').forEach((i) => { v[i.dataset.k] = parseFloat(i.value) || 0; });
    baseGroup.quaternion.setFromAxisAngle(new THREE.Vector3(0, 0, 1), (v.yaw * Math.PI) / 180);
    baseGroup.position.set(v.x / 1000, v.y / 1000, this.z);
  },
};
document.querySelectorAll('#editor [data-mode]').forEach((b) => { b.onclick = () => editor.mode(b.dataset.mode); });
document.querySelectorAll('#editor input[data-k]').forEach((i) => { i.oninput = () => editor.fromFields(); });
gizmo.addEventListener('objectChange', () => { baseGroup.position.z = editor.z; editor.fields(); });
$('#ed-cancel').onclick = () => editor.close();
$('#ed-save').onclick = () => { send({ type: 'set_base', ...editor.values() }); editor.close(); };
document.addEventListener('keydown', (e) => {
  if (!editor.active || document.activeElement.tagName === 'INPUT') return;
  if (e.key === 'm' || e.key === 'M') editor.mode('translate');
  if (e.key === 'r' || e.key === 'R') editor.mode('rotate');
  if (e.key === 'Escape') editor.close();
});


// ── tabs ────────────────────────────────────────────────────────────────────
// one set of camera feeds, moved into whichever tab is showing
function showTab(name) {
  for (const t of ['calib', 'qa', 'cams', 'obj', 'claude', 'exp']) $(`#tab-${t}`).classList.toggle('hidden', t !== name);
  document.querySelectorAll('#tabs button').forEach((b) => b.classList.toggle('on', b.dataset.tab === name));
  const home = { calib: '#calib-cams', qa: '#qa-cams', cams: '#cams-only', obj: '#obj-cams' }[name];
  if (home) $(home).appendChild($('#feeds'));
  if (name === 'exp') expShow(expOpen, expWithTrace);
  else history.replaceState(null, '', name === 'calib' ? location.pathname : `#${name}`);
}
$('#tabs').onclick = (e) => { const b = e.target.closest('button'); if (b) showTab(b.dataset.tab); };

// ── port QA ─────────────────────────────────────────────────────────────────
let qaSel = 1;
let qaLast = null;
let qaVlmT = null;                // follow the VLM's port each time it reports a new one
const VLM_FRESH_S = 5;
const sw3d = new SwitchView($('#qa-3d'), { onPick: (p) => { qaSel = p; if (qaLast) renderQA(qaLast); } });
$('#qa-view').onclick = () => sw3d.resetView();
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
    <div><div class="k">${esc(q.iface)}</div><div class="v ${L.up ? '' : 'alert'}">${L.error ? 'No interface' : L.up ? 'Link up' : 'No link'}</div></div>
    <div class="m">${L.up ? `${L.speed ?? '?'} Mb/s · ${esc(L.duplex)} duplex` : 'cable out or port dead'}</div>
    <div class="m">${q.busy ? `checking ${esc(q.busy)}…` : esc(q.message)}</div>`);
  setHTML('#qa-ports', q.ports.map((p) => `
    <button class="port ${p.status === 'untested' ? '' : p.status} ${p.port === qaSel ? 'sel' : ''} ${p.port === busyPort ? 'testing' : ''}" data-port="${p.port}">
      <span class="num">${p.port}</span><span class="st2">${p.port === busyPort ? 'testing' : esc(p.status)}</span>
      <span class="det">in ${qaDet(p.insert)}<br>out ${qaDet(p.extract)}</span>
    </button>`).join(''));

  const ev = (q.events || []).slice().reverse().map((e) =>
    `<div class="e ${e.up ? '' : 'down'}"><span>${esc(e.t)}</span><span>${e.up ? 'link up' : 'link down'}</span><span>${e.up && e.speed ? `${e.speed} Mb/s` : ''}</span></div>`).join('');
  setHTML('#qa-c-link', `
    <div class="card-head"><div><h3>Link</h3><p>${esc(q.iface)} · live from /sys/class/net</p></div>
      <span class="chip ${L.up ? '' : 'bad'}">${L.up ? `${L.speed ?? '?'} Mb/s` : 'down'}</span></div>
    <div class="rows">
      <b>Carrier</b><span>${L.up ? 'up' : 'down'}</span>
      <b>Duplex</b><span>${esc(L.duplex ?? '—')}</span>
      <b>Carrier changes</b><span>${L.carrier_changes ?? '—'}</span>
      <b>Pass rule</b><span>≥ ${q.cfg.min_speed} Mb/s · holds ${fmt(q.cfg.stable_s, 1)} s · ${fmt(q.cfg.timeout_s, 0)} s timeout</span>
    </div>
    <div class="events">${ev || '<div class="e down"><span>—</span><span>no link changes yet</span><span></span></div>'}</div>
    ${L.error ? `<p class="note" style="color:#d2483c">${esc(L.error)}</p>` : ''}`);

  const p = q.ports[qaSel - 1];
  const resRows = (name, r) => !r ? `<b>${name}</b><span>not checked</span>`
    : `<b>${name}</b><span class="${r.ok ? '' : 'alert'}">${r.ok ? 'pass' : 'fail'} · ${esc(r.reason)}</span>`;
  setHTML('#qa-c-port', `
    <div class="card-head"><div><h3>Port ${qaSel}</h3><p>${p.stamp ? esc(p.stamp.replace('T', ' ')) : 'not tested yet'} · ${p.attempts} insert${p.attempts === 1 ? '' : 's'}</p></div>
      <span class="chip ${p.status === 'fail' ? 'bad' : p.status === 'untested' ? 'quiet' : ''}">${esc(p.status)}</span></div>
    <div class="rows">
      <b>Gripper cam VLM</b><span class="${vlm && !vlm.fresh ? 'alert' : ''}">${vlm ? `port ${vlm.port}${vlm.confidence != null ? ` · ${Math.round(vlm.confidence * 100)}%` : ''} · ${fmt(vlm.age, 1)} s ago` : 'no report yet'}</span>
      ${resRows('Insert', p.insert)}${resRows('Extract', p.extract)}
      ${p.insert?.flaps ? `<b>Flaps on insert</b><span class="alert">${p.insert.flaps}</span>` : ''}</div>`);

  const n = (st) => q.ports.filter((x) => x.status === st).length;
  const host = location.host || 'localhost:8090';
  setHTML('#qa-c-run', `
    <div class="card-head"><div><h3>Run</h3><p>${q.ports.length} ports · results in data/qa/ports.json</p></div>
      <span class="chip ${n('fail') ? 'bad' : ''}">${n('pass')}/${q.ports.length} pass</span></div>
    <div class="state">
      <div><div class="k">Pass</div><div class="v">${n('pass')}</div></div>
      <div><div class="k">Fail</div><div class="v ${n('fail') ? 'alert' : 'muted'}">${n('fail')}</div></div>
      <div><div class="k">Left</div><div class="v ${n('untested') ? '' : 'muted'}">${n('untested')}</div></div>
    </div>
    <b style="font-size:11px;letter-spacing:.18em;text-transform:uppercase">Robot API</b>
    <pre class="api">http://${esc(host)}
POST /api/qa/vlm {"port": ${qaSel}, "confidence": 0.9}
POST /api/qa/current/insert
POST /api/qa/current/extract
→ 200 pass · 409 fail · JSON result</pre>`);

  const busy = !!q.busy;
  for (const id of ['#qa-insert', '#qa-extract', '#qa-reset-port', '#qa-reset-all']) $(id).disabled = busy;
}
$('#qa-ports').onclick = (e) => {
  const b = e.target.closest('.port');
  if (!b) return;
  qaSel = +b.dataset.port;
  if (qaLast) renderQA(qaLast);
};
$('#qa-insert').onclick = () => send({ type: 'qa_test', port: qaSel, phase: 'insert' });
$('#qa-extract').onclick = () => send({ type: 'qa_test', port: qaSel, phase: 'extract' });
$('#qa-reset-port').onclick = () => { if (confirm(`Clear the results of port ${qaSel}?`)) send({ type: 'qa_reset', port: qaSel }); };
$('#qa-reset-all').onclick = () => { if (confirm('Clear the results of all ports?')) send({ type: 'qa_reset' }); };

// ── objects ─────────────────────────────────────────────────────────────────
// bounding box of a configured object (configs/objects.yaml) in the live clouds;
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
      return;
    }
    const u = new Uint8Array(ev.data, 0, 2);
    if (u[0] === 1) showFrame(u[1], new Blob([new Uint8Array(ev.data, 2)], { type: 'image/jpeg' }));
    else if (u[0] === 2) applyCloud(ev.data);
  };
}
connect();
setInterval(() => { $('#clock').textContent = new Date().toLocaleTimeString('en-GB'); }, 1000);

// ── claude ──────────────────────────────────────────────────────────────────
// frontend to the local Claude Code CLI (server runs `claude -p` on the subscription)
let ccSession = null;
let ccAbort = null;
const ccLog = $('#cc-log');

function ccAdd(cls, text) {
  const d = document.createElement('div');
  d.className = `msg ${cls}`;
  d.textContent = text;
  ccLog.appendChild(d);
  ccLog.scrollTop = ccLog.scrollHeight;
  return d;
}

function ccEvent(ev) {
  if (ev.type === 'system' && ev.subtype === 'init') ccSession = ev.session_id;
  else if (ev.type === 'assistant') {
    for (const c of ev.message.content || []) {
      if (c.type === 'text' && c.text.trim()) ccAdd('bot', c.text);
      else if (c.type === 'tool_use') ccAdd('tool', `▸ ${c.name} ${JSON.stringify(c.input).slice(0, 400)}`);
    }
  } else if (ev.type === 'user') {
    for (const c of ev.message.content || []) {
      if (c.type !== 'tool_result') continue;
      const t = Array.isArray(c.content) ? c.content.map((x) => x.text || '').join('') : String(c.content ?? '');
      ccAdd(c.is_error ? 'tool err' : 'tool', t.slice(0, 1200));
    }
  } else if (ev.type === 'result') {
    if (ev.session_id) ccSession = ev.session_id;
    const denied = (ev.permission_denials || []).map((p) => p.tool_name).join(', ');
    ccAdd(ev.is_error ? 'meta err' : 'meta',
      `${ev.subtype} · ${fmt((ev.duration_ms || 0) / 1000, 1)} s · ${ev.num_turns} turns${denied ? ` · denied: ${denied}` : ''}`);
  }
}

async function ccSend(prompt) {
  ccAdd('me', prompt);
  ccAbort = new AbortController();
  $('#cc-send').textContent = 'Stop';
  try {
    const r = await fetch('/api/claude', {
      method: 'POST', headers: { 'Content-Type': 'application/json' }, signal: ccAbort.signal,
      body: JSON.stringify({ prompt, session: ccSession, mode: $('#cc-mode').value }),
    });
    const rd = r.body.getReader(), dec = new TextDecoder();
    let buf = '';
    for (;;) {
      const { value, done } = await rd.read();
      if (done) break;
      buf += dec.decode(value, { stream: true });
      let i;
      while ((i = buf.indexOf('\n')) >= 0) {
        const line = buf.slice(0, i).trim();
        buf = buf.slice(i + 1);
        if (!line) continue;
        try { ccEvent(JSON.parse(line)); } catch { ccAdd('meta err', line); }
      }
    }
    if (buf.trim() || !r.ok) ccAdd('meta err', `HTTP ${r.status} ${buf.trim()}`);
  } catch (e) {
    ccAdd('meta err', e.name === 'AbortError' ? 'stopped' : String(e));
  }
  ccAbort = null;
  $('#cc-send').textContent = 'Send';
}

$('#cc-form').onsubmit = (e) => {
  e.preventDefault();
  if (ccAbort) { ccAbort.abort(); return; }
  const p = $('#cc-prompt').value.trim();
  if (!p) return;
  $('#cc-prompt').value = '';
  ccSend(p);
};
$('#cc-prompt').onkeydown = (e) => {
  if (e.key === 'Enter' && !e.shiftKey) { e.preventDefault(); $('#cc-form').requestSubmit(); }
};
$('#cc-new').onclick = () => { ccSession = null; ccLog.textContent = ''; };

// ── experiments ─────────────────────────────────────────────────────────────
// agent runs under results/: a grid of runs; a run shows its camera videos, its tags and
// comments, its final output and, on request, the whole trace (text, tool calls and their results)
let expOpen = null;               // run shown, or null for the grid
let expWithTrace = false;         // open the run with its whole trace (#exp/<run>/trace)
let expPoll = null;
let expRun = null;                // detail of the open run
let expTag = null;                // grid shows only the runs with this tag

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
  history.replaceState(null, '', run ? `#exp/${encodeURIComponent(run)}` : '#exp');
  if (run) expLoadRun(run); else expLoadList();
}

async function expLoadList() {
  const grid = $('#exp-grid');
  try {
    const { dir, runs: all } = await getJSON('/api/experiments');
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
    expRun = await getJSON(`/api/experiments/${encodeURIComponent(run)}`);
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
    const n = await sendJSON(method, `/api/experiments/${encodeURIComponent(run)}/${path}`, body);
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
    st = await getJSON(`/api/experiments/${encodeURIComponent(run)}/videos`);
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
          src="/api/experiments/${encodeURIComponent(run)}/video/${encodeURIComponent(box.dataset.file)}"></video>`;
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
    history.replaceState(null, '', `#exp/${encodeURIComponent(run)}`);
    return;
  }
  btn.textContent = 'Loading trace…';
  let tr;
  try {
    tr = await getJSON(`/api/experiments/${encodeURIComponent(run)}/trace`);
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
  history.replaceState(null, '', `#exp/${encodeURIComponent(run)}/trace`);
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
  $('#exp-text').onclick = (e) => { const b = e.target.closest('button.exp-t'); if (b) expSeek(b.dataset.at); };
}

// open the tab named in the address: #qa, #cams, #obj, #claude, #exp, #exp/<run> or #exp/<run>/trace
{
  const h = decodeURIComponent(location.hash.slice(1));
  const m = h.match(/^exp(?:\/([^/]+)(\/trace)?)?$/);
  if (['qa', 'cams', 'obj', 'claude'].includes(h)) showTab(h);
  else if (m) { expOpen = m[1] || null; expWithTrace = !!m[2]; showTab('exp'); }
}
