// Calibration dashboard frontend: one websocket, a three.js scene in the world
// frame (checkerboard centre, z up), camera feeds and calibration cards. See
// dashboard/server.py for the frames and the wire format.

import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';
import { CSS2DRenderer, CSS2DObject } from './vendor/CSS2DRenderer.js';
import { TransformControls } from './vendor/TransformControls.js';
import { UR3eModel } from './ur3e_model.js';
import { HandEModel } from './hande_model.js';
import { SwitchView } from './switch3d.js';

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

// checkerboard, centred on the world origin in the z = 0 plane
const boardG = new THREE.Group();
scene.add(boardG);
let boardKey = '';
function buildBoard(b) {
  const key = `${b.cols}x${b.rows}x${b.square}`;
  if (key === boardKey) return;
  boardKey = key;
  boardG.clear();
  const s = b.square;
  const geom = new THREE.PlaneGeometry(s, s);
  const white = new THREE.MeshBasicMaterial({ color: C.cream, side: THREE.DoubleSide });
  const black = new THREE.MeshBasicMaterial({ color: 0x111111, side: THREE.DoubleSide });
  // inner corners cols x rows -> squares (cols+1) x (rows+1); world y = -board y
  const cx = ((b.cols - 1) * s) / 2, cy = ((b.rows - 1) * s) / 2;
  for (let j = -1; j < b.rows; j++) {
    for (let i = -1; i < b.cols; i++) {
      const m = new THREE.Mesh(geom, (i + j) % 2 === 0 ? black : white);
      m.position.set((i + 0.5) * s - cx, -((j + 0.5) * s - cy), 0.0005);
      boardG.add(m);
    }
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
  ensureFeeds(s.cams);
  if (!editor.active) baseGroup.matrix.copy(mat4(s.T_world_base));
  if (s.robot.q) robot.setJoints(s.robot.q);
  if (s.robot.flange) flangeG.matrix.copy(mat4(s.robot.flange));
  buildBoard(s.board);

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
  renderCards(s);
  if (s.qa) renderQA(s.qa);
}

function applyCloud(buf) {
  const cloud = clouds[new Uint8Array(buf, 1, 1)[0]];
  if (!cloud) return;
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
  for (const t of ['calib', 'qa', 'claude']) $(`#tab-${t}`).classList.toggle('hidden', t !== name);
  document.querySelectorAll('#tabs button').forEach((b) => b.classList.toggle('on', b.dataset.tab === name));
  if (name !== 'claude') $(name === 'qa' ? '#qa-cams' : '#calib-cams').appendChild($('#feeds'));
  history.replaceState(null, '', name === 'calib' ? location.pathname : `#${name}`);
}
$('#tabs').onclick = (e) => { const b = e.target.closest('button'); if (b) showTab(b.dataset.tab); };
if (location.hash === '#qa' || location.hash === '#claude') showTab(location.hash.slice(1));

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
