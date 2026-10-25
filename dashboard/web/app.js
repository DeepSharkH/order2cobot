// Calibration dashboard frontend: one websocket, a three.js scene (world =
// checkerboard centre, z up, once calibrated; the robot base until then),
// camera feeds and calibration cards. See dashboard/server.py for the wire format.

import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';
import { CSS2DRenderer, CSS2DObject } from './vendor/CSS2DRenderer.js';
import { TransformControls } from './vendor/TransformControls.js';
import { UR3eModel } from './ur3e_model.js';
import { HandEModel } from './hande_model.js';
import { SwitchView } from './switch3d.js';

const C = { bg: 0x111111, cream: 0xfef5da, brass: 0xc9a15d, grey: 0x8c8577, blue: 0x5b8fd6, red: 0xd2483c };
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
const HOME = { pos: new THREE.Vector3(0.9, -1.0, 0.8), target: new THREE.Vector3(0, 0, 0.15) };
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
scene.add(new THREE.AxesHelper(0.1));

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

// everything the robot knows lives in its base frame; baseGroup = T_world_base
const baseGroup = placed(scene);
const robot = new UR3eModel();
baseGroup.add(robot.root);
robot.load().catch((e) => console.error('UR3e meshes failed to load', e));
const armLabel = label('UR3e');
armLabel.obj.position.set(0, 0, 0.62);
baseGroup.add(armLabel.obj);
const baseAxes = new THREE.AxesHelper(0.12);
baseGroup.add(baseAxes);
const flangeG = placed(baseGroup);
flangeG.add(new THREE.AxesHelper(0.06));

// Robotiq Hand-E on the flange (coupler + body along flange z; fingers along x)
const hande = new HandEModel();
flangeG.add(hande.root);
hande.load().catch((e) => console.error('Hand-E meshes failed to load', e));

// checkerboard (board frame: OpenCV, z into the table)
const boardG = placed(baseGroup);
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
  // inner corners cols x rows -> squares (cols+1) x (rows+1), first square at (-s, -s)
  for (let j = -1; j < b.rows; j++) {
    for (let i = -1; i < b.cols; i++) {
      const m = new THREE.Mesh(geom, (i + j) % 2 === 0 ? black : white);
      m.position.set((i + 0.5) * s, (j + 0.5) * s, 0.0005);
      boardG.add(m);
    }
  }
  boardG.add(new THREE.AxesHelper(s * 3));
  const l = label('Board');
  l.obj.position.set((b.cols * s) / 2, -2 * s, 0);
  boardG.add(l.obj);
}

// cameras, drawn like the Hestia dashboard: a short pyramid from the
// intrinsics (optical frame: x right, y down, z forward), an "up" triangle
// above the image's top edge, a faint face, axes and a label. With "Frustum
// images" on, the face shows the live feed.
const GLYPH_DEPTH = { fixed: 0.14, gripper: 0.08 };
function frustum(parent, color, name, depth) {
  const g = placed(parent);
  const lines = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color }));
  const face = new THREE.Mesh(new THREE.BufferGeometry(), new THREE.MeshBasicMaterial({
    color, transparent: true, opacity: 0.08, side: THREE.DoubleSide, depthWrite: false }));
  const canvas = document.createElement('canvas');
  const tex = new THREE.CanvasTexture(canvas);
  tex.colorSpace = THREE.SRGBColorSpace;
  const image = new THREE.Mesh(new THREE.BufferGeometry(), new THREE.MeshBasicMaterial({
    map: tex, side: THREE.DoubleSide, transparent: true, opacity: 0.9, depthWrite: false }));
  image.visible = false;
  const glyph = new THREE.Group();      // hidden while looking through this camera (the cloud stays)
  glyph.add(lines, face, image, new THREE.AxesHelper(0.06));
  g.add(glyph);
  const l = label(name, 'label3d cam');
  glyph.add(l.obj);
  return { g, glyph, lines, face, image, canvas, tex, l, depth, K: null, size: null, key: '' };
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
  // image top-left pixel -> tl (y down)
  f.image.geometry.setFromPoints([tl, tr, br, bl]);
  f.image.geometry.setAttribute('uv', new THREE.Float32BufferAttribute([0, 1, 1, 1, 1, 0, 0, 0], 2));
  f.image.geometry.setIndex([0, 1, 2, 0, 2, 3]);
  f.l.obj.position.copy(up).add(new THREE.Vector3(0, -0.03, 0));
}
function frustumImage(f, img) {
  if (f.canvas.width !== img.naturalWidth || f.canvas.height !== img.naturalHeight) {
    f.canvas.width = img.naturalWidth;
    f.canvas.height = img.naturalHeight;
    f.tex.dispose();          // new size: re-upload
  }
  f.canvas.getContext('2d').drawImage(img, 0, 0);
  f.tex.needsUpdate = true;
  f.image.visible = showFrustumImages && view.cam !== f;
}
let showFrustumImages = false;
// the fixed cam hangs in the world (it may be placed before the base is known);
// the gripper cam rides on the robot
const fcam = frustum(scene, C.grey, 'Fixed D435', GLYPH_DEPTH.fixed);
const gcam = frustum(baseGroup, C.brass, 'Gripper cam', GLYPH_DEPTH.gripper);
const FCAM_PLACEHOLDER = new THREE.Matrix4().makeRotationX(Math.PI).setPosition(0, -0.35, 0.9);

// point clouds, each in its camera's optical frame (the gripper's rides on the arm)
const clouds = [fcam, gcam].map((f, i) => {
  const c = new THREE.Points(new THREE.BufferGeometry(),
    new THREE.PointsMaterial({ size: i === 0 ? 0.004 : 0.002, vertexColors: true, sizeAttenuation: true }));
  c.frustumCulled = false;
  f.g.add(c);
  return c;
});

// sample flange poses
const samplesG = new THREE.Group();
baseGroup.add(samplesG);
let samplesKey = '';
function drawSamples(samples) {
  const key = samples.map((s) => `${s.id}:${s.rms}`).join(',');
  if (key === samplesKey) return;
  samplesKey = key;
  samplesG.clear();
  for (const s of samples) {
    const a = new THREE.AxesHelper(0.025);
    a.matrixAutoUpdate = false;
    a.matrix.copy(mat4(s.flange));
    samplesG.add(a);
    const dot = new THREE.Mesh(new THREE.SphereGeometry(0.006, 10, 8),
      new THREE.MeshBasicMaterial({ color: s.rms != null && s.rms > 2 ? C.red : C.blue }));
    dot.position.setFromMatrixPosition(a.matrix);
    samplesG.add(dot);
  }
}

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
  f.l.obj.visible = on;                   // CSS2D labels ignore their parents' visibility
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
    camera.setViewOffset && camera.clearViewOffset();
    scene.background = null;
    resize();
  }
  [fcam, gcam].forEach((g) => { g.image.visible = showFrustumImages && g !== f && g.canvas.width > 0; });
  document.querySelectorAll('#view-seg button').forEach((b) => b.classList.toggle('on', b.dataset.view === (which || 'free')));
}
function followCamera() {
  const f = view.cam;
  if (!f || !f.K) return;
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
toggle('#btn-samples', (on) => { samplesG.visible = on; });
toggle('#btn-fimg', (on) => {
  showFrustumImages = on;
  [fcam, gcam].forEach((f) => { f.image.visible = on && f !== view.cam && f.canvas.width > 0; });
});
toggle('#btn-ovl', (on) => send({ type: 'set_overlays', on }));

// ── camera feeds ────────────────────────────────────────────────────────────
const feeds = [];
function ensureFeeds(cams) {
  cams.forEach((c, i) => {
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
    frustumImage(i === 0 ? fcam : gcam, f.img);
  };
  f.img.src = url;
  f.nosig.classList.add('hidden');
}

// ── cards ───────────────────────────────────────────────────────────────────
let last = null;
let staticBuilt = false;
function buildCards() {
  $('#cards').innerHTML = `
    <div class="card dark" id="c-robot"></div>
    <div class="card" id="c-cap">
      <div class="card-head"><div><h3>Capture</h3><p>Freedrive the arm so the gripper cam sees the whole board, then capture</p></div><span class="chip" id="cap-chip"></span></div>
      <div class="actions">
        <button class="primary" id="b-cap">Capture  <kbd>Space</kbd></button>
        <button id="b-new" title="Start an empty sample set">New session</button>
      </div>
      <div class="rows" id="cap-rows"></div>
      <div class="samples" id="samples"></div>
      <p class="note" style="margin-top:6px">Aim for 15–30 samples: tilt ±30°, rotate the wrist, vary distance. Click a sample to drop it.</p>
    </div>
    <div class="card" id="c-solve">
      <div class="card-head"><div><h3>Solve</h3><p>Hand-eye · board · fixed cam · base · DH</p></div><span class="chip quiet" id="res-chip"></span></div>
      <div class="opts">
        <label>Gripper intrinsics <select id="o-intr"><option value="1">estimate</option><option value="0">keep</option></select></label>
        <label>FK / DH <select id="o-dh"><option value="full">refine all</option><option value="offsets">joint offsets</option><option value="none">controller as is</option></select></label>
      </div>
      <div class="actions"><button class="primary" id="b-solve">Solve + save</button></div>
      <div class="rows" id="res-rows"></div>
      <div class="msg" id="msg"></div>
      <pre class="log" id="log"></pre>
    </div>`;
  $('#b-cap').onclick = () => send({ type: 'capture' });
  $('#b-new').onclick = () => { if (confirm('Start a new, empty sample session? (the current one stays on disk)')) send({ type: 'new_session' }); };
  $('#b-solve').onclick = () => send({ type: 'solve', opts: { intrinsics: $('#o-intr').value === '1', dh: $('#o-dh').value } });
  $('#samples').onclick = (e) => {
    const s = e.target.closest('.s');
    if (s && confirm(`Remove sample ${s.dataset.id}?`)) send({ type: 'delete_sample', id: +s.dataset.id });
  };
  document.addEventListener('keydown', (e) => {
    if (e.code === 'Space' && !['INPUT', 'SELECT', 'BUTTON'].includes(document.activeElement.tagName)) {
      e.preventDefault();
      send({ type: 'capture' });
    }
  });
  staticBuilt = true;
}

function errRow(name, e) {
  if (!e) return `<b>${name}</b><span>—</span>`;
  const mm = e[0] * 1000;
  return `<b>${name}</b><span class="${mm > 1 ? 'alert' : ''}">${mm.toFixed(2)} mm · ${(e[1] * 1000).toFixed(2)} mrad</span>`;
}

function renderCards(s) {
  if (!staticBuilt) buildCards();
  const r = s.robot;
  const q = r.q || [];
  $('#c-robot').innerHTML = `
    <div class="card-head"><div><h3>UR3e</h3><p>${esc(r.mode ?? '—')} · ${esc(r.control ?? '—')} · tier ${esc(r.tier ?? '—')}</p></div>
      <span class="chip ${r.connected ? '' : 'bad'}">${r.connected ? `${fmt(r.rate, 0)} Hz` : 'offline'}</span></div>
    <div class="joints">${JN.map((n, i) => {
      const v = q[i];
      const pct = v == null ? 50 : ((v + 2 * Math.PI) / (4 * Math.PI)) * 100;
      return `<div class="joint"><span class="n">${n}</span><span class="track"><i style="left:${pct}%"></i></span><span class="val">${v == null ? '—' : deg(v)}</span></div>`;
    }).join('')}</div>
    <div class="rows">
      <b>FK check vs controller</b><span>${esc(r.tcp ?? '')}</span>
      ${errRow('Nominal DH (internet)', r.fk_err_nominal)}
      ${errRow('Controller DH', r.fk_err_ctrl)}
      ${r.fk_err_refined ? errRow('Refined DH', r.fk_err_refined) : ''}
    </div>
    ${r.error ? `<p class="note" style="color:#d2483c">${esc(r.error)}</p>` : ''}`;

  const [fc, gc] = s.cams;
  const camRow = (c) => {
    const b = c.board;
    const seen = b.seen ? `board ✓ ${fmt(b.dist, 2)} m · ${fmt(b.rms, 2)} px`
      : b.partial ? `partial ${b.partial[0]}×${b.partial[1]} (need ${s.board.cols}×${s.board.rows})` : 'no board';
    return `<b>${esc(c.name)}</b><span class="${b.seen ? '' : 'alert'}">${seen}</span>`;
  };
  $('#cap-rows').innerHTML = `${camRow(gc)}${camRow(fc)}
    <b>Board</b><span>${s.board.cols}×${s.board.rows} inner · ${(s.board.square * 1000).toFixed(1)} mm</span>
    <b>Session</b><span>${esc(s.session)}</span>`;
  $('#cap-chip').textContent = `${s.samples.length} samples`;
  $('#b-cap').disabled = !!s.busy || !r.connected;
  $('#b-solve').disabled = !!s.busy;
  $('#samples').innerHTML = s.samples.map((x) =>
    `<span class="s ${x.rms != null && x.rms > 2 ? 'bad' : ''}" data-id="${x.id}" title="gripper ${x.gripper ? '✓' : '✗'} · fixed ${x.fixed ? '✓' : '✗'}${x.rms != null ? ` · ${x.rms.toFixed(2)} px` : ''}">
      ${x.id}<i class="${x.gripper ? 'on' : ''}"></i><i class="${x.fixed ? 'on' : ''}"></i></span>`).join('');

  const res = s.result;
  $('#res-chip').textContent = res ? res.stamp.replace('T', ' ') : 'not calibrated';
  if (res) {
    const fit = (k) => res.fits[k] ? `${fmt(res.fits[k].gripper_rms_px)} px${res.fits[k].fixed_rms_px != null ? ` · fixed ${fmt(res.fits[k].fixed_rms_px)} px` : ''}` : '—';
    const K = res.gripper_intrinsics?.K;
    const sh = res.fk_refined_vs_controller;
    $('#res-rows').innerHTML = `
      <b>Samples</b><span>${res.n_samples} gripper · ${res.n_fixed} fixed</span>
      <b>Reproj · nominal DH</b><span>${fit('nominal')}</span>
      <b>Reproj · controller DH</b><span>${fit('controller')}</span>
      ${res.fits.refined ? `<b>Reproj · refined DH</b><span>${fit('refined')}</span>` : ''}
      ${sh ? `<b>Refined FK shift</b><span>${fmt(sh.mean_mm)} mm mean · ${fmt(sh.max_mm)} max</span>` : ''}
      <b>Nominal vs controller FK</b><span>${fmt(res.fk_nominal_vs_controller_mm?.mean)} mm mean</span>
      ${K ? `<b>Gripper fx · fy</b><span>${fmt(K[0][0], 1)} · ${fmt(K[1][1], 1)}${res.gripper_intrinsics.rms_px ? ` (${fmt(res.gripper_intrinsics.rms_px)} px)` : ''}</span>` : ''}
      ${s.calib.T_world_base ? `<b>Base in world</b><span>${s.calib.T_world_base.slice(0, 3).map((row) => (row[3] * 1000).toFixed(1)).join(', ')} mm</span>` : ''}`;
    $('#log').textContent = (res.log || []).join('\n');
  }
  $('#msg').textContent = s.busy ? `${s.busy}…` : s.message;
}

// ── state → scene ───────────────────────────────────────────────────────────
function applyState(s) {
  last = s;
  ensureFeeds(s.cams);
  const calibrated = !!s.calib.T_world_base;
  if (!editor.active) baseGroup.matrix.copy(calibrated ? mat4(s.calib.T_world_base) : new THREE.Matrix4());
  armLabel.el.innerHTML = { calibrated: 'UR3e', manual: 'UR3e <small>manual base</small>' }[s.base_source]
    ?? 'UR3e <small>base uncalibrated</small>';
  if (s.robot.q) robot.setJoints(s.robot.q);
  if (s.robot.flange) flangeG.matrix.copy(mat4(s.robot.flange));

  buildBoard(s.board);
  boardG.visible = !!s.calib.T_base_board;
  if (s.calib.T_base_board) boardG.matrix.copy(mat4(s.calib.T_base_board));

  // fixed cam: calibrated pose, else posed so the depth-fitted table plane is
  // z = 0, else a placeholder looking down
  const fc = s.cams[0];
  // saved base, not the editor's: the camera stays put while the base is dragged
  const Twb = calibrated ? mat4(s.calib.T_world_base) : new THREE.Matrix4();
  fcam.g.matrix.copy(fc.T_base_cam ? Twb.multiply(mat4(fc.T_base_cam))
    : fc.T_world_cam_table ? mat4(fc.T_world_cam_table) : FCAM_PLACEHOLDER);
  fcam.l.el.textContent = fc.T_base_cam ? 'Fixed D435'
    : fc.T_world_cam_table ? 'Fixed D435 (table fit, not calibrated)' : 'Fixed D435 (not calibrated)';
  setFrustum(fcam, fc.K, fc.size);
  // gripper cam: hand-eye result, else the configured mount guess
  const gc = s.cams[1];
  gcam.g.visible = !!gc.T_base_cam;
  gcam.l.obj.visible = gcam.g.visible && view.cam !== gcam;
  if (gc.T_base_cam) gcam.g.matrix.copy(mat4(gc.T_base_cam));
  gcam.l.el.textContent = gc.pose_calibrated ? 'Gripper cam' : 'Gripper cam (mount guess)';
  setFrustum(gcam, gc.K, gc.size);
  drawSamples(s.samples);

  s.cams.forEach((c, i) => {
    const f = feeds[i];
    if (!f) return;
    f.cap.textContent = c.ok ? `${c.size?.[0]}×${c.size?.[1]} · ${c.fps} fps · K ${c.intrinsics}` : (c.error || 'offline');
    if (!c.ok) f.nosig.classList.remove('hidden');
  });

  const st = [
    ['Robot', s.robot.connected ? 'ok' : 'bad'],
    ['Fixed cam', s.cams[0].ok ? 'ok' : 'bad'],
    ['Gripper cam', s.cams[1].ok ? 'ok' : 'bad'],
    [s.base_source === 'manual' ? 'Manual base' : 'Calibrated', calibrated ? 'ok' : 'warn'],
    ['Switch link', s.qa?.live?.up ? 'ok' : 'bad'],
  ];
  $('#status').innerHTML = st.map(([n, c]) => `<span class="st ${c}"><i></i>${n}</span>`).join('');
  const b = $('#banner');
  b.classList.toggle('hidden', calibrated || editor.active);
  b.textContent = 'Not calibrated — scene shown in the robot base frame';
  editor.sync(s);
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

// ── base editor ─────────────────────────────────────────────────────────────
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
    gizmo.attach(baseGroup);
    this.mode('translate');
    $('#editor').classList.remove('hidden');
    $('#btn-edit').classList.add('on');
    $('#ed-msg').textContent = '';
    this.fields();
  },
  close() {
    this.active = false;
    gizmo.detach();
    baseGroup.matrixAutoUpdate = false;
    $('#editor').classList.add('hidden');
    $('#btn-edit').classList.remove('on');
  },
  mode(m) {
    gizmo.setMode(m);
    gizmo.showX = gizmo.showY = m === 'translate';     // the base turns about z only
    document.querySelectorAll('#editor [data-mode]').forEach((b) => b.classList.toggle('on', b.dataset.mode === m));
  },
  fields() {
    const e = new THREE.Euler().setFromQuaternion(baseGroup.quaternion, 'ZYX');
    const v = { x: baseGroup.position.x * 1000, y: baseGroup.position.y * 1000, z: baseGroup.position.z * 1000,
      yaw: (e.z * 180) / Math.PI };
    document.querySelectorAll('#editor input[data-k]').forEach((i) => {
      if (document.activeElement !== i) i.value = v[i.dataset.k].toFixed(i.dataset.k === 'yaw' ? 1 : 0);
    });
  },
  fromFields() {
    const v = {};
    document.querySelectorAll('#editor input[data-k]').forEach((i) => { v[i.dataset.k] = parseFloat(i.value) || 0; });
    const e = new THREE.Euler().setFromQuaternion(baseGroup.quaternion, 'ZYX');
    e.z = (v.yaw * Math.PI) / 180;
    baseGroup.quaternion.setFromEuler(e);
    baseGroup.position.set(v.x / 1000, v.y / 1000, v.z / 1000);
  },
  rows() {
    baseGroup.updateMatrix();
    const m = baseGroup.matrix.elements;          // column-major
    return [0, 1, 2, 3].map((r) => [0, 1, 2, 3].map((c) => m[c * 4 + r]));
  },
  sync(s) {
    $('#ed-active').textContent = { manual: 'manual', calibrated: 'calibrated', none: 'not set' }[s.base_source];
    $('#ed-calib').disabled = s.base_source !== 'manual' || !s.result;
  },
};
$('#btn-edit').onclick = () => (editor.active ? editor.close() : editor.open());
document.querySelectorAll('#editor [data-mode]').forEach((b) => { b.onclick = () => editor.mode(b.dataset.mode); });
document.querySelectorAll('#editor input[data-k]').forEach((i) => { i.oninput = () => editor.fromFields(); });
gizmo.addEventListener('objectChange', () => editor.fields());
$('#ed-cancel').onclick = () => editor.close();
$('#ed-save').onclick = () => { send({ type: 'set_manual_base', T_world_base: editor.rows(), active: true }); editor.close(); };
$('#ed-calib').onclick = () => { send({ type: 'set_manual_base', T_world_base: null, active: false }); editor.close(); };
document.addEventListener('keydown', (e) => {
  if (!editor.active || document.activeElement.tagName === 'INPUT') return;
  if (e.key === 'm' || e.key === 'M') editor.mode('translate');
  if (e.key === 'r' || e.key === 'R') editor.mode('rotate');
  if (e.key === 'Escape') editor.close();
});

// ── tabs ────────────────────────────────────────────────────────────────────
// one set of camera feeds, moved into whichever tab is showing
function showTab(name) {
  for (const t of ['calib', 'qa']) $(`#tab-${t}`).classList.toggle('hidden', t !== name);
  document.querySelectorAll('#tabs button').forEach((b) => b.classList.toggle('on', b.dataset.tab === name));
  $(name === 'qa' ? '#qa-cams' : '#calib-cams').appendChild($('#feeds'));
  history.replaceState(null, '', name === 'qa' ? '#qa' : location.pathname);
}
$('#tabs').onclick = (e) => { const b = e.target.closest('button'); if (b) showTab(b.dataset.tab); };
if (location.hash === '#qa') showTab('qa');

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
