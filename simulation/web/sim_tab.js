// Simulation tab: the simulated UR3e behind the fake AI Agent Bridge (python -m simulation).
// Drawn in the robot base frame like the 3D Visualization tab draws the arm: UR3e + Hand-E
// from the joints, the simulated objects at their true poses (CAD meshes + the contact boxes
// the simulator uses), the TCP path and the contact force. Below: the URScript that runs
// (current line highlighted) and everything the skills send to the arm (bridge calls, socket
// commands). Data: /api/sim/* (proxied to the simulator by the dashboard server).

import * as THREE from 'three';
import { OrbitControls } from '../vendor/OrbitControls.js';
import { CSS2DRenderer, CSS2DObject } from '../vendor/CSS2DRenderer.js';
import { GLTFLoader } from '../vendor/GLTFLoader.js';
import { UR3eModel } from '../ur3e_model.js';
import { HandEModel } from '../hande_model.js';

const C = { bg: 0x111111, cream: 0xfef5da, brass: 0xc9a15d, grey: 0x8c8577, red: 0xd2483c };
const JN = ['Base', 'Shoulder', 'Elbow', 'Wrist 1', 'Wrist 2', 'Wrist 3'];
const $ = (sel, root = document) => root.querySelector(sel);
const esc = (s) => String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
const fmt = (v, d = 2) => (v == null ? '—' : Number(v).toFixed(d));
const mm = (v) => fmt(v * 1000, 1);
const deg = (r) => `${((r * 180) / Math.PI).toFixed(1)}°`;
const mat4 = (m) => new THREE.Matrix4().set(...m[0], ...m[1], ...m[2], ...m[3]);
const clock = (t) => new Date(t * 1000).toTimeString().slice(0, 8) + `.${String(Math.floor((t % 1) * 1000)).padStart(3, '0')}`;
const api = async (path, opts) => {
  const r = await fetch(`/api/sim/${path}`, { cache: 'no-store', ...opts });
  const j = await r.json();
  if (!r.ok) throw Object.assign(new Error(j.error || r.status), j);
  return j;
};

let root = null;
let visible = false;
let timers = [];
const S = { state: null, seq: 0, trailN: 0, hash: null, worldKey: null, reads: false, follow: true, line: 0, lines: [] };

const HTML = `
  <section class="section scene">
    <div class="section-head">
      <h2>Simulation</h2>
      <div class="legend">
        <span><i class="sw sw-arm"></i>UR3e (sim)</span>
        <span><i class="sw sw-board"></i>Objects (sim truth)</span>
        <span><i class="sw sw-mix"></i>TCP path</span>
        <span><i class="sw sw-fail"></i>Contact force</span>
      </div>
      <div class="tools">
        <button id="sim-clear" title="Clear the TCP path">Clear path</button>
        <button id="sim-view" title="Reset the view">Reset</button>
      </div>
    </div>
    <div id="sim-3d" class="viewport">
      <div class="sim-offline hidden" id="sim-offline"></div>
    </div>
  </section>
  <section class="section cams sim-bottom">
    <div class="sim-panels">
      <div class="sim-panel">
        <div class="sim-panel-head"><h3>URScript</h3><span id="sim-prog"></span>
          <button id="sim-follow" class="toggle on" title="Keep the executing line in view">Follow</button></div>
        <div class="sim-code" id="sim-code"><div class="sim-empty">No program has run yet.</div></div>
      </div>
      <div class="sim-panel">
        <div class="sim-panel-head"><h3>Sent to the arm</h3><span>bridge calls · socket commands</span>
          <button id="sim-reads" class="toggle" title="Also list read-only calls (state polls, logs, IK)">Reads</button></div>
        <div class="sim-list" id="sim-traffic"></div>
      </div>
    </div>
  </section>
  <section class="section side">
    <div class="section-head"><h2>Simulator</h2></div>
    <div class="cards">
      <div class="card dark" id="sim-c-ctrl"></div>
      <div class="card" id="sim-c-tool"></div>
      <div class="card sim-logcard" id="sim-c-log">
        <div class="card-head"><div><h3>Controller log</h3><p>psx_logs · textmsg results</p></div></div>
        <div class="sim-log" id="sim-log"></div>
      </div>
    </div>
  </section>`;

// ── 3D ──────────────────────────────────────────────────────────────
let renderer, labels, scene, camera, controls, robot, flangeG, hande, tcpG, arrow, trail, trailPts, tcpLabel, objG;
// from the front-left of the workcell: the switch's port face and the breadboard, the arm at the right
const HOME = { pos: new THREE.Vector3(-0.02, -0.95, 0.62), target: new THREE.Vector3(-0.22, 0.04, 0.08) };

function label(text, cls = 'label3d') {
  const el = document.createElement('div');
  el.className = cls;
  el.innerHTML = text;
  return { el, obj: new CSS2DObject(el) };
}

function build3d() {
  const vp = $('#sim-3d', root);
  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.setClearColor(C.bg);
  vp.appendChild(renderer.domElement);
  labels = new CSS2DRenderer();
  Object.assign(labels.domElement.style, { position: 'absolute', top: '0', pointerEvents: 'none' });
  vp.appendChild(labels.domElement);

  scene = new THREE.Scene();
  camera = new THREE.PerspectiveCamera(45, 1, 0.005, 50);
  camera.up.set(0, 0, 1);
  camera.position.copy(HOME.pos);
  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.target.copy(HOME.target);
  scene.add(new THREE.HemisphereLight(0xffffff, 0x2a2622, 1.8));
  const sun = new THREE.DirectionalLight(0xffffff, 1.4);
  sun.position.set(1, -1.5, 3);
  scene.add(sun);
  const grid = new THREE.GridHelper(2, 20, 0x2f2f2f, 0x1d1d1d);
  grid.rotation.x = Math.PI / 2;
  scene.add(grid);
  scene.add(new THREE.AxesHelper(0.1));        // the robot base: the sim's frame

  robot = new UR3eModel();
  scene.add(robot.root);
  robot.load().catch((e) => console.error('UR3e meshes failed to load', e));
  const armLabel = label('UR3e <small>sim</small>');
  armLabel.obj.position.set(0, 0, 0.62);
  scene.add(armLabel.obj);
  flangeG = new THREE.Group();
  flangeG.matrixAutoUpdate = false;
  scene.add(flangeG);
  hande = new HandEModel();
  flangeG.add(hande.root);
  hande.load().catch((e) => console.error('Hand-E meshes failed to load', e));

  tcpG = new THREE.Group();
  tcpG.matrixAutoUpdate = false;
  tcpG.add(new THREE.Mesh(new THREE.SphereGeometry(0.003, 16, 12), new THREE.MeshBasicMaterial({ color: C.cream })));
  tcpG.add(new THREE.AxesHelper(0.02));
  scene.add(tcpG);
  tcpLabel = label('', 'label3d dark sim-tcp-label');
  tcpLabel.obj.position.set(0, 0, 0.012);
  tcpG.add(tcpLabel.obj);

  arrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(), 0.05, C.red, 0.012, 0.007);
  arrow.visible = false;
  scene.add(arrow);

  trailPts = new Float32Array(3 * 6000);
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(trailPts, 3));
  g.setDrawRange(0, 0);
  trail = new THREE.Line(g, new THREE.LineBasicMaterial({ color: C.brass }));
  trail.frustumCulled = false;
  scene.add(trail);
  trail.n = 0;

  objG = new THREE.Group();
  scene.add(objG);
  new ResizeObserver(resize).observe(vp);
  $('#sim-view', root).onclick = () => { camera.position.copy(HOME.pos); controls.target.copy(HOME.target); };
  (function loop() {
    requestAnimationFrame(loop);
    if (!visible) return;
    controls.update();
    renderer.render(scene, camera);
    labels.render(scene, camera);
  })();
}

function resize() {
  const vp = $('#sim-3d', root);
  const w = vp.clientWidth;
  const h = vp.clientHeight;
  if (!w || !h) return;
  renderer.setSize(w, h);
  labels.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}

const gltf = new GLTFLoader();
async function buildWorld(w) {
  objG.clear();
  for (const o of w.objects) {
    const g = new THREE.Group();
    g.matrixAutoUpdate = false;
    g.matrix.copy(mat4(o.T));
    objG.add(g);
    // the box the simulator collides with
    const size = o.hi.map((h, i) => h - o.lo[i]);
    const box = new THREE.BoxGeometry(...size);
    box.translate(...o.hi.map((h, i) => (h + o.lo[i]) / 2));
    g.add(new THREE.LineSegments(new THREE.EdgesGeometry(box), new THREE.LineBasicMaterial({ color: C.cream, transparent: true, opacity: 0.5 })));
    if (o.kind === 'switch') {
      for (const [i, p] of o.pockets.entries()) {             // port mouths
        const m = new THREE.Mesh(new THREE.PlaneGeometry(0.0118, 0.0066), new THREE.MeshBasicMaterial({ color: 0x000000, side: THREE.DoubleSide }));
        m.rotation.x = Math.PI / 2;
        m.position.set(p[0], p[1] - 0.0002, p[2]);
        m.name = o.labels[i];
        g.add(m);
      }
    }
    const src = w.sources[o.name] || {};
    const l = label(`${o.name} <small>${esc(src.source || '')}${src.truth_offset_mm?.some((v) => v) ? ` · offset ${src.truth_offset_mm.join('/')} mm` : ''}</small>`);
    l.obj.position.set(0, 0, o.hi[2] + 0.02);
    g.add(l.obj);
    try {                                                       // the CAD mesh when the dashboard has it
      const m = await gltf.loadAsync(`./models/cad/${o.name}.glb`);
      m.scene.traverse((x) => {
        if (!x.isMesh) return;
        x.material = x.material.clone();
        Object.assign(x.material, { transparent: true, opacity: 0.45, depthWrite: false });
      });
      const sc = src.scale || [1, 1];
      m.scene.scale.set(sc[0], sc[1], 1);
      g.add(m.scene);
    } catch { /* boxes only */ }
  }
}

// ── state → page ────────────────────────────────────────────────────
function applyState(s) {
  S.state = s;
  const r = s.robot;
  robot.setJoints(r.q);
  flangeG.matrix.copy(mat4(r.flange));
  flangeG.matrixWorldNeedsUpdate = true;
  hande.setOpening(r.gripper.width / 2);
  const tp = r.tcp_pose;
  const rv = new THREE.Vector3(tp[3], tp[4], tp[5]);
  const ang = rv.length();
  const Rm = new THREE.Matrix4().makeRotationAxis(ang > 1e-9 ? rv.clone().normalize() : new THREE.Vector3(0, 0, 1), ang);
  tcpG.matrix.copy(Rm.setPosition(tp[0], tp[1], tp[2]));
  tcpG.matrixWorldNeedsUpdate = true;
  const F = new THREE.Vector3(...r.force.slice(0, 3));
  const fn = F.length();
  arrow.visible = fn > 0.3;
  if (arrow.visible) {
    arrow.position.set(tp[0], tp[1], tp[2]);
    arrow.setDirection(F.clone().normalize());
    arrow.setLength(Math.min(0.15, 0.015 + fn * 0.003), 0.012, 0.007);
  }
  const c = r.contact;
  tcpLabel.el.innerHTML = `${fn > 0.3 ? `${fmt(fn, 1)} N` : 'TCP'}${c?.pocket ? ` · ${esc(c.pocket)} ${c.depth > 0 ? `${fmt(c.depth, 1)} mm in` : ''}` : ''}`;
  objG.traverse((m) => {
    if (m.isMesh && m.name?.startsWith('port:')) m.material.color.setHex(c?.pocket === m.name && c.inside ? C.brass : 0x000000);
  });
  renderCards(s);
  highlight(s.program.line);
}

// the control card is built once (mount); only its text parts change, so clicks and the
// open rate menu survive the 10 Hz updates
const CTRL = `
  <div class="card-head"><div><h3>UR3e · simulated</h3><p id="sim-ctrl-sub"></p></div><span id="sim-ctrl-chip"></span></div>
  <div class="rows sim-rows" id="sim-ctrl-rows"></div>
  <div class="joints" id="sim-ctrl-joints"></div>
  <div class="actions sim-actions">
    <button data-a="stop">Stop</button>
    <button data-a="unlock">Unlock</button>
    <button data-a="reset" title="Back to the home joints, clear the path">Reset</button>
    <button data-a="mode" title="Pendant control mode: play is refused in Local"></button>
    <select data-a="rate" title="Simulated seconds per wall second">${[0.25, 0.5, 1, 2, 5, 0].map((v) => `<option value="${v}">${v ? `${v}×` : 'max'}</option>`).join('')}</select>
  </div>`;
const shown = new Map();
function setHTML(id, html) {
  if (shown.get(id) === html) return;
  shown.set(id, html);
  $(id, root).innerHTML = html;
}

function renderCards(s) {
  const r = s.robot;
  const p = s.program;
  const stop = r.safety !== 'NORMAL';
  setHTML('#sim-ctrl-chip', stop ? '<span class="chip bad">Protective stop</span>' : `<span class="chip">${esc(p.state)}</span>`);
  setHTML('#sim-ctrl-sub', `${esc(p.robotmode)} · ${esc(p.controlmode)} · tier ${esc(p.tier)} · ${fmt(r.t, 1)} s simulated`);
  setHTML('#sim-ctrl-rows', `
      <b>Program</b><span>${esc(p.loaded ?? '—')}${p.run_name ? ` <small>(${esc(p.run_name)})</small>` : ''}</span>
      <b>Line</b><span>${p.line || '—'}</span>
      ${p.error ? `<b>Error</b><span class="alert">${esc(p.error)}</span>` : ''}
      ${stop ? `<b>Stop</b><span class="alert">${esc(r.stop_reason)}</span>` : ''}`);
  setHTML('#sim-ctrl-joints', JN.map((n, i) => {
    const v = r.q[i];
    const pct = ((v + 2 * Math.PI) / (4 * Math.PI)) * 100;
    return `<div class="joint"><span class="n">${n}</span><span class="track${Math.abs(v) > 6 ? ' near' : ''}"><i style="left:${pct.toFixed(2)}%"></i></span><span class="val">${deg(v)}</span></div>`;
  }).join(''));
  const card = $('#sim-c-ctrl', root);
  card.querySelector('[data-a="stop"]').disabled = p.state === 'STOPPED';
  card.querySelector('[data-a="unlock"]').disabled = !stop;
  card.querySelector('[data-a="mode"]').textContent = p.controlmode === 'REMOTE' ? 'Remote' : 'Local';
  const sel = card.querySelector('[data-a="rate"]');
  if (document.activeElement !== sel) sel.value = String(s.rate);
  const tp = r.tcp_pose;
  const fm = r.force_mode;
  const c = r.contact;
  const fr = r.force_reading;
  setHTML('#sim-c-tool', `
    <div class="card-head"><div><h3>Tool</h3><p>TCP p[${r.tcp.slice(0, 3).map(mm).join(', ')}] mm from the flange</p></div>
      <span class="chip quiet">${r.speed > 1e-4 ? `${fmt(r.speed * 1000, 1)} mm/s` : 'still'}</span></div>
    <div class="rows">
      <b>TCP</b><span>${tp.slice(0, 3).map(mm).join(', ')} mm</span>
      <b>Rotation</b><span>${tp.slice(3).map((v) => fmt(v, 3)).join(', ')}</span>
      <b>Force</b><span>${fr.slice(0, 3).map((v) => fmt(v, 1)).join(', ')} N <small>(sensor, zeroed)</small></span>
      <b>Force mode</b><span>${fm ? `[${fm.sel.slice(0, 3).join(' ')}] · ${fm.wrench.slice(0, 3).map((v) => fmt(v, 1)).join(', ')} N` : 'off'}</span>
      <b>Contact</b><span class="${c?.pen > 1 ? 'alert' : ''}">${c ? `${esc(c.object)}${c.pocket ? ` · ${esc(c.pocket)}` : ''}${c.pen ? ` · ${fmt(c.pen, 2)} mm` : ''}` : 'free'}</span>
      ${c?.pocket ? `<b>In ${esc(c.pocket)}</b><span>${fmt(c.depth, 2)} mm deep · off axis ${fmt(c.u, 2)} / ${fmt(c.v, 2)} mm</span>` : ''}
      <b>Hand-E</b><span>${r.gripper.active ? `${mm(r.gripper.width)} mm open` : 'not activated'}</span>
    </div>`);
}

function renderEvents(evs) {
  const box = $('#sim-traffic', root);
  const log = $('#sim-log', root);
  for (const e of evs) {
    if (e.kind === 'log') {
      const text = e.text.startsWith('text_msg ') ? JSON.parse(e.text.slice(9)).text : e.text;
      const row = document.createElement('div');
      row.className = `sim-lrow ${e.level === 'INFO' ? '' : 'warn'} ${e.text.startsWith('text_msg') ? 'msg' : ''}`;
      row.innerHTML = `<span class="t">${clock(e.t)}</span><span>${esc(text)}</span>`;
      log.prepend(row);
      continue;
    }
    const row = document.createElement('div');
    const read = e.kind === 'call' && e.category === 'read';
    row.className = `sim-row ${e.kind} ${read ? 'read' : ''} ${e.category || ''} ${e.ok === false ? 'err' : ''}`;
    if (read && !S.reads) row.classList.add('hidden');
    const what = e.kind === 'call' ? `<b>${esc(e.text)}</b> <span class="args">${esc(e.args)}</span>${e.error ? `<div class="e">${esc(e.error)}</div>` : ''}` : `<b>${esc(e.text)}</b>`;
    row.innerHTML = `<span class="t">${clock(e.t)}</span><span class="k">${e.kind === 'call' ? esc(e.category) : esc(e.kind)}</span>
      <span class="w">${what}</span><span class="c">${esc(e.client || '')}${e.ms != null ? ` · ${e.ms} ms` : ''}</span>`;
    box.prepend(row);
  }
  for (const el of [box, log]) while (el.children.length > 400) el.lastChild.remove();
}

function renderCode(text) {
  S.lines = text.split('\n');
  const box = $('#sim-code', root);
  box.innerHTML = S.lines.map((l, i) => `<div class="ln" data-n="${i + 1}"><span>${i + 1}</span><code>${esc(l) || ' '}</code></div>`).join('');
  S.line = 0;
}

function highlight(n) {
  if (!n || n === S.line) return;
  const box = $('#sim-code', root);
  box.querySelector('.ln.on')?.classList.remove('on');
  const el = box.querySelector(`.ln[data-n="${n}"]`);
  S.line = n;
  if (!el) return;
  el.classList.add('on');
  if (S.follow) {
    const top = el.offsetTop - box.clientHeight / 2;
    if (Math.abs(box.scrollTop - top) > box.clientHeight / 3) box.scrollTop = top;
  }
}

function offline(msg) {
  const el = $('#sim-offline', root);
  el.classList.toggle('hidden', !msg);
  if (msg) el.innerHTML = `<b>Simulator offline</b><span>${esc(msg)}</span><code>python -m simulation</code>`;
}

// ── polling ─────────────────────────────────────────────────────────
async function pollState() {
  try {
    const s = await api('state');
    offline(null);
    if (s.world !== S.worldKey) { S.worldKey = s.world; buildWorld(await api('world')); }
    if (s.program.hash && s.program.hash !== S.hash) {
      S.hash = s.program.hash;
      const p = await api('program');
      renderCode(p.text);
      $('#sim-prog', root).textContent = p.name || '';
    }
    if (s.trail_n < S.trailN) clearTrail();
    applyState(s);
  } catch (e) {
    offline(e.offline ? e.error : String(e.message || e));
  }
}

async function pollEvents() {
  try {
    const r = await api(`events?since=${S.seq}`);
    if (r.seq < S.seq) S.seq = 0;                // simulator restarted
    else { S.seq = r.seq; renderEvents(r.events); }
  } catch { /* shown by pollState */ }
}

function clearTrail() {
  trail.n = 0;
  S.trailN = 0;
  trail.geometry.setDrawRange(0, 0);
}

async function pollTrail() {
  try {
    const r = await api(`trail?since=${S.trailN}`);
    S.trailN = r.n;
    for (const p of r.points) {
      if (trail.n >= 6000) {                      // keep the newest half
        trailPts.copyWithin(0, 3 * 3000);
        trail.n = 3000;
      }
      trailPts.set(p, 3 * trail.n);
      trail.n += 1;
    }
    trail.geometry.attributes.position.needsUpdate = true;
    trail.geometry.setDrawRange(0, trail.n);
  } catch { /* offline */ }
}

async function control(action, value) {
  await api('control', { method: 'POST', body: JSON.stringify({ action, value }), headers: { 'Content-Type': 'application/json' } });
  if (action === 'reset') clearTrail();
  pollState();
}

// ── tab API (dashboard/web/app.js) ──────────────────────────────────
export function mount(el) {
  root = el;
  if (!document.querySelector('link[data-sim]')) {
    const l = document.createElement('link');
    Object.assign(l, { rel: 'stylesheet', href: '/simweb/sim.css' });
    l.dataset.sim = '1';
    document.head.appendChild(l);
  }
  root.innerHTML = HTML;
  build3d();
  $('#sim-clear', root).onclick = () => { control('clear_trail'); clearTrail(); };
  $('#sim-follow', root).onclick = (e) => { S.follow = !S.follow; e.target.classList.toggle('on', S.follow); };
  $('#sim-reads', root).onclick = (e) => {
    S.reads = !S.reads;
    e.target.classList.toggle('on', S.reads);
    root.querySelectorAll('.sim-row.read').forEach((r) => r.classList.toggle('hidden', !S.reads));
  };
  $('#sim-c-ctrl', root).innerHTML = CTRL;
  $('#sim-c-ctrl', root).addEventListener('click', (e) => {
    const b = e.target.closest('button[data-a]');
    if (!b) return;
    const a = b.dataset.a === 'mode' ? (S.state?.program.controlmode === 'REMOTE' ? 'local' : 'remote') : b.dataset.a;
    control(a);
  });
  $('#sim-c-ctrl', root).addEventListener('change', (e) => {
    if (e.target.dataset.a === 'rate') control('rate', Number(e.target.value));
  });
}

export function show() {
  visible = true;
  resize();
  if (!timers.length) {
    pollState();
    timers = [setInterval(pollState, 100), setInterval(pollEvents, 300), setInterval(pollTrail, 400)];
    pollEvents();
    pollTrail();
  }
}

export function hide() {
  visible = false;
  timers.forEach(clearInterval);
  timers = [];
}
