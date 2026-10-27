// Projects: the list on the left, a new project's prompt, and inside a project its circuit, the
// breadboard layout, the chat that refines it and, once the robot has started, the construction.
// A thin client over /api/circuit (dashboard/circuit_design.py).

import { h, sv, drawBoard, isSupply, isGround, ohm, LED } from '/board.js';
import { startBackground } from '/projects/bg3d.js';

const $ = (s) => document.querySelector(s);
// the server's project states (circuit_design.CircuitDesigner.state)
const STATUS = { new: 'new', designing: 'designing', question: 'needs answer', failed: 'needs attention', ready: 'ready to build',
  outdated: 'outdated', waiting: 'queued', running: 'building', waiting_part: 'needs a part', paused: 'paused',
  interrupted: 'interrupted', done: 'built', aborted: 'aborted' };
const SPIN = ['designing', 'waiting', 'running'];               // states with work going on
const LIVE = ['designing', 'waiting', 'running', 'waiting_part'];   // the page polls the state while in these
const spinner = () => h('i', { class: 'spin', 'aria-hidden': 'true' });
const WORK = [['clarify', 'Request'], ['netlist', 'Circuit'], ['solve', 'Layout'], ['assembly', 'Order']];
const IDEAS = [
  ['LED + resistor', 'A yellow LED with a 1 kΩ series resistor on 5 V.'],
  ['RC low-pass', 'An RC low-pass filter between a signal input and an output, 1 kΩ and 100 nF.'],
  ['Voltage divider', 'A voltage divider from 5 V with 10 kΩ and 2 kΩ, output wired by hand.'],
];

let stat = null;      // /api/circuit/static: board, parts, solver
let holes = null;     // every hole the board has
let P = null;         // the open project
let stateTimer = null, clockTimer = null;

// ── list ────────────────────────────────────────────────────────────────────
async function loadList() {
  const list = await fetch('/api/circuit/list').then((r) => r.json()).catch(() => null);
  if (!list) return;
  $('#list').replaceChildren(...list.map((o) => h('li', { class: P?.id === o.id ? 'on' : null, onclick: () => open(o.id) },
    h('div', { class: 't', title: o.text }, o.text || o.id),
    h('div', { class: `s ${o.state}` }, SPIN.includes(o.state) ? spinner() : null, STATUS[o.state] || o.state || ''))));
}

// Parts by id; a part no longer in the stock (an older project) is rebuilt from its id and pins.
function partsFor(circuit) {
  const byId = Object.fromEntries((stat?.parts || []).map((p) => [p.part_id, p]));
  for (const c of circuit?.components || []) {
    if (byId[c.part_id]) continue;
    const pins = [...new Set(circuit.nets.flatMap((n) => n.pins).filter((p) => p.startsWith(`${c.id}.`)).map((p) => p.split('.')[1]))];
    const id = c.part_id, type = id.split('_')[0], parameters = {};
    const r = id.match(/_(\d+(?:\.\d+)?)(k|m)?ohm/i);
    if (r) parameters.resistance_ohm = Number(r[1]) * (r[2] ? (r[2].toLowerCase() === 'k' ? 1e3 : 1e6) : 1);
    const col = id.match(/^led_([a-z]+)/);
    if (col) parameters.color = col[1];
    const gap = id.match(/gap_(\d+)/);
    byId[id] = { part_id: id, type, pins, parameters, pin_spacing: gap ? Number(gap[1]) + 1 : 1, stale: true };
  }
  return byId;
}

// ── circuit diagram: every net a rail, every part a column between its rails ─
function value(part) {
  const q = part?.parameters || {};
  if (q.resistance_ohm) return ohm(q.resistance_ohm);
  if (q.capacitance_F) return `${Math.round(q.capacitance_F * 1e9)} nF`;
  if (q.color) return q.color;
  return part?.type === 'connector' ? 'supply' : '';
}

function drawCircuit(svg, circuit) {
  svg.replaceChildren();
  svg.classList.add('net');
  const pinsOf = {};
  for (const n of circuit.nets) for (const p of n.pins) {
    const [c, pin] = p.split('.');
    (pinsOf[c] ||= []).push({ pin, net: n.id });
  }
  const supplyNets = new Set(), groundNets = new Set();
  for (const n of circuit.nets) {
    if (isSupply(n.id) || n.pins.includes('J_POWER.positive')) supplyNets.add(n.id);
    if (isGround(n.id) || n.pins.includes('J_POWER.negative')) groundNets.add(n.id);
  }
  const nets = [...circuit.nets.filter((n) => supplyNets.has(n.id)), ...circuit.nets.filter((n) => !supplyNets.has(n.id) && !groundNets.has(n.id)),
    ...circuit.nets.filter((n) => groundNets.has(n.id) && !supplyNets.has(n.id))].map((n) => n.id);
  const comps = circuit.components.filter((c) => pinsOf[c.id] && c.id !== 'J_POWER');
  const RG = 56, CG = 92, X0 = 92, Y0 = 26;
  const W = X0 + Math.max(comps.length, 1) * CG + 10, H = Y0 * 2 + (nets.length - 1) * RG + 10;
  svg.setAttribute('viewBox', `0 0 ${W} ${H}`);
  const yOf = Object.fromEntries(nets.map((n, i) => [n, Y0 + i * RG]));
  const xOf = (k) => X0 + k * CG + CG / 2 - 10;
  const span = {};
  comps.forEach((c, k) => pinsOf[c.id].forEach(({ net }) => { (span[net] ||= []).push(xOf(k)); }));
  for (const n of nets) {
    const y = yOf[n], xs = span[n] || [X0];
    sv('line', { x1: X0 - 14, y1: y, x2: Math.max(...xs) + 8, y2: y, class: 'rail', stroke: supplyNets.has(n) ? '#d2483c' : null }, svg);
    const t = sv('text', { x: X0 - 20, y, class: 'netl', 'text-anchor': 'end', 'dominant-baseline': 'central' }, svg);
    t.textContent = supplyNets.has(n) ? `+ ${n}` : groundNets.has(n) ? `− ${n}` : n;
  }
  const partById = partsFor(circuit);
  comps.forEach((c, k) => {
    const part = partById[c.part_id], pins = pinsOf[c.id];
    const ys = pins.map((p) => yOf[p.net]), top = Math.min(...ys), bot = Math.max(...ys);
    const x = xOf(k), mid = top === bot ? top + RG / 2 : (top + bot) / 2;
    const off = (i) => (pins.length > 2 ? (i - (pins.length - 1) / 2) * 12 : 0);
    pins.forEach((p, i) => {
      const px = x + off(i), y = yOf[p.net];
      sv('line', { x1: px, y1: y, x2: px, y2: mid, class: 'lead' }, svg);
      sv('circle', { cx: px, cy: y, r: 3, class: 'dot' }, svg);
    });
    const g = sv('g', { transform: `translate(${x} ${mid})` }, svg);
    const type = part?.type;
    if (type === 'resistor') sv('rect', { x: -6, y: -15, width: 12, height: 30, class: 'sym' }, g);
    else if (type === 'capacitor') {
      sv('rect', { x: -12, y: -4, width: 24, height: 8, fill: 'var(--cream)' }, g);
      for (const dy of [-4, 4]) sv('line', { x1: -12, y1: dy, x2: 12, y2: dy, class: 'lead', 'stroke-width': 3 }, g);
    } else if (type === 'led' && pins.length === 2) {
      const a = pins.find((p) => p.pin === 'anode'), down = a ? yOf[a.net] <= mid : true;
      sv('path', { d: down ? 'M -10 -8 L 10 -8 L 0 8 Z' : 'M -10 8 L 10 8 L 0 -8 Z', class: 'sym', fill: LED[part.parameters?.color] || '#ddd' }, g);
      sv('line', { x1: -10, y1: down ? 8 : -8, x2: 10, y2: down ? 8 : -8, class: 'lead' }, g);
    } else sv('rect', { x: -8 - (pins.length > 2 ? 12 : 0), y: -11, width: 16 + (pins.length > 2 ? 24 : 0), height: 22, class: 'sym' }, g);
    sv('text', { x: 16, y: -5, class: 'lbl' }, g).textContent = c.id;
    sv('text', { x: 16, y: 8, class: 'val' }, g).textContent = value(part);
  });
}

// ── pan and zoom on a drawing: wheel about the pointer, drag to move, double click back ─
// Works on the viewBox, so the lines stay sharp; a redraw with the same box keeps the view.
const zooms = new WeakMap();
function panZoom(svg) {
  const z = { base: null, str: null, vb: null, drag: null };
  zooms.set(svg, z);
  const fit = (vb) => {      // viewBox units per pixel and the box's origin on screen (xMidYMid meet)
    const r = svg.getBoundingClientRect(), s = Math.max(vb[2] / r.width, vb[3] / r.height);
    return { r, s, ox: vb[0] - (r.width * s - vb[2]) / 2, oy: vb[1] - (r.height * s - vb[3]) / 2 };
  };
  const set = () => { svg.setAttribute('viewBox', z.vb.join(' ')); svg.classList.toggle('zoomed', true); };
  svg.addEventListener('wheel', (e) => {
    if (!z.base) return;
    e.preventDefault();
    const vb = z.vb || [...z.base], { r, s, ox, oy } = fit(vb);
    const px = ox + (e.clientX - r.left) * s, py = oy + (e.clientY - r.top) * s;
    const f = Math.min(Math.max(Math.exp(e.deltaY * 0.0015), (z.base[2] / 12) / vb[2]), (z.base[2] * 1.5) / vb[2]);
    z.vb = [px - (px - vb[0]) * f, py - (py - vb[1]) * f, vb[2] * f, vb[3] * f];
    set();
  }, { passive: false });
  svg.addEventListener('pointerdown', (e) => {
    if (!z.base || e.button !== 0) return;
    z.drag = { x: e.clientX, y: e.clientY, vb: z.vb || [...z.base], s: fit(z.vb || z.base).s };
    svg.setPointerCapture(e.pointerId);
  });
  svg.addEventListener('pointermove', (e) => {
    const d = z.drag;
    if (!d) return;
    z.vb = [d.vb[0] - (e.clientX - d.x) * d.s, d.vb[1] - (e.clientY - d.y) * d.s, d.vb[2], d.vb[3]];
    set();
  });
  svg.addEventListener('pointerup', () => { z.drag = null; });
  svg.addEventListener('dblclick', () => { z.vb = null; if (z.str) svg.setAttribute('viewBox', z.str); svg.classList.remove('zoomed'); });
}
function rezoom(svg) {         // after a redraw: a new box resets the view, the same box keeps it
  const z = zooms.get(svg), b = svg.getAttribute('viewBox');
  if (!z || !b) return;
  if (b === z.str) {
    if (z.vb) svg.setAttribute('viewBox', z.vb.join(' '));
    return;
  }
  z.str = b;
  z.base = b.split(/[\s,]+/).map(Number);
  z.vb = null;
  svg.classList.remove('zoomed');
}

// ── board: layout, insertion step-through, what the person wires by hand ────
function boardState() {
  const run = P.run, seq = P.assembly?.sequence || [];
  if (run) {
    const steps = run.steps;
    return {
      ghost: new Set(steps.filter((s) => s.status === 'pending').map((s) => s.item)),
      current: steps.find((s) => s.status === 'active')?.item,
      failed: new Set(steps.filter((s) => s.status === 'failed').map((s) => s.item)),
    };
  }
  if (P.step == null || !seq.length) return {};
  return { ghost: new Set(seq.slice(P.step + 1).map((s) => s.item)), current: seq[P.step].item };
}

function renderBoard() {
  const bd = $('#board'), seq = P.assembly?.sequence || [];
  const partById = partsFor(P.circuit);
  try {
    drawBoard(bd, { board: stat.board, partById, circuit: P.circuit, mapping: P.mapping, assembly: P.assembly },
      { view: 'fit', casings: !P.run, badges: !!P.assembly && !P.run && P.step == null, labels: true, power: true, state: boardState() });
  } catch (e) {
    bd.replaceChildren();
    console.warn('board', e);
  }
  rezoom(bd);

  // insertion step-through, before the build
  const tools = $('#board-tools');
  if (seq.length && !P.run) {
    const k = P.step;
    const go = (to) => { P.step = to; renderBoard(); };
    tools.replaceChildren(
      h('button', { onclick: () => go(k == null ? seq.length - 1 : k > 0 ? k - 1 : null) }, '‹'),
      h('span', {}, k == null ? `${seq.length} steps` : `${k + 1}/${seq.length} ${seq[k].item} → ${seq[k].holes.join('–')}`),
      h('button', { onclick: () => go(k == null ? 0 : k < seq.length - 1 ? k + 1 : null) }, '›'));
  } else tools.replaceChildren();

  // the foot: hand-wired holes, holes or parts the board and stock no longer have
  const foot = [];
  const mc = P.mapping?.manual_connections || {};
  if (Object.keys(mc).length) {
    foot.push(h('div', {}, h('span', { class: 'k' }, 'By hand '), Object.entries(mc).map(([pin, hole]) => {
      const [c, p] = pin.split('.');
      return `${c === 'J_POWER' ? ({ positive: '+', negative: '−' }[p] || p) : pin} ${hole}`;
    }).join(' · ')));
  }
  const { missing, gone } = stale();
  if (missing.length) {
    foot.push(h('div', { class: 'warn' }, `Not on the board: ${missing.join(' ')} `,
      can('solve') && !gone.length ? h('button', { onclick: solveAgain, disabled: P.running }, 'Solve again') : null));
  }
  $('#board-foot').replaceChildren(...foot);
}

// holes the board no longer has, parts no longer in stock: such a layout cannot be built
function stale() {
  const mc = P.mapping?.manual_connections || {}, partById = partsFor(P.circuit);
  const missing = P.mapping ? [...Object.values(P.mapping.pin_to_hole), ...Object.values(mc),
    ...P.mapping.jumpers.flatMap((j) => Object.values(j.pins))].filter((x) => !holes.has(x)) : [];
  const gone = (P.circuit?.components || []).filter((c) => partById[c.part_id]?.stale).map((c) => c.id);
  return { missing, gone };
}

// ── construction: only once the robot has started ──────────────────────────
const can = (a) => !!P.view?.actions?.includes(a);
const CONTROLS = [['pause', 'Pause'], ['resume', 'Resume'], ['retry', 'Retry'], ['skip', 'Skip step'], ['abort', 'Abort']];

function renderProgress() {
  const run = P.run, pane = $('#p-progress');
  pane.hidden = !run;
  if (!run) return;
  const st = P.view?.state || run.status;
  const done = run.steps.filter((s) => s.status === 'done').length;
  const last = [...run.log || []].reverse().find((l) => l.level !== 'info');
  $('#progress').replaceChildren(
    h('div', { class: 'run-head' },
      h('span', { class: `s ${st}` }, SPIN.includes(st) ? spinner() : null, STATUS[st] || st),
      h('span', {}, `${done} / ${run.steps.length}`),
      run.mode === 'plan' ? h('span', { class: 'mode', title: 'circuit.build.motion is off: the skills run in plan mode' }, 'plan only') : null,
      h('span', { class: 'ctl' }, ...CONTROLS.filter(([a]) => can(a)).map(([a, label]) =>
        h('button', { class: a === 'abort' ? 'danger' : null, onclick: () => control(a) }, label)))),
    h('div', { class: 'bar' }, h('i', { style: `width:${(100 * done) / Math.max(run.steps.length, 1)}%` })),
    h('div', { class: 'steps' }, ...run.steps.map((s) => h('span', { class: `step ${s.status}`, title: s.note || null },
      s.status === 'active' ? spinner() : null,
      s.item + (s.status === 'active' && s.phase ? ` · ${s.phase}` : '') + (s.attempts > 1 ? ` ×${s.attempts}` : '')))),
    can('slot') && run.need ? h('div', { class: 'need' }, h('span', {}, `Rack slot with ${run.need.casing}`),
      h('span', { class: 'slots' }, ...Array.from({ length: 20 }, (_, i) =>
        h('button', { onclick: () => control('slot', { slot: `R${i + 1}` }) }, `R${i + 1}`)))) : null,
    last && ['paused', 'aborted', 'failed', 'waiting_part', 'interrupted'].includes(st) ? h('div', { class: `why ${last.level}` }, last.text) : null);
}

async function control(action, body = {}) {
  const r = await fetch(`/api/circuit/${P.id}/build/${action}`, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
  const j = await r.json().catch(() => ({}));
  if (!r.ok) apply({ t: 'error', text: j.reason || `HTTP ${r.status}` });
  else setView(j);
}

function render() {
  if (P.circuit) drawCircuit($('#schematic'), P.circuit);
  else $('#schematic').replaceChildren();
  rezoom($('#schematic'));
  renderBoard();
  renderProgress();
  renderActions();
}

// the project's state from the server; polled while something runs
function setView(v) {
  const was = P.view?.state;
  P.view = v;
  P.run = v.run;
  render();
  if (was !== v.state) loadList();
  clearTimeout(stateTimer);
  if (LIVE.includes(v.state) || P.running) stateTimer = setTimeout(refresh, 2000);
}
async function refresh() {
  clearTimeout(stateTimer);
  if (!P?.id) return;
  const id = P.id;
  const r = await fetch(`/api/circuit/${id}/state`).catch(() => null);
  if (P?.id !== id) return;
  if (r?.ok) setView(await r.json());
  else stateTimer = setTimeout(refresh, 4000);
}

// ── chat: progress of a turn, Stop, Build ───────────────────────────────────
function renderWork() {
  const box = $('#work');
  box.hidden = !P.running || !P.id;
  if (box.hidden) return;
  const secs = Math.round((Date.now() - P.turnStart) / 1000);
  box.replaceChildren(
    h('div', { class: 'stages' }, ...WORK.map(([k, label]) => h('span', { class: `stg ${P.stages[k] || 'pending'}` },
      P.stages[k] === 'active' ? spinner() : null, label))),
    h('span', { class: 'clock' }, `${Math.floor(secs / 60)}:${String(secs % 60).padStart(2, '0')}`),
    h('button', { onclick: () => fetch(`/api/circuit/${P.id}/stop`, { method: 'POST' }) }, 'Stop'));
}

function renderActions() {
  const st = P.view?.state, idle = !P.running;
  const out = [];
  if (idle && can('build')) {
    out.push(h('button', { class: `launch${P.launching ? ' busy' : ''}`, onclick: build, disabled: P.launching },
      P.launching ? spinner() : playIcon(),
      h('span', {}, 'Build it'), h('small', {}, `${P.assembly?.sequence?.length || 0} steps`)));
  }
  if (idle && st === 'outdated') {
    out.push(h('button', { class: 'redo', onclick: () => say('Redesign this circuit with the parts in stock and the board as they are now.') }, 'Redesign'));
  }
  $('#actions').replaceChildren(...out);
  $('#chat-form').hidden = !!st && !can('message') && !P.running;
}

function playIcon() {
  const svg = sv('svg', { viewBox: '0 0 24 24', 'aria-hidden': 'true' });
  sv('path', { d: 'M7 4.5v15l12-7.5z' }, svg);
  return svg;
}

async function build() {
  P.launching = true;
  renderActions();
  const r = await fetch(`/api/circuit/${P.id}/build`, { method: 'POST' });
  const j = await r.json().catch(() => ({}));
  P.launching = false;
  if (!r.ok) { apply({ t: 'error', text: j.reason || `HTTP ${r.status}` }); renderActions(); return; }
  P.step = null;
  setView(j);
}

function push(el) {
  const t = $('#thread');
  t.append(el);
  t.scrollTop = t.scrollHeight;
}

function questions(ev) {
  const picks = ev.questions.map(() => new Set());
  const send = h('button', { class: 'primary', disabled: true }, 'Send');
  const card = h('div', { class: 'q' });
  ev.questions.forEach((q, i) => {
    const opts = h('div', { class: 'opts' });
    for (const o of q.options || []) {
      const b = h('button', { type: 'button', title: o.description || null }, o.label);
      b.onclick = () => {
        if (!q.multiSelect) { picks[i].clear(); opts.querySelectorAll('button').forEach((x) => x.classList.remove('on')); }
        if (picks[i].has(o.label)) { picks[i].delete(o.label); b.classList.remove('on'); } else { picks[i].add(o.label); b.classList.add('on'); }
        send.disabled = P.running || picks.some((p) => !p.size);
      };
      opts.append(b);
    }
    card.append(h('p', {}, q.question), opts);
  });
  send.onclick = () => say(`My answers:\n${ev.questions.map((q, i) => `- ${q.question} → ${[...picks[i]].join('; ')}`).join('\n')}`);
  card.append(send);
  push(card);
}

function apply(ev) {
  if (ev.i !== undefined) { if (ev.i < P.seen) return; P.seen = ev.i + 1; }
  switch (ev.t) {
    case 'user':
      document.querySelectorAll('.q button').forEach((b) => { b.disabled = true; });
      if (!$('#title').textContent) $('#title').textContent = ev.text;
      push(h('p', { class: 'msg user' }, ev.text));
      P.stages = {};
      break;
    case 'agent': if (ev.text) push(h('p', { class: 'msg' }, ev.text)); break;
    case 'questions': questions(ev); break;
    case 'error': push(h('p', { class: 'msg error' }, ev.text)); break;
    case 'stage':
      P.stages[ev.stage] = ev.state;
      renderWork();
      break;
    case 'circuit': P.circuit = ev.circuit; P.mapping = P.assembly = null; P.step = null; render(); break;
    case 'mapping': P.mapping = ev.mapping; P.assembly = null; render(); break;
    case 'assembly': P.assembly = ev; render(); break;
    case 'handoff': P.handoff = true; break;
    case 'turn_end': refresh(); break;
  }
  renderActions();
}

async function read(r) {
  const rd = r.body.getReader(), dec = new TextDecoder();
  let buf = '';
  for (;;) {
    const { value: chunk, done } = await rd.read();
    if (done) break;
    buf += dec.decode(chunk, { stream: true });
    let k;
    while ((k = buf.indexOf('\n')) >= 0) {
      const line = buf.slice(0, k).trim();
      buf = buf.slice(k + 1);
      if (line) apply(JSON.parse(line));
    }
  }
}

function busy(on, live = true) {
  P.running = on && live;
  P.turnStart = Date.now();
  $('#send').disabled = on;
  $('#start-form button').disabled = on;
  if (on) document.querySelectorAll('.q button').forEach((b) => { b.disabled = true; });
  clearInterval(clockTimer);
  if (P.running) clockTimer = setInterval(renderWork, 1000);
  renderWork();
  renderActions();
  if (!on && P.id) refresh();
}

// ── navigation ──────────────────────────────────────────────────────────────
function fresh(id) {
  P?.ctrl?.abort();
  clearTimeout(stateTimer);
  clearInterval(clockTimer);
  P = { id, seen: 0, running: false, circuit: null, mapping: null, assembly: null, run: null, handoff: false,
    view: null, launching: false, stages: {}, step: null, turnStart: 0, ctrl: new AbortController() };
}

const embedded = new URLSearchParams(location.search).has('embed');     // the dashboard's Projects tab
if (embedded) document.body.classList.add('embed');
function address(id) {
  history.replaceState(null, '', `/projects?${embedded ? 'embed=1&' : ''}${id ? `id=${id}` : ''}`.replace(/[?&]$/, ''));
  if (embedded) window.parent.postMessage({ project: id }, location.origin);
}

function showStart() {
  fresh(null);
  address(null);
  $('#start').hidden = false;
  $('#project').hidden = true;
  $('#start-text').focus();
  loadList();
}

function showProject(id) {
  address(id);
  $('#start').hidden = true;
  $('#project').hidden = false;
  $('#thread').replaceChildren();
  $('#title').textContent = '';
  render();
  loadList();
}

async function open(id) {
  fresh(id);
  showProject(id);
  busy(true, false);
  try {
    const r = await fetch(`/api/circuit/${id}/events?from=0`, { signal: P.ctrl.signal });
    if (r.ok) {
      // a turn still running keeps this stream open: show it as working once the replay is in
      setTimeout(() => { if (P.id === id && $('#send').disabled) busy(true); }, 800);
      await read(r);
    }
  } catch (e) { if (e.name !== 'AbortError') apply({ t: 'error', text: String(e) }); }
  if (P.id === id) busy(false);
}

async function stream(url, body) {
  if (P.running) return;
  const ctrl = P.ctrl;
  busy(true);
  try {
    const r = await fetch(url, { method: 'POST', signal: ctrl.signal, headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
    if (!r.ok) apply({ t: 'error', text: (await r.json().catch(() => ({}))).reason || `HTTP ${r.status}` });
    else {
      if (!P.id) { P.id = r.headers.get('X-Design'); showProject(P.id); renderWork(); }
      await read(r);
    }
  } catch (e) { if (e.name !== 'AbortError') apply({ t: 'error', text: String(e) }); }
  if (P.ctrl === ctrl) busy(false);
}

const say = (text) => text.trim() && stream('/api/circuit', { text, design: P.id });
const solveAgain = () => stream(`/api/circuit/${P.id}/solve`, { solver: stat.solver });

// Enter sends; → at the end takes the rest of the suggestion (the placeholder), as in a terminal
function submitOnEnter(ta, form) {
  ta.onkeydown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) { e.preventDefault(); form.requestSubmit(); return; }
    const hint = ta.placeholder, v = ta.value;
    if (e.key === 'ArrowRight' && 'suggest' in ta.dataset && ta.selectionStart === v.length && v.length < hint.length && hint.toLowerCase().startsWith(v.toLowerCase())) {
      e.preventDefault();
      ta.value = v + hint.slice(v.length);
      ta.setSelectionRange(ta.value.length, ta.value.length);
    }
  };
}
$('#start-form').onsubmit = (e) => { e.preventDefault(); const t = $('#start-text'); const v = t.value; t.value = ''; say(v); };
$('#chat-form').onsubmit = (e) => { e.preventDefault(); const t = $('#chat-text'); const v = t.value; t.value = ''; say(v); };
submitOnEnter($('#start-text'), $('#start-form'));
submitOnEnter($('#chat-text'), $('#chat-form'));
$('#new').onclick = showStart;
$('#ideas').replaceChildren(...IDEAS.map(([label, text]) => h('button', { type: 'button', onclick: () => { $('#start-text').value = text; $('#start-text').focus(); } }, label)));
setInterval(() => { if (!document.hidden) loadList(); }, 10000);

// the project list's width: drag its right edge
{
  const side = $('#side'), set = (w) => side.style.setProperty('--side', `${Math.min(Math.max(w, 180), 520)}px`);
  try { const w = Number(localStorage.getItem('projectsSide')); if (w) set(w); } catch { /* storage blocked */ }
  $('#grip').onpointerdown = (e) => {
    e.preventDefault();
    document.body.classList.add('resizing');
    const move = (ev) => set(ev.clientX - side.getBoundingClientRect().left);
    const up = () => {
      document.body.classList.remove('resizing');
      removeEventListener('pointermove', move);
      try { localStorage.setItem('projectsSide', String(side.getBoundingClientRect().width)); } catch { /* storage blocked */ }
    };
    addEventListener('pointermove', move);
    addEventListener('pointerup', up, { once: true });
  };
}
try { startBackground($('#start')); } catch (e) { console.warn('background', e); }
panZoom($('#schematic'));
panZoom($('#board'));

stat = await fetch('/api/circuit/static').then((r) => r.json());
holes = new Set(stat.board.conductive_groups.flatMap((g) => g.holes));
const q = new URLSearchParams(location.search).get('id');
if (q) open(q); else showStart();
