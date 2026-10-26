// ── circuit assembly ────────────────────────────────────────────────────────
// Describe a circuit, answer the designer's questions, watch it become a netlist, a CSP
// layout and an assembly order, then hand it to the assembly pipeline and follow the build
// (dashboard/circuit_design.py). The tab replays the design's events from the server, so a
// reload or a second browser picks up where it was. Until there is a layout the conversation
// fills the tab; then the board takes the stage and the conversation moves to the side.

const $ = (s) => document.querySelector(s);
const SVGNS = 'http://www.w3.org/2000/svg';
const PITCH = 2.54;

function h(tag, attrs, ...kids) {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs || {})) {
    if (v == null || v === false) continue;
    if (k.startsWith('on')) e[k] = v;
    else e.setAttribute(k, v === true ? '' : v);
  }
  for (const k of kids.flat()) if (k != null && k !== false) e.append(k);
  return e;
}
function sv(tag, attrs, parent) {
  const e = document.createElementNS(SVGNS, tag);
  for (const [k, v] of Object.entries(attrs || {})) if (v != null) e.setAttribute(k, v);
  if (parent) parent.appendChild(e);
  return e;
}
const pad2 = (n) => String(n).padStart(2, '0');
const clock = (s) => `${pad2(Math.floor(s / 60))}:${pad2(Math.floor(s % 60))}`;

// the rail: server stages grouped into what a person follows
const RAIL = [
  ['Describe', ['request']], ['Clarify', ['clarify']], ['Layout', ['netlist', 'solve']],
  ['Plan', ['assembly']], ['Approve', ['review']], ['Build', ['build']],
];
const DOING = {
  clarify: 'Reading your request', netlist: 'Writing the netlist',
  solve: 'Placing the parts on the breadboard', assembly: 'Planning the insertion order',
};

let stat = null;            // board, parts, gripper
let partById = {};
let D = fresh();
let viewMode = 'fit';
let showCasings = true;
let playTimer = null;

function fresh() {
  return { id: null, n: 0, stages: {}, circuit: null, mapping: null, assembly: null,
    phase: 'design', running: false, stream: null, run: null, step: 0, view: 'design', activeStage: null, workStart: null };
}

// ── names and colours ──────────────────────────────────────────────────────
const ohm = (v) => (v >= 1e6 ? `${v / 1e6} MΩ` : v >= 1e3 ? `${v / 1e3} kΩ` : `${v} Ω`);
function partLabel(p) {
  if (!p) return '';
  const q = p.parameters || {};
  switch (p.type) {
    case 'resistor': return `${ohm(q.resistance_ohm)} resistor`;
    case 'led': return `${q.color} LED`;
    case 'rgb_led': return 'RGB LED';
    case 'jumper': return `jumper, ${p.pin_spacing} pitch`;
    case 'connector': return 'power connector';
    case 'capacitor': return q.polarized ? `${q.capacitance_F * 1e6} µF electrolytic` : `${Math.round(q.capacitance_F * 1e9)} nF capacitor`;
    case 'bjt': return `${q.polarity} transistor`;
    case 'potentiometer': return `${ohm(q.resistance_ohm)} potentiometer`;
    case 'switch': return q.momentary ? 'pushbutton' : `${q.contact_form} switch`;
    default: return (p.name || p.part_id).replace(/ - \d gap$/, '');
  }
}
const NET_PALETTE = ['#c9a15d', '#3f8f86', '#3d6fb6', '#8a4f7d', '#6b7a2f', '#d9822b', '#5d6b7a', '#a4553a', '#2f7a9a', '#7c6a3a'];
function netColours(circuit) {
  const col = {};
  let k = 0;
  for (const n of circuit.nets) {
    if (/^(VCC|VDD|VIN|V\+|\+?\d+V\d*)$/i.test(n.id)) col[n.id] = '#d2483c';
    else if (/^(GND|VSS|0V)$/i.test(n.id)) col[n.id] = '#111111';
    else col[n.id] = NET_PALETTE[k++ % NET_PALETTE.length];
  }
  for (const p of circuit.unconnected_pins || []) col[`UNCONNECTED:${p}`] = '#8c8577';
  return col;
}
function pinNets(circuit) {
  const m = {};
  for (const n of circuit.nets) for (const p of n.pins) m[p] = n.id;
  for (const p of circuit.unconnected_pins || []) m[p] = `UNCONNECTED:${p}`;
  return m;
}

// ── rail and status line ───────────────────────────────────────────────────
function railState(keys) {
  const sts = keys.map((k) => D.stages[k]?.state || 'pending');
  for (const s of ['active', 'failed', 'wait']) if (sts.includes(s)) return s;
  if (sts.every((s) => s === 'done')) return 'done';
  return sts.includes('done') ? 'active' : 'pending';
}

function renderRail() {
  $('#ca-rail').replaceChildren(...RAIL.map(([label, keys]) => h('li', { class: railState(keys) }, h('i'), h('span', {}, label))));
  renderStatus();
}

function renderStatus() {
  const el = $('#ca-status');
  const solve = D.stages.solve;
  let text = '', cls = '';
  if (D.running && D.activeStage) text = DOING[D.activeStage] || 'Working';
  else if (D.phase === 'handoff') text = D.stages.build?.detail || 'Handed to the assembly agent';
  else if (D.phase === 'review') text = 'Check the plan, then start the assembly';
  else if (D.stages.clarify?.state === 'wait' && D.stages.request?.state === 'done') text = 'The designer is waiting for your answer';
  else if (D.stages.request?.state === 'wait') text = 'Describe a circuit to build';
  if (D.lastError && !D.running) { text = D.lastError; cls = 'bad'; }
  el.className = `ca-status ${cls}`;
  el.replaceChildren(h('span', { class: 'txt' }, text), h('span', { class: 't', id: 'ca-clock' }),
    h('b', { class: 'new', onclick: restart, title: 'Discard this design and start over' }, D.id ? 'New design' : ''));
  const bar = $('#ca-bar');
  bar.classList.toggle('on', !!(D.running && D.activeStage));
  bar.classList.toggle('det', !!(D.running && D.activeStage === 'solve' && solve?.budget));
  tick();
}

// timers: elapsed time of the running step, the solver's budget, the build clock
function tick() {
  const c = $('#ca-clock');
  if (c) {
    const s = D.stages.solve;
    if (D.running && D.workStart) {
      const t = Date.now() / 1000 - (D.activeStage === 'solve' && s?.started ? s.started : D.workStart);
      c.textContent = D.activeStage === 'solve' && s?.budget ? `${Math.floor(t)} / ${s.budget} s` : clock(t);
    } else c.textContent = '';
    if (D.running && D.activeStage === 'solve' && s?.budget) {
      $('#ca-bar i').style.width = `${Math.min(100, (100 * (Date.now() / 1000 - s.started)) / s.budget)}%`;
    }
  }
  if (D.view === 'run' && D.run) runClock();
}
setInterval(tick, 250);

// ── conversation ───────────────────────────────────────────────────────────
const log = () => $('#ca-log');
function push(el) {
  const l = log();
  const w = $('#ca-working');
  l.insertBefore(el, w && w.parentElement === l ? w : null);
  l.scrollTop = l.scrollHeight;
  return el;
}

function addUser(ev) {
  D.lastError = null;
  $('#ca-intro')?.remove();
  $('#ca-design').classList.remove('blank');
  for (const q of document.querySelectorAll('.ca-q:not(.done)')) q.classList.add('done');
  push(h('div', { class: 'ca-m user' }, ev.text));
}
function addAgent(ev) {
  if (ev.text) push(h('div', { class: `ca-m agent${ev.kind === 'refuse' ? ' refuse' : ''}` }, ev.text));
}
function addReason(ev) {
  push(h('details', { class: 'ca-why' }, h('summary', {}, 'Why'), h('p', {}, ev.text)));
}
// solver retries are worth a line; the agent's timings are not
function addTrace(ev) {
  let t = null;
  if (/UNKNOWN/.test(ev.text)) t = 'No layout found in time, searching again';
  else if (/INFEASIBLE/.test(ev.text)) t = 'It does not fit in this part of the board, trying the whole board';
  else if (/^solver/.test(ev.text)) t = ev.text.replace(/^solver[^:]*: /, '');
  if (t) push(h('div', { class: 'ca-m sys' }, t));
}
function addError(ev) {
  D.lastError = ev.text === 'Stopped.' ? null : 'Something went wrong, see the conversation';
  push(h('div', { class: 'ca-m error' }, ev.text));
}

function addQuestions(ev) {
  const picks = ev.questions.map(() => new Set());
  const others = [];
  const n = ev.questions.length;
  const card = h('div', { class: 'ca-q' }, h('div', { class: 'ca-q-head' }, n === 1 ? 'One question' : `${n} questions`));
  ev.questions.forEach((q, i) => {
    const opts = h('div', { class: 'ca-opts' });
    q.options.forEach((o) => {
      const b = h('button', { type: 'button', class: 'ca-opt', title: o.description || '' },
        h('b', {}, o.label.replace(/\s*\(Recommended\)\s*$/i, '')),
        /\(Recommended\)/i.test(o.label) ? h('span', { class: 'rec' }, 'Recommended') : null,
        o.description ? h('span', { class: 'desc' }, o.description) : null);
      b.onclick = () => {
        if (card.classList.contains('done')) return;
        if (!q.multiSelect) { picks[i].clear(); opts.querySelectorAll('.ca-opt').forEach((x) => x.classList.remove('on')); }
        if (picks[i].has(o.label)) { picks[i].delete(o.label); b.classList.remove('on'); } else { picks[i].add(o.label); b.classList.add('on'); }
        if (!q.multiSelect) others[i].value = '';
        ready();
      };
      opts.append(b);
    });
    others[i] = h('input', { type: 'text', class: 'ca-other', placeholder: 'Something else…' });
    others[i].oninput = () => {
      if (others[i].value.trim() && !q.multiSelect) { picks[i].clear(); opts.querySelectorAll('.ca-opt').forEach((x) => x.classList.remove('on')); }
      ready();
    };
    card.append(h('div', { class: 'ca-qq' }, h('p', {}, q.question), opts, others[i]));
  });
  const send = h('button', { type: 'button', class: 'primary', disabled: true }, 'Answer');
  const ready = () => {
    send.disabled = D.running || ev.questions.some((_, i) => !picks[i].size && !others[i].value.trim());
  };
  send.onclick = () => {
    const lines = ev.questions.map((q, i) => {
      const a = [...picks[i]];
      if (others[i].value.trim()) a.push(others[i].value.trim());
      return `- ${q.question} → ${a.join('; ')}`;
    });
    card.classList.add('done');
    sendText(`My answers:\n${lines.join('\n')}`);
  };
  card.append(h('div', { class: 'ca-q-foot' }, send));
  push(card);
}

function addCircuit(ev) {
  const c = ev.circuit;
  const col = netColours(c);
  push(h('div', { class: 'ca-net' },
    h('div', { class: 'ca-net-head' }, h('b', {}, 'Netlist'), h('span', {}, `${c.components.length} parts · ${c.nets.length} nets`)),
    h('p', {}, c.description),
    h('div', { class: 'ca-chips' }, c.nets.map((n) => h('span', { title: n.pins.join(', ') }, h('i', { style: `background:${col[n.id]}` }), n.id))),
    h('details', {}, h('summary', {}, 'Connections'),
      h('div', { class: 'ca-netrows' }, c.nets.map((n) => h('div', {}, h('b', {}, n.id), h('span', {}, n.pins.join('  ')))))),
    c.assumptions?.length ? h('details', {}, h('summary', {}, 'Assumptions'), h('ul', {}, c.assumptions.map((a) => h('li', {}, a)))) : null));
}

function working(on) {
  const w = $('#ca-working');
  if (!on) { w?.remove(); D.workStart = null; return; }
  D.workStart ??= Date.now() / 1000;
  if (w) return;
  log().append(h('div', { id: 'ca-working', class: 'ca-working' }, h('span', { class: 'what' }, DOING[D.activeStage] || 'Thinking'), h('i')));
  log().scrollTop = log().scrollHeight;
}
function relabelWorking() {
  const w = $('#ca-working');
  if (w) w.querySelector('.what').textContent = DOING[D.activeStage] || 'Thinking';
  D.workStart = Date.now() / 1000;
}

// ── events ─────────────────────────────────────────────────────────────────
function apply(ev) {
  if (ev.i != null) D.n = ev.i + 1;
  switch (ev.t) {
    case 'user': addUser(ev); break;
    case 'agent': addAgent(ev); break;
    case 'reasoning': addReason(ev); break;
    case 'trace': addTrace(ev); break;
    case 'note': push(h('div', { class: 'ca-m sys' }, ev.text)); break;
    case 'error': addError(ev); break;
    case 'questions': addQuestions(ev); break;
    case 'circuit': D.circuit = ev.circuit; addCircuit(ev); break;
    case 'mapping': D.mapping = ev.mapping; D.solveSecs = ev.seconds; renderWork(); break;
    case 'assembly': D.assembly = ev; D.step = ev.sequence.length; renderWork(); break;
    case 'stage':
      D.stages[ev.stage] = ev;
      if (ev.state === 'active') { D.activeStage = ev.stage; relabelWorking(); }
      if (ev.stage === 'review' && ev.state === 'wait') D.phase = 'review';
      renderRail();
      renderActions();
      break;
    case 'handoff':
      D.phase = 'handoff';
      D.stages.build = { state: 'wait', detail: 'Waiting for the assembly agent' };
      renderRail();
      renderActions();
      break;
    default: break;
  }
}

async function readStream(r) {
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
      if (line) apply(JSON.parse(line));
    }
  }
}

function busy(on) {
  D.running = on;
  working(on);
  updateInput();
  renderStatus();
  renderActions();
  for (const b of document.querySelectorAll('.ca-q:not(.done) .ca-q-foot .primary')) if (on) b.disabled = true;
}

async function sendText(text) {
  if (D.running || !text.trim()) return;
  busy(true);
  const ac = new AbortController();
  D.stream = ac;
  try {
    const r = await fetch('/api/circuit', {
      method: 'POST', headers: { 'Content-Type': 'application/json' }, signal: ac.signal,
      body: JSON.stringify({ text, design: D.id }),
    });
    if (!r.ok) {
      const j = await r.json().catch(() => ({}));
      addError({ text: j.reason || `HTTP ${r.status}` });
    } else {
      if (!D.id) { D.id = r.headers.get('X-Design'); remember(D.id); address(); }
      await readStream(r);
    }
  } catch (e) {
    if (e.name !== 'AbortError') addError({ text: String(e) });
  }
  D.stream = null;
  busy(false);
}

async function attach(id) {
  const r = await fetch(`/api/circuit/${encodeURIComponent(id)}/events?from=0`).catch(() => null);
  if (!r || !r.ok) { remember(null); return false; }
  D.id = id;
  address();
  busy(true);
  const ac = new AbortController();
  D.stream = ac;
  try { await readStream(r); } catch { /* left */ }
  D.stream = null;
  busy(false);
  if (D.phase === 'handoff') { await pollRun(); showView('run'); }
  return true;
}

// #circuit:<id> opens that design (link to share, or for a second screen)
function address() {
  if (!$('#tab-circuit').classList.contains('hidden')) history.replaceState(null, '', D.id ? `#circuit:${D.id}` : '#circuit');
}

function remember(id) {
  try { if (id) localStorage.setItem('circuitDesign', id); else localStorage.removeItem('circuitDesign'); } catch { /* storage blocked */ }
}

async function restart() {
  if (D.running && D.id) await fetch(`/api/circuit/${D.id}/stop`, { method: 'POST' }).catch(() => {});
  D.stream?.abort();
  stopPlay();
  D = fresh();
  remember(null);
  address();
  log().replaceChildren();
  intro();
  renderRail();
  renderWork();
  showView('design');
  updateInput();
  $('#ca-text').focus();
}

const EXAMPLES = [
  ['Five LEDs in parallel', 'Five red LEDs in parallel on 5 V, each with its own 220 Ω resistor'],
  ['Transistor LED driver', 'An NPN transistor that switches a red LED from a 5 V control input, with a 1 kΩ base resistor'],
  ['RC low-pass filter', 'An RC low-pass filter between a signal input and an output, 1 kΩ and 100 nF'],
];
function intro() {
  $('#ca-design').classList.add('blank');
  log().append(h('div', { class: 'ca-intro', id: 'ca-intro' },
    h('h2', {}, 'What should the robot build?'),
    h('p', {}, 'Describe a circuit in your own words. The designer asks about anything unclear, lays it out on the breadboard and plans the build.'),
    h('div', { class: 'ca-ex' }, EXAMPLES.map(([label, text]) => h('button', { type: 'button', title: text,
      onclick: () => { $('#ca-text').value = text; $('#ca-text').focus(); } }, label)))));
}

function updateInput() {
  const form = $('#ca-form');
  form.classList.toggle('hidden', D.phase !== 'design' || !!D.mapping);
  const ta = $('#ca-text');
  ta.placeholder = D.running ? 'The designer is working…' : D.id ? 'Reply to the designer' : 'For example: a green LED with a 220 Ω resistor on 5 V';
  $('#ca-send').textContent = D.running ? 'Stop' : 'Send';
}

// ── board drawing ──────────────────────────────────────────────────────────
// rows run left to right, column J on top; finger pads may hang over the board edge
const P = 18, MX = 34, MY = 2 * P;     // the svg overflows: finger pads may hang past the board

function boardModel() {
  const G = stat.board.geometry, R = G.rows, colPos = G.column_positions;
  const cols = Object.values(colPos);
  return { G, R, colPos, maxCol: Math.max(...cols), minCol: Math.min(...cols), rowPos: (r) => R.first_position + (r - R.first) * R.spacing };
}

function usedRows() {
  if (!D.mapping) return [];
  const holes = [...Object.values(D.mapping.pin_to_hole), ...Object.values(D.mapping.manual_connections || {}),
    ...D.mapping.jumpers.flatMap((j) => Object.values(j.pins))];
  return holes.map((x) => parseInt(x.slice(1), 10));
}

function drawBoard(svg, opt) {
  const bm = boardModel();
  const { R, colPos, maxCol, minCol, rowPos } = bm;
  let lo = R.first, hi = R.last;
  const rows = usedRows();
  if (opt.view === 'fit' && rows.length) {
    const a = Math.min(...rows), b = Math.max(...rows), want = Math.max(opt.minRows || 20, b - a + 9);
    lo = Math.max(R.first, Math.min(Math.round((a + b - want) / 2), R.last - want + 1));
    hi = Math.min(R.last, lo + want - 1);
  } else if (opt.view === 'fit') { hi = Math.min(R.last, R.first + 29); }
  const X = (u) => MX + (u / PITCH - rowPos(lo)) * P;            // u, v in mm (assembly_plan.py)
  const Y = (v) => MY + (maxCol - v / PITCH) * P;
  const rowOf = (hole) => parseInt(hole.slice(1), 10);
  const inView = (hole) => rowOf(hole) >= lo && rowOf(hole) <= hi;
  const xy = (hole) => [MX + (rowPos(rowOf(hole)) - rowPos(lo)) * P, MY + (maxCol - colPos[hole[0]]) * P];
  const W = 2 * MX + (rowPos(hi) - rowPos(lo)) * P, H = 2 * MY + (maxCol - minCol) * P;
  svg.replaceChildren();
  svg.setAttribute('viewBox', `0 0 ${W} ${H}`);

  sv('rect', { x: MX - 26, y: MY - 30, width: W - 2 * MX + 52, height: H - 2 * MY + 60, fill: 'var(--cream)', class: 'bb' }, svg);
  const upper = Object.values(colPos).filter((c) => c > 4);
  if (upper.length) {
    const gapTop = MY + (maxCol - Math.min(...upper)) * P;
    sv('rect', { x: MX - 10, y: gapTop + P * 0.8, width: W - 2 * MX + 20, height: P * 1.4, fill: 'var(--beige)' }, svg);
  }
  for (const [letter, c] of Object.entries(colPos)) {
    const y = MY + (maxCol - c) * P;
    for (const x of [MX - 17, W - MX + 17]) sv('text', { x, y, class: 'ax', 'text-anchor': 'middle', 'dominant-baseline': 'central' }, svg).textContent = letter;
  }
  const step = hi - lo < 40 ? 1 : 5;
  for (let r = lo; r <= hi; r++) {
    if (r !== lo && r % step) continue;
    const x = MX + (rowPos(r) - rowPos(lo)) * P;
    sv('text', { x, y: MY - 17, class: 'ax', 'text-anchor': 'middle' }, svg).textContent = r;
    sv('text', { x, y: H - MY + 24, class: 'ax', 'text-anchor': 'middle' }, svg).textContent = r;
  }

  const L = (name) => sv('g', { class: name }, svg);
  const stripsG = L('strips'), holesG = L('holes'), casG = L('casings'), wiresG = L('wires'), partsG = L('parts'),
    fingG = L('fingers'), labelsG = L('labels'), badgeG = L('badges');

  if (opt.empty && upper.length) {
    const gy = MY + (maxCol - Math.min(...upper)) * P + P * 1.5;
    sv('text', { x: W / 2, y: gy, class: 'empty', 'text-anchor': 'middle', 'dominant-baseline': 'central' }, svg).textContent = opt.empty;
  }
  const holeGroup = {};
  stat.board.conductive_groups.forEach((g, i) => g.holes.forEach((x) => { holeGroup[x] = i; }));
  const circuit = D.circuit, mapping = D.mapping;
  const col = circuit ? netColours(circuit) : {};
  const pn = circuit ? pinNets(circuit) : {};
  const groupNet = {};
  if (mapping) {
    for (const [pin, hole] of Object.entries({ ...mapping.pin_to_hole, ...(mapping.manual_connections || {}) })) groupNet[holeGroup[hole]] = pn[pin];
    mapping.jumpers.forEach((j) => Object.values(j.pins).forEach((x) => { groupNet[holeGroup[x]] ??= j.net; }));
  }
  const st = opt.state || {};
  const cls = (id) => (st.ghost?.has(id) ? ' ghost' : '') + (st.current === id ? ' current' : '') + (st.failed?.has(id) ? ' failed' : '');

  stat.board.conductive_groups.forEach((g, i) => {
    const shown = g.holes.filter(inView);
    if (!shown.length) return;
    const pts = shown.map(xy), xs = pts.map((p) => p[0]), ys = pts.map((p) => p[1]);
    const net = groupNet[i], pd = P * 0.42;
    sv('rect', {
      x: Math.min(...xs) - pd, y: Math.min(...ys) - pd, width: Math.max(...xs) - Math.min(...xs) + 2 * pd,
      height: Math.max(...ys) - Math.min(...ys) + 2 * pd, fill: net ? col[net] : 'rgba(17,17,17,.045)',
      'fill-opacity': net ? 0.2 : null, stroke: net ? col[net] : null, 'stroke-opacity': net ? 0.6 : null, 'data-nets': net,
    }, stripsG);
  });
  for (const hole of Object.keys(holeGroup).filter(inView)) {
    const [x, y] = xy(hole);
    sv('rect', { x: x - 2.4, y: y - 2.4, width: 4.8, height: 4.8, fill: '#111', 'fill-opacity': 0.82 }, holesG);
  }
  if (!mapping) return;

  const steps = D.assembly?.sequence || [];
  const items = Object.fromEntries(steps.map((s) => [s.item, s]));
  const order = Object.fromEntries(steps.map((s, k) => [s.item, k + 1]));
  const rect = (r, attrs, parent) => sv('rect', { x: X(r[0]), y: Y(r[3]), width: ((r[2] - r[0]) / PITCH) * P, height: ((r[3] - r[1]) / PITCH) * P, ...attrs }, parent);

  // casings: every item's printed block (the mapper's footprint)
  if (opt.casings) {
    for (const it of steps) {
      if (it.holes.some((x) => !inView(x))) continue;
      rect(it.body, { class: `cas${cls(it.item)}${st.over?.has(it.item) && !st.ghost?.has(it.item) ? ' over' : ''}`, 'data-focus': it.item }, casG);
    }
  }

  mapping.jumpers.forEach((j) => {
    const id = j.id, holes = Object.values(j.pins);
    if (!holes.every(inView)) return;
    const [[x1, y1], [x2, y2]] = holes.map(xy);
    const g = sv('g', { class: `item jumper${cls(id)}`, 'data-nets': j.net, 'data-focus': id }, wiresG);
    sv('line', { x1, y1, x2, y2, stroke: col[j.net], 'stroke-width': 5, 'stroke-linecap': 'square' }, g);
    for (const [x, y] of [[x1, y1], [x2, y2]]) sv('rect', { x: x - 2.2, y: y - 2.2, width: 4.4, height: 4.4, fill: '#d8d8d8', stroke: '#111', 'stroke-width': 0.7 }, g);
  });
  for (const c of circuit.components) {
    if (isManual(c.id)) drawManual(c, xy, inView, col, pn, partsG, labelsG);
    else drawPart(c, xy, inView, col, pn, partsG, labelsG, cls(c.id));
  }

  // the item going in now: its casing and the fingertips' swept pads
  if (st.current && items[st.current]) {
    const it = items[st.current];
    for (const f of it.fingers) rect(f, { class: 'finger' }, fingG);
    rect(it.body, { class: 'cas-now' }, fingG);
  }
  // order badges: where each item sits in the sequence
  if (opt.badges) {
    for (const it of steps) {
      if (!it.holes.every(inView) || st.ghost?.has(it.item)) continue;
      const cx = X((it.body[0] + it.body[2]) / 2), cy = Y((it.body[1] + it.body[3]) / 2);
      const g = sv('g', { class: `badge${st.current === it.item ? ' now' : ''}`, transform: `translate(${cx} ${cy})` }, badgeG);
      sv('rect', { x: -8, y: -7, width: 16, height: 14 }, g);
      sv('text', { x: 0, y: 0.5, 'text-anchor': 'middle', 'dominant-baseline': 'central' }, g).textContent = order[it.item];
    }
  }
}

const BAND = ['#1a1a1a', '#7b4a1e', '#d32f2f', '#f57c00', '#fbc02d', '#388e3c', '#1565c0', '#7b1fa2', '#808080', '#f2f2f2'];
function bands(v) {
  let e = Math.floor(Math.log10(v)) - 1, sig = Math.round(v / 10 ** e);
  if (sig >= 100) { sig = Math.round(sig / 10); e += 1; }
  return [BAND[Math.floor(sig / 10)], BAND[sig % 10], e >= 0 ? BAND[e] : '#c9a227', '#c9a227'];
}
const LED = { red: '#e0463a', green: '#3f9a4a', blue: '#2f74c8', yellow: '#e8c33a', white: '#f2f2f2' };

function drawBody(g, part, L) {
  const p = part.parameters || {}, half = L / 2;
  const capsule = (fill, stroke, w = 10) => sv('rect', { class: 'body', x: -half - 5, y: -w / 2, width: L + 10, height: w, rx: w / 2, fill, stroke }, g);
  switch (part.type) {
    case 'resistor':
      capsule('#d9c49b', '#8a7448');
      if (p.resistance_ohm) bands(p.resistance_ohm).forEach((c, i) => sv('rect', { x: -half + L * [0.08, 0.3, 0.52, 0.85][i] - 1.2, y: -5, width: 2.4, height: 10, fill: c }, g));
      break;
    case 'led': {
      const r = Math.min(9, half + 4), a = r * 0.72, b = Math.sqrt(r * r - a * a);
      sv('path', { class: 'body', d: `M ${a} ${-b} A ${r} ${r} 0 1 0 ${a} ${b} Z`, fill: LED[p.color] || '#ddd', stroke: 'rgba(0,0,0,.55)' }, g);
      sv('circle', { cx: -r * 0.35, cy: -r * 0.35, r: r * 0.26, fill: '#fff', 'fill-opacity': 0.5 }, g);
      break;
    }
    case 'rgb_led': sv('circle', { class: 'body', r: half + 6, fill: '#f4f4f4', stroke: '#777' }, g); break;
    case 'diode': capsule('#262626', '#000'); sv('rect', { x: half - 1, y: -5, width: 3, height: 10, fill: '#cfcfcf' }, g); break;
    case 'capacitor':
      if (p.polarized) sv('circle', { class: 'body', r: half + 4, fill: '#2a4d8f', stroke: '#16305e' }, g);
      else sv('circle', { class: 'body', r: half + 2, fill: '#e0a030', stroke: '#9c6c12' }, g);
      break;
    case 'bjt': sv('rect', { class: 'body', x: -half - 6, y: -7, width: L + 12, height: 12, fill: '#2b2b2b', stroke: '#000' }, g); break;
    case 'potentiometer':
      sv('rect', { class: 'body', x: -half - 6, y: -8, width: L + 12, height: 16, fill: '#3b6aa8', stroke: '#21406b' }, g);
      sv('circle', { r: 4.5, fill: '#e8e8e8', stroke: '#777' }, g);
      break;
    case 'switch':
      sv('rect', { class: 'body', x: -half - 6, y: -8, width: L + 12, height: 16, fill: '#5b5f66', stroke: '#2f3236' }, g);
      sv('circle', { r: 4, fill: '#1f2125' }, g);
      break;
    case 'connector': sv('rect', { class: 'body', x: -half - 7, y: -7, width: L + 14, height: 14, fill: '#1b1b1b', stroke: '#000' }, g); break;
    default: capsule('#9aa0a8', '#5f656d');
  }
}

// a hand-wired connection (the power connector): where the wire goes, not a part the robot places
const isManual = (id) => Object.keys(D.mapping?.manual_connections || {}).some((p) => p.startsWith(`${id}.`));
function drawManual(c, xy, inView, col, pn, partsG, labelsG) {
  const mc = D.mapping.manual_connections;
  const pins = partById[c.part_id].pins.filter((pin) => mc[`${c.id}.${pin}`] && inView(mc[`${c.id}.${pin}`]));
  if (!pins.length) return;
  const nets = [...new Set(pins.map((pin) => pn[`${c.id}.${pin}`]))].join(' ');
  const g = sv('g', { class: 'item manual', 'data-nets': nets, 'data-focus': c.id }, partsG);
  for (const pin of pins) {
    const [x, y] = xy(mc[`${c.id}.${pin}`]), net = pn[`${c.id}.${pin}`];
    sv('circle', { cx: x, cy: y, r: 6.5, fill: 'var(--cream)', stroke: col[net], 'stroke-width': 2, 'stroke-dasharray': '2.6 1.6' }, g);
    sv('text', { x, y: y + 0.5, 'text-anchor': 'middle', 'dominant-baseline': 'central', class: 'mark', fill: col[net] === '#111111' ? '#111' : col[net] }, g)
      .textContent = { positive: '+', negative: '−' }[pin] || pin[0];
  }
  const [x0, y0] = xy(mc[`${c.id}.${pins[0]}`]);
  sv('text', { x: x0, y: y0 - 14, 'text-anchor': 'middle', class: 'lbl' }, labelsG).textContent = c.id;
}

function drawPart(c, xy, inView, col, pn, partsG, labelsG, extra) {
  const part = partById[c.part_id];
  const holes = part.pins.map((pin) => D.mapping.pin_to_hole[`${c.id}.${pin}`]);
  if (holes.includes(undefined) || !holes.every(inView)) return;
  const pts = holes.map(xy), [x0, y0] = pts[0], [x1, y1] = pts[pts.length - 1];
  const cx = (x0 + x1) / 2, cy = (y0 + y1) / 2, L = Math.hypot(x1 - x0, y1 - y0);
  const nets = [...new Set(part.pins.map((pin) => pn[`${c.id}.${pin}`]))].join(' ');
  const g = sv('g', { class: `item comp${extra}`, 'data-nets': nets, 'data-focus': c.id }, partsG);
  sv('polyline', { points: pts.map((p) => p.join(',')).join(' '), fill: 'none', stroke: '#8c8577', 'stroke-width': 2.5 }, g);
  drawBody(sv('g', { transform: `translate(${cx} ${cy}) rotate(${(Math.atan2(y1 - y0, x1 - x0) * 180) / Math.PI})` }, g), part, L);
  part.pins.forEach((pin, k) => {
    const [x, y] = pts[k];
    if (part.type === 'connector') {
      sv('text', { x, y, 'text-anchor': 'middle', 'dominant-baseline': 'central', class: 'pinmark' }, g).textContent = { positive: '+', negative: '−' }[pin] || pin[0];
    } else sv('circle', { cx: x, cy: y, r: 2.3, fill: '#d8d8d8', stroke: '#111', 'stroke-width': 0.7 }, g);
  });
  const horiz = Math.abs(x1 - x0) >= Math.abs(y1 - y0);
  sv('text', horiz ? { x: cx, y: cy - 14, 'text-anchor': 'middle', class: `lbl${extra}` }
    : { x: cx + 13, y: cy, 'text-anchor': 'start', 'dominant-baseline': 'central', class: `lbl${extra}` }, labelsG).textContent = c.id;
}


// ── workspace: board, order, review ────────────────────────────────────────
function planState(k) {
  const a = D.assembly;
  if (!a) return {};
  const seq = a.sequence.map((s) => s.item);
  const n = seq.length;
  // casings in a bounding-box collision are outlined red: all of them, or the current step's
  if (k >= n) return { placed: new Set(seq), over: new Set(a.sequence.flatMap((s) => (s.collisions.length ? [s.item, ...s.collisions] : []))) };
  const c = a.sequence[k].collisions;
  return { placed: new Set(seq.slice(0, k)), current: seq[k], ghost: new Set(seq.slice(k + 1)), over: new Set(c.length ? [seq[k], ...c] : []) };
}

function renderWork() {
  if (!stat) return;
  const has = !!D.mapping;
  $('#ca-design').classList.toggle('has-layout', has);
  updateInput();
  if (!has) { renderActions(); return; }
  $('#ca-title').textContent = D.circuit.circuit_id.replace(/_/g, ' ');
  const m = D.mapping, nj = m.jumpers.length;
  $('#ca-sub').textContent = `${m.status === 'OPTIMAL' ? 'Optimal' : 'Feasible'} layout · ${D.circuit.components.length} parts · ${nj} jumper${nj === 1 ? '' : 's'}`;
  const k = D.assembly ? D.step : Infinity;
  drawBoard($('#ca-svg'), { view: viewMode, casings: showCasings && !!D.assembly, badges: !!D.assembly && k >= D.assembly.sequence.length, state: planState(k) });
  renderParts();
  renderFilm();
  renderCheck();
  renderActions();
}

function itemOf(id) { return D.assembly?.sequence.find((s) => s.item === id); }
// the mapper names jumpers jumper:<n>; people read W1, W2 in the order it lists them
function label(id) {
  const k = D.mapping?.jumpers.findIndex((j) => j.id === id) ?? -1;
  return k >= 0 ? `W${k + 1}` : id;
}
function describeItem(id) {
  const comp = D.circuit?.components.find((c) => c.id === id);
  const jumper = D.mapping?.jumpers.find((j) => j.id === id);
  return partLabel(partById[comp?.part_id || jumper?.part_id]);
}

// ── component list: one row per part kind, its icon drawn like the board draws it ──
const KIND_ORDER = ['connector', 'resistor', 'led', 'rgb_led', 'capacitor', 'diode', 'bjt', 'switch', 'potentiometer', 'jumper'];

function idRange(ids) {
  const m = ids.map((x) => x.match(/^([A-Z_]*?)(\d+)$/));
  if (ids.length > 2 && m.every((x) => x && x[1] === m[0][1])) {
    const ns = m.map((x) => +x[2]);
    if (ns.every((v, i) => !i || v === ns[i - 1] + 1)) return `${ids[0]}–${ids[ids.length - 1]}`;
  }
  return ids.join(', ');
}

function partIcon(part) {
  const s = document.createElementNS(SVGNS, 'svg');
  s.setAttribute('viewBox', '-25 -15 50 30');
  s.setAttribute('class', 'ca-icon');
  const L = Math.min(22, (part.pins.length - 1) * part.pin_spacing * 13);
  const xs = part.pins.map((_, k) => (part.pins.length > 1 ? -L / 2 + (L * k) / (part.pins.length - 1) : 0));
  sv('polyline', { points: xs.map((x) => `${x},0`).join(' '), fill: 'none', stroke: '#8c8577', 'stroke-width': 2.5 }, s);
  if (part.type === 'jumper') {
    sv('line', { x1: -L / 2, y1: 0, x2: L / 2, y2: 0, stroke: '#c9a15d', 'stroke-width': 5 }, s);
  } else drawBody(sv('g', {}, s), part, L);
  for (const x of xs) {
    if (part.type !== 'connector') sv('circle', { cx: x, cy: 0, r: 2.3, fill: '#d8d8d8', stroke: '#111', 'stroke-width': 0.7 }, s);
  }
  return s;
}

function renderParts() {
  const box = $('#ca-parts');
  if (!D.circuit || !D.mapping) { box.replaceChildren(); return; }
  const groups = new Map();
  const add = (pid, id) => { if (!groups.has(pid)) groups.set(pid, []); groups.get(pid).push(id); };
  for (const c of D.circuit.components) add(c.part_id, c.id);
  D.mapping.jumpers.forEach((j) => add(j.part_id, j.id));
  const kind = (pid) => { const i = KIND_ORDER.indexOf(partById[pid]?.type); return i < 0 ? 50 : i; };
  const rows = [...groups].sort((a, b) => kind(a[0]) - kind(b[0]));
  const now = planState(D.assembly ? D.step : Infinity).current;
  const total = rows.reduce((n, [, ids]) => n + ids.length, 0);
  box.replaceChildren(
    h('div', { class: 'ca-parts-head' }, h('b', {}, 'Components'), h('span', {}, String(total))),
    h('ul', {}, rows.map(([pid, ids]) => {
      const byHand = ids.every(isManual);
      const li = h('li', { class: `${ids.includes(now) ? 'now' : ''}${byHand ? ' manual' : ''}` }, partIcon(partById[pid]),
        h('div', {}, h('b', {}, partLabel(partById[pid])), h('span', {}, `${idRange(ids.map(label))}${byHand ? ' · wired by hand' : ''}`)),
        h('em', {}, `×${ids.length}`));
      li.onmouseenter = () => spotlight(ids);
      li.onmouseleave = () => spotlight(null);
      return li;
    })));
}

// dim everything on the board but these items
function spotlight(ids) {
  const keep = new Set(ids || []);
  for (const el of $('#ca-svg').querySelectorAll('[data-focus]')) el.classList.toggle('dim', !!ids && !keep.has(el.dataset.focus));
  for (const el of $('#ca-svg').querySelectorAll('.strips [data-nets], .lbl')) el.classList.toggle('dim', !!ids);
}

// the order as a strip of numbered tiles; the selected one is explained underneath
function renderFilm() {
  const film = $('#ca-film'), note = $('#ca-note');
  $('#ca-scrub').classList.toggle('hidden', !D.assembly);
  if (!D.assembly) { note.replaceChildren(h('span', { class: 'muted' }, 'Planning the insertion order…')); return; }
  const a = D.assembly, n = a.sequence.length;
  film.replaceChildren(...a.sequence.map((s, k) => {
    const b = h('button', { type: 'button', class: `${k === D.step ? 'on' : ''}${k < D.step ? ' past' : ''}${s.collisions.length ? ' bad' : ''}` },
      h('span', {}, pad2(k + 1)), h('b', {}, label(s.item)));
    b.onclick = () => { stopPlay(); D.step = D.step === k ? n : k; renderWork(); };
    return b;
  }));
  film.querySelector('.on')?.scrollIntoView({ block: 'nearest', inline: 'nearest' });
  $('#ca-play').textContent = playTimer ? 'Pause' : 'Play';
  if (D.step >= n) {
    note.replaceChildren(h('b', {}, `${n} steps`), h('span', {}, 'Numbers on the board give the insertion order. Pick a step or press Play.'));
    return;
  }
  const s = a.sequence[D.step];
  const why = s.why.replace(/jumper:\d+/g, label);
  note.replaceChildren(h('b', {}, `Step ${D.step + 1} · ${label(s.item)}`), h('span', {}, `${describeItem(s.item)}, ${s.holes.join('–')}. ${why}`),
    s.collisions.length ? h('span', { class: 'bad' }, `Collides with ${s.collisions.map(label).join(', ')}`) : null);
}

function renderCheck() {
  const box = $('#ca-check');
  box.classList.toggle('hidden', !D.assembly);
  if (!D.assembly) return;
  const a = D.assembly, nj = D.mapping.jumpers.length;
  const pairs = a.sequence.flatMap((s) => s.collisions.map((o) => `${label(s.item)}/${label(o)}`));
  const fig = (v, text, cls = '') => h('div', { class: cls }, h('b', {}, String(v)), h('span', {}, text));
  box.replaceChildren(
    h('div', { class: 'ca-figs' },
      fig(a.sequence.length, 'robot steps'), fig(nj, nj === 1 ? 'jumper' : 'jumpers'),
      fig(pairs.length, pairs.length === 1 ? 'collision' : 'collisions', pairs.length ? 'bad' : '')),
    h('p', {}, pairs.length ? `Bounding boxes collide: ${pairs.join(', ')}.`
      : `No bounding-box collisions: casings and ${a.gripper.finger_width_mm} × ${a.gripper.finger_thickness_mm} mm fingertips stay clear at every step.`));
}

function stopPlay() { if (playTimer) { clearInterval(playTimer); playTimer = null; } }
function play() {
  if (playTimer) { stopPlay(); renderWork(); return; }
  const n = D.assembly.sequence.length;
  if (D.step >= n) D.step = 0;
  playTimer = setInterval(() => {
    D.step += 1;
    if (D.step >= n) stopPlay();
    renderWork();
  }, 900);
  renderWork();
}

function renderActions() {
  const start = $('#ca-start');
  const acts = $('#ca-actions');
  acts.classList.toggle('hidden', !D.mapping);
  start.disabled = !((D.phase === 'review' && !D.running) || D.phase === 'handoff');
  start.textContent = D.phase === 'handoff' ? 'Open the build' : D.assembly ? 'Start assembly' : 'Planning…';
}

// hover: name the item, dim the nets it does not touch
function hover(svg, info) {
  svg.addEventListener('pointerover', (e) => {
    const t = e.target.closest('[data-focus]');
    const id = t?.dataset.focus;
    if (id === svg._hover) return;
    svg._hover = id;
    const nets = new Set((svg.querySelector(`.item[data-focus="${id}"]`)?.dataset.nets || '').split(' ').filter(Boolean));
    for (const el of svg.querySelectorAll('[data-nets]')) {
      const ns = (el.dataset.nets || '').split(' ').filter(Boolean);
      el.classList.toggle('dim', nets.size > 0 && !ns.some((n) => nets.has(n)));
    }
    if (!info) return;
    info.replaceChildren();
    if (!id || !D.mapping) return;
    const comp = D.circuit.components.find((c) => c.id === id);
    const where = { ...D.mapping.pin_to_hole, ...(D.mapping.manual_connections || {}) };
    const holes = comp ? partById[comp.part_id].pins.map((p) => `${p} ${where[`${id}.${p}`]}`)
      : Object.values(D.mapping.jumpers.find((j) => j.id === id)?.pins || {});
    info.append(h('b', {}, label(id)), h('span', {}, describeItem(id)), h('span', {}, holes.join(' · ')), h('span', {}, [...nets].join(' · ')));
  });
  svg.addEventListener('pointerleave', () => {
    svg._hover = null;
    for (const el of svg.querySelectorAll('.dim')) el.classList.remove('dim');
    if (info) info.replaceChildren();
  });
}

// ── assembly run (scaffolding: the pipeline reports to POST /api/circuit/<id>/run) ──
const RUN_LABEL = { waiting: 'Waiting for agent', running: 'Running', paused: 'Paused', done: 'Complete', failed: 'Failed', aborted: 'Aborted' };
const PH = ['pick', 'move', 'insert', 'verify'];

async function pollRun() {
  if (!D.id || D.phase !== 'handoff') return;
  const r = await fetch(`/api/circuit/${D.id}/run`).catch(() => null);
  if (!r || !r.ok) return;
  D.run = await r.json();
  const run = D.run, n = run.steps.length, done = run.steps.filter((s) => s.status === 'done').length;
  D.stages.build = {
    state: { waiting: 'wait', running: 'active', paused: 'wait', done: 'done', failed: 'failed', aborted: 'failed' }[run.status] || 'wait',
    detail: run.status === 'waiting' ? 'Waiting for the assembly agent' : `${done} of ${n} placed · ${RUN_LABEL[run.status] || run.status}`,
  };
  renderRail();
  if (D.view === 'run') renderRun();
}
setInterval(() => { if (!$('#tab-circuit').classList.contains('hidden')) pollRun(); }, 1000);

function runClock() {
  const el = $('#ca-rclock');
  if (!el || !D.run) return;
  const t0 = D.run.started || D.run.created;
  const end = ['done', 'failed', 'aborted'].includes(D.run.status) ? D.run.updated : Date.now() / 1000;
  el.textContent = clock(Math.max(0, end - t0));
}

function renderRun() {
  const run = D.run;
  if (!run) return;
  const n = run.steps.length;
  const done = run.steps.filter((s) => s.status === 'done').length;
  const retries = run.steps.reduce((a, s) => a + Math.max(0, (s.attempts || 0) - 1), 0);
  const cur = run.current != null ? run.steps[run.current] : null;
  const big = (k, v, cls = '') => h('div', { class: `ca-big ${cls}` }, h('span', { class: 'k' }, k), h('span', { class: 'v' }, v));
  $('#ca-runhead').replaceChildren(
    h('div', { class: 'ca-bigs' },
      big('Status', RUN_LABEL[run.status] || run.status, `st-${run.status}`),
      big('Placed', `${pad2(done)}/${pad2(n)}`),
      big('Now', cur ? label(cur.item) : '—'),
      big('Phase', cur?.phase || '—'),
      big('Retries', pad2(retries), retries ? 'warn' : ''),
      h('div', { class: 'ca-big' }, h('span', { class: 'k' }, 'Elapsed'), h('span', { class: 'v', id: 'ca-rclock' }, '00:00'))),
    h('div', { class: 'ca-segs' }, run.steps.map((s, k) => h('i', { class: s.status, title: `${pad2(k + 1)} ${label(s.item)} · ${s.status}` }))));
  runClock();

  $('#ca-runlist').replaceChildren(...run.steps.map((s, k) => {
    const it = itemOf(s.item);
    return h('li', { class: s.status },
      h('span', { class: 'k' }, pad2(k + 1)),
      h('span', { class: 'id' }, label(s.item)),
      h('span', { class: 'what' }, h('b', {}, describeItem(s.item)), h('em', {}, it ? it.holes.join(' – ') : ''), s.note ? h('em', { class: 'note' }, s.note) : null),
      h('span', { class: 'pips' }, PH.map((p) => {
        const at = PH.indexOf(s.phase);
        const state = s.status === 'done' ? 'done' : s.status === 'active' ? (PH.indexOf(p) < at ? 'done' : PH.indexOf(p) === at ? 'now' : '') : s.status === 'failed' && PH.indexOf(p) === at ? 'bad' : '';
        return h('i', { class: state, title: p }, p);
      })),
      h('span', { class: `tries${s.attempts > 1 ? ' warn' : ''}` }, s.attempts > 1 ? `${s.attempts} tries` : ''));
  }));
  $('#ca-runlist li.active')?.scrollIntoView({ block: 'nearest' });

  const placed = new Set(run.steps.filter((s) => s.status === 'done').map((s) => s.item));
  const failed = new Set(run.steps.filter((s) => s.status === 'failed').map((s) => s.item));
  const current = cur && cur.status === 'active' ? cur.item : null;
  const ghost = new Set(run.steps.filter((s) => !placed.has(s.item) && s.item !== current && !failed.has(s.item)).map((s) => s.item));
  drawBoard($('#ca-runsvg'), { view: 'fit', minRows: 24, casings: true, state: { placed, current, ghost, failed } });

  const lg = $('#ca-runlog');
  const t0 = run.created;
  lg.replaceChildren(...[...run.log].reverse().map((e) => h('div', { class: `e ${e.level || 'info'}` },
    h('span', {}, `+${clock(e.at - t0)}`), h('span', {}, e.item ? label(e.item) : ''), h('span', {}, e.text))));
  if (run.status === 'waiting') {
    lg.append(h('div', { class: 'ca-handoff', title: `data/circuit_designs/${D.id}/plan.json · progress: POST /api/circuit/${D.id}/run` },
      h('b', {}, 'Plan handed off'), h('span', {}, 'Waiting for the assembly agent to start')));
  }
  $('#ca-abort').disabled = ['done', 'failed', 'aborted'].includes(run.status);
}

function showView(v) {
  D.view = v;
  $('#ca-design').classList.toggle('hidden', v !== 'design');
  $('#ca-run').classList.toggle('hidden', v !== 'run');
  if (v === 'run') renderRun();
  else renderWork();
}

// ── wiring ─────────────────────────────────────────────────────────────────
$('#ca-form').onsubmit = (e) => {
  e.preventDefault();
  if (D.running) { if (D.id) fetch(`/api/circuit/${D.id}/stop`, { method: 'POST' }); return; }
  const t = $('#ca-text').value.trim();
  if (!t) return;
  $('#ca-text').value = '';
  sendText(t);
};
$('#ca-text').onkeydown = (e) => {
  if (e.key === 'Enter' && !e.shiftKey) { e.preventDefault(); $('#ca-form').requestSubmit(); }
};
$('#ca-restart').onclick = restart;
$('#tabs').addEventListener('click', (e) => { if (e.target.closest('button')?.dataset.tab === 'circuit') address(); });
$('#ca-start').onclick = async () => {
  if (D.phase === 'handoff') { await pollRun(); showView('run'); return; }
  const r = await fetch(`/api/circuit/${D.id}/start`, { method: 'POST' });
  if (!r.ok) { addError({ text: (await r.json().catch(() => ({}))).reason || `HTTP ${r.status}` }); return; }
  apply({ t: 'handoff', at: Date.now() / 1000 });
  updateInput();
  await pollRun();
  showView('run');
};
$('#ca-back').onclick = () => showView('design');
$('#ca-abort').onclick = async () => {
  const b = $('#ca-abort');
  if (!b.classList.contains('arm')) {      // second click within 3 s confirms
    b.classList.add('arm');
    b.textContent = 'Confirm abort';
    setTimeout(() => { b.classList.remove('arm'); b.textContent = 'Abort'; }, 3000);
    return;
  }
  b.classList.remove('arm');
  b.textContent = 'Abort';
  await fetch(`/api/circuit/${D.id}/run`, { method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ status: 'aborted', note: 'aborted from the dashboard', level: 'warn' }) });
  pollRun();
};
for (const b of document.querySelectorAll('[data-ca-view]')) {
  b.onclick = () => {
    viewMode = b.dataset.caView;
    document.querySelectorAll('[data-ca-view]').forEach((x) => x.classList.toggle('on', x === b));
    renderWork();
  };
}
$('#ca-cas').onclick = () => { showCasings = !showCasings; $('#ca-cas').classList.toggle('on', showCasings); renderWork(); };
$('#ca-prev').onclick = () => { stopPlay(); D.step = Math.max(0, Math.min(D.step, D.assembly.sequence.length) - 1); renderWork(); };
$('#ca-next').onclick = () => { stopPlay(); D.step = Math.min(D.assembly.sequence.length, D.step + 1); renderWork(); };
$('#ca-play').onclick = play;
hover($('#ca-svg'), $('#ca-info'));
hover($('#ca-runsvg'), null);

(async () => {
  stat = await (await fetch('/api/circuit/static')).json();
  partById = Object.fromEntries(stat.parts.map((p) => [p.part_id, p]));
  renderRail();
  renderWork();
  updateInput();
  let id = null;
  try { id = localStorage.getItem('circuitDesign'); } catch { /* storage blocked */ }
  const m = decodeURIComponent(location.hash).match(/^#circuit:(.+)$/);
  if (m) id = m[1];
  // nothing remembered (or it is gone): open the most recent design; "New design" starts a fresh one
  if (!id || !(await attach(id))) {
    const latest = (await (await fetch('/api/circuit/latest')).json()).id;
    if (latest) await attach(latest);
  }
  if (!log().children.length) intro();
})();
