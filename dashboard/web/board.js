// ── board drawing ───────────────────────────────────────────────────────────
// The breadboard, its parts and the robot's casings, drawn from a design's circuit, mapping and
// assembly plan (dashboard/circuit_design.py, assembly_plan.py). Shared by the Circuit Assembly
// tab (circuit.js) and the customer order page (order/order.js); no globals: every drawing
// takes a model {board, partById, circuit, mapping, assembly}.

export const SVGNS = 'http://www.w3.org/2000/svg';
export const PITCH = 2.54;

export function h(tag, attrs, ...kids) {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs || {})) {
    if (v == null || v === false) continue;
    if (k.startsWith('on')) e[k] = v;
    else e.setAttribute(k, v === true ? '' : v);
  }
  for (const k of kids.flat()) if (k != null && k !== false) e.append(k);
  return e;
}
export function sv(tag, attrs, parent) {
  const e = document.createElementNS(SVGNS, tag);
  for (const [k, v] of Object.entries(attrs || {})) if (v != null) e.setAttribute(k, v);
  if (parent) parent.appendChild(e);
  return e;
}

// ── names and colours ──────────────────────────────────────────────────────
export const ohm = (v) => (v >= 1e6 ? `${v / 1e6} MΩ` : v >= 1e3 ? `${v / 1e3} kΩ` : `${v} Ω`);
export function partLabel(p) {
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
export const isSupply = (net) => /^(VCC|VDD|VIN|V\+|\+?\d+V\d*)$/i.test(net);
export const isGround = (net) => /^(GND|VSS|0V)$/i.test(net);
export function netColours(circuit) {
  const col = {};
  let k = 0;
  for (const n of circuit.nets) {
    if (isSupply(n.id)) col[n.id] = '#d2483c';
    else if (isGround(n.id)) col[n.id] = '#111111';
    else col[n.id] = NET_PALETTE[k++ % NET_PALETTE.length];
  }
  for (const p of circuit.unconnected_pins || []) col[`UNCONNECTED:${p}`] = '#8c8577';
  return col;
}
export function pinNets(circuit) {
  const m = {};
  for (const n of circuit.nets) for (const p of n.pins) m[p] = n.id;
  for (const p of circuit.unconnected_pins || []) m[p] = `UNCONNECTED:${p}`;
  return m;
}

// ── board drawing ──────────────────────────────────────────────────────────
// rows run left to right, column J on top; finger pads may hang over the board edge
const P = 18, MX = 34, MY = 2 * P;     // the svg overflows: finger pads may hang past the board

function boardModel(board) {
  const G = board.geometry, R = G.rows, colPos = G.column_positions;
  const cols = Object.values(colPos);
  return { G, R, colPos, maxCol: Math.max(...cols), minCol: Math.min(...cols), rowPos: (r) => R.first_position + (r - R.first) * R.spacing };
}

export function usedRows(mapping) {
  if (!mapping) return [];
  const holes = [...Object.values(mapping.pin_to_hole), ...Object.values(mapping.manual_connections || {}),
    ...mapping.jumpers.flatMap((j) => Object.values(j.pins))];
  return holes.map((x) => parseInt(x.slice(1), 10));
}

// a hand-wired connection (the power connector): where the wire goes, not a part the robot places
export const isManual = (mapping, id) => Object.keys(mapping?.manual_connections || {}).some((p) => p.startsWith(`${id}.`));

export function drawBoard(svg, model, opt) {
  const bm = boardModel(model.board);
  const { R, colPos, maxCol, minCol, rowPos } = bm;
  let lo = R.first, hi = R.last;
  const rows = usedRows(model.mapping);
  if (opt.rows) {             // [first, last] row to show (the customer page's crops)
    lo = Math.max(R.first, opt.rows[0]);
    hi = Math.min(R.last, opt.rows[1]);
  } else if (opt.view === 'fit' && rows.length) {
    const a = Math.min(...rows), b = Math.max(...rows), want = Math.max(opt.minRows || 20, b - a + (opt.pad ?? 9));
    lo = Math.max(R.first, Math.min(Math.round((a + b - want) / 2), R.last - want + 1));
    hi = Math.min(R.last, lo + want - 1);
  } else if (opt.view === 'fit') { hi = Math.min(R.last, R.first + 29); }
  // power (customer page): the row numbers sit higher, so a label above a part in row J clears them
  const lift = opt.power ? 14 : 0, my = MY + lift;
  const X = (u) => MX + (u / PITCH - rowPos(lo)) * P;            // u, v in mm (assembly_plan.py)
  const Y = (v) => my + (maxCol - v / PITCH) * P;
  const rowOf = (hole) => parseInt(hole.slice(1), 10);
  const inView = (hole) => rowOf(hole) >= lo && rowOf(hole) <= hi;
  const xy = (hole) => [MX + (rowPos(rowOf(hole)) - rowPos(lo)) * P, my + (maxCol - colPos[hole[0]]) * P];
  const W = 2 * MX + (rowPos(hi) - rowPos(lo)) * P, H = 2 * my + (maxCol - minCol) * P;
  svg.replaceChildren();
  svg.setAttribute('viewBox', `0 0 ${W} ${H}`);
  svg.classList.add('bd');

  sv('rect', { x: MX - 26, y: my - 30 - lift, width: W - 2 * MX + 52, height: H - 2 * my + 60 + 2 * lift, fill: 'var(--cream, #fef5da)', class: 'bb' }, svg);
  const upper = Object.values(colPos).filter((c) => c > 4);
  if (upper.length) {
    const gapTop = my + (maxCol - Math.min(...upper)) * P;
    sv('rect', { x: MX - 10, y: gapTop + P * 0.8, width: W - 2 * MX + 20, height: P * 1.4, fill: 'var(--beige, #e6dec6)' }, svg);
  }
  for (const [letter, c] of Object.entries(colPos)) {
    const y = my + (maxCol - c) * P;
    for (const x of [MX - 17, W - MX + 17]) sv('text', { x, y, class: 'ax', 'text-anchor': 'middle', 'dominant-baseline': 'central' }, svg).textContent = letter;
  }
  const step = hi - lo < 40 ? 1 : 5;
  for (let r = lo; r <= hi; r++) {
    if (r !== lo && r % step) continue;
    const x = MX + (rowPos(r) - rowPos(lo)) * P;
    sv('text', { x, y: my - 17 - lift, class: 'ax', 'text-anchor': 'middle' }, svg).textContent = r;
    if (!opt.power) sv('text', { x, y: H - my + 24, class: 'ax', 'text-anchor': 'middle' }, svg).textContent = r;     // power: labels may go there
  }

  const L = (name) => sv('g', { class: name }, svg);
  const stripsG = L('strips'), holesG = L('holes'), casG = L('casings'), wiresG = L('wires'), partsG = L('parts'),
    fingG = L('fingers'), labelsG = L('labels'), badgeG = L('badges'), powerG = L('power');

  if (opt.empty && upper.length) {
    const gy = my + (maxCol - Math.min(...upper)) * P + P * 1.5;
    sv('text', { x: W / 2, y: gy, class: 'empty', 'text-anchor': 'middle', 'dominant-baseline': 'central' }, svg).textContent = opt.empty;
  }
  const holeGroup = {};
  model.board.conductive_groups.forEach((g, i) => g.holes.forEach((x) => { holeGroup[x] = i; }));
  const { circuit, mapping } = model;
  const col = circuit ? netColours(circuit) : {};
  const pn = circuit ? pinNets(circuit) : {};
  const groupNet = {};
  if (mapping) {
    for (const [pin, hole] of Object.entries({ ...mapping.pin_to_hole, ...(mapping.manual_connections || {}) })) groupNet[holeGroup[hole]] = pn[pin];
    mapping.jumpers.forEach((j) => Object.values(j.pins).forEach((x) => { groupNet[holeGroup[x]] ??= j.net; }));
  }
  const st = opt.state || {};
  const cls = (id) => (st.ghost?.has(id) ? ' ghost' : '') + (st.current === id ? ' current' : '') + (st.failed?.has(id) ? ' failed' : '');

  model.board.conductive_groups.forEach((g, i) => {
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

  const steps = model.assembly?.sequence || [];
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
    if (j.hand) {          // a wire the operator plugs in after the robot: a loose arc, not a rigid jumper
      const g = sv('g', { class: `item wire hand${cls(id)}`, 'data-focus': id }, wiresG);
      const mx = (x1 + x2) / 2, my = (y1 + y2) / 2, len = Math.hypot(x2 - x1, y2 - y1) || 1;
      const bow = Math.min(28, 8 + len * 0.25), nx = -(y2 - y1) / len, ny = (x2 - x1) / len;
      sv('path', { d: `M ${x1} ${y1} Q ${mx + nx * bow} ${my + ny * bow} ${x2} ${y2}`, fill: 'none', stroke: '#e9b65a',
        'stroke-width': 2.4, 'stroke-dasharray': '5 3', 'stroke-linecap': 'round' }, g);
      for (const [x, y] of [[x1, y1], [x2, y2]]) sv('circle', { cx: x, cy: y, r: 3, fill: '#e9b65a', stroke: '#111', 'stroke-width': 0.7 }, g);
      return;
    }
    const g = sv('g', { class: `item jumper${cls(id)}`, 'data-nets': j.net, 'data-focus': id }, wiresG);
    sv('line', { x1, y1, x2, y2, stroke: col[j.net], 'stroke-width': 5, 'stroke-linecap': 'square' }, g);
    for (const [x, y] of [[x1, y1], [x2, y2]]) sv('rect', { x: x - 2.2, y: y - 2.2, width: 4.4, height: 4.4, fill: '#d8d8d8', stroke: '#111', 'stroke-width': 0.7 }, g);
  });
  // power callouts are big: part labels move to the other side of their part instead of under one
  const avoid = opt.power ? Object.entries(mapping.manual_connections || {}).filter(([p, x]) => p.startsWith('J_POWER.') && inView(x)).map(([, x]) => xy(x)) : [];
  const ctx = { model, xy, inView, col, pn, partsG, labelsG, powerG, avoid, labels: opt.labels };
  for (const c of circuit.components) {
    if (isManual(mapping, c.id)) drawManual(c, ctx, opt.power);
    else drawPart(c, ctx, cls(c.id));
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
export const LED = { red: '#e0463a', green: '#3f9a4a', blue: '#2f74c8', yellow: '#e8c33a', white: '#f2f2f2' };

export function drawBody(g, part, L) {
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

// what marks a hand-wired hole on the customer's picture (power: + and −; "How to connect it"
// names the holes with the same marks)
export const wireMark = (id) => (id === 'J_POWER' ? null : /SW/.test(id) ? 'S' : 'W');

// opt.power (customer page): big + and − callouts where the supply wires go; the other wires the
// customer connects (a switch, a signal) are marked S or W, never + or −, so they don't read as
// a second supply
function drawManual(c, { model, xy, inView, col, pn, partsG, labelsG, powerG, labels }, power) {
  const mc = model.mapping.manual_connections;
  const pins = model.partById[c.part_id].pins.filter((pin) => mc[`${c.id}.${pin}`] && inView(mc[`${c.id}.${pin}`]));
  if (!pins.length) return;
  const nets = [...new Set(pins.map((pin) => pn[`${c.id}.${pin}`]))].join(' ');
  const big = power && c.id === 'J_POWER';         // drawn on top of the parts and their labels
  const g = sv('g', { class: 'item manual', 'data-nets': nets, 'data-focus': c.id }, big ? powerG : partsG);
  for (const pin of pins) {
    const [x, y] = xy(mc[`${c.id}.${pin}`]), net = pn[`${c.id}.${pin}`];
    if (big) {
      const plus = pin === 'positive';
      sv('circle', { cx: x, cy: y, r: 8, fill: plus ? '#d2483c' : '#111', stroke: '#fff', 'stroke-width': 1.5, class: 'pwr' }, g);
      sv('text', { x, y: y + 0.5, 'text-anchor': 'middle', 'dominant-baseline': 'central', class: 'pwrmark' }, g).textContent = plus ? '+' : '−';
      continue;
    }
    sv('circle', { cx: x, cy: y, r: 6.5, fill: 'var(--cream, #fef5da)', stroke: col[net], 'stroke-width': 2, 'stroke-dasharray': '2.6 1.6' }, g);
    sv('text', { x, y: y + 0.5, 'text-anchor': 'middle', 'dominant-baseline': 'central', class: 'mark', fill: col[net] === '#111111' ? '#111' : col[net] }, g)
      .textContent = power ? wireMark(c.id) : { positive: '+', negative: '−' }[pin] || pin[0];
  }
  if (power) sv('title', {}, g).textContent = labels?.[c.id] ?? c.id;    // the page's "How to connect it" names the marks
  const [x0, y0] = xy(mc[`${c.id}.${pins[0]}`]);
  if (!big && !power) sv('text', { x: x0, y: y0 - 14, 'text-anchor': 'middle', class: 'lbl' }, labelsG).textContent = labels?.[c.id] ?? c.id;
}

function drawPart(c, { model, xy, inView, pn, partsG, labelsG, avoid, labels }, extra) {
  const part = model.partById[c.part_id];
  const holes = part.pins.map((pin) => model.mapping.pin_to_hole[`${c.id}.${pin}`]);
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
  const near = (x, y, dx) => (avoid || []).some(([ax, ay]) => Math.abs(ax - x) < dx && Math.abs(ay - y) < 12);
  const flip = horiz ? near(cx, cy - 14, 40) : near(cx + 30, cy, 30);
  sv('text', horiz ? { x: cx, y: flip ? cy + 22 : cy - 14, 'text-anchor': 'middle', class: `lbl${extra}` }
    : { x: flip ? cx - 13 : cx + 13, y: cy, 'text-anchor': flip ? 'end' : 'start', 'dominant-baseline': 'central', class: `lbl${extra}` },
  labelsG).textContent = labels?.[c.id] ?? c.id;
}

// ── a part drawn on its own, as the board draws it ─────────────────────────
export function partIcon(part, cls = 'ca-icon') {
  const s = document.createElementNS(SVGNS, 'svg');
  s.setAttribute('viewBox', '-25 -15 50 30');
  s.setAttribute('class', cls);
  s.setAttribute('aria-hidden', 'true');
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

// ── plain words and a simple circuit picture (the customer page) ────────────
// what a maker calls each part; hand-wired connectors by their job (J_POWER, J_SW, J_IN, J_OUT)
export function plainName(c, part) {
  if (part.type === 'connector') {
    if (c.id === 'J_POWER') return 'power wires';
    return /SW/.test(c.id) ? 'switch you wire in' : /IN/.test(c.id) ? 'signal-in wires' : /OUT/.test(c.id) ? 'signal-out wires' : 'wires you connect';
  }
  const q = part.parameters || {};
  switch (part.type) {
    case 'led': return `${q.color} light`;
    case 'rgb_led': return 'colour-mixing light';
    case 'capacitor': return 'capacitor';
    default: return partLabel(part);
  }
}

// the circuit as one path from + to − through two-legged parts, top to bottom: [{c, part, up, down}]
// (up/down: the pin on the side nearer + / −), or null when it branches
export function chain(circuit, partById) {
  const pn = pinNets(circuit);
  const ends = (side) => (circuit.components.some((c) => c.id === 'J_POWER') ? pn[`J_POWER.${side}`]
    : circuit.nets.find((n) => (side === 'positive' ? isSupply : isGround)(n.id))?.id);
  const top = ends('positive'), bottom = ends('negative');
  if (!top || !bottom) return null;
  const legs = circuit.components.filter((c) => c.id !== 'J_POWER').map((c) => ({
    c, part: partById[c.part_id], pins: partById[c.part_id].pins.filter((p) => pn[`${c.id}.${p}`] && !pn[`${c.id}.${p}`].startsWith('UNCONNECTED')),
  }));
  if (!legs.length || legs.some((x) => x.pins.length !== 2)) return null;
  const out = [], used = new Set();
  for (let net = top; net !== bottom;) {
    const next = legs.filter((x) => !used.has(x.c.id) && x.pins.some((p) => pn[`${x.c.id}.${p}`] === net));
    if (next.length !== 1 || out.length > legs.length) return null;
    const x = next[0], [up, down] = pn[`${x.c.id}.${x.pins[0]}`] === net ? x.pins : [x.pins[1], x.pins[0]];
    used.add(x.c.id);
    out.push({ c: x.c, part: x.part, up, down });
    net = pn[`${x.c.id}.${down}`];
  }
  return used.size === legs.length ? out : null;
}

// "The 1 kΩ resistor connects + (power) and the red light." for every part: the text version of
// both pictures (screen readers, colour-blind visitors, and anyone who prefers words)
export function sentences(circuit, partById) {
  const pn = pinNets(circuit);
  const power = circuit.components.some((c) => c.id === 'J_POWER');
  const top = power ? pn['J_POWER.positive'] : null, bottom = power ? pn['J_POWER.negative'] : null;
  const name = (c) => plainName(c, partById[c.part_id]);
  const where = (net, self) => {
    if (net === top || (!power && isSupply(net))) return '+ (power)';
    if (net === bottom || (!power && isGround(net))) return '− (ground)';
    const others = (circuit.nets.find((n) => n.id === net)?.pins || []).map((p) => p.split('.')[0]).filter((id) => id !== self);
    const names = [...new Set(others)].map((id) => `the ${name(circuit.components.find((c) => c.id === id))}`);
    return names.join(' and ') || 'nothing';
  };
  return circuit.components.filter((c) => c.id !== 'J_POWER').map((c) => {
    const nets = partById[c.part_id].pins.map((p) => pn[`${c.id}.${p}`]).filter((n) => n && !n.startsWith('UNCONNECTED'));
    const to = [...new Set(nets.map((n) => where(n, c.id)))];
    const n = name(c), verb = /wires$/.test(n) ? 'connect' : 'connects';
    return `The ${n} ${verb} ${to.length === 2 ? `${to[0]} and ${to[1]}` : to.join(', ')}.`;
  });
}

// + at the top, − at the bottom, every part on the wire between them, labelled in plain words.
// Returns false (and draws nothing) when the circuit is not one path: the sentences stand alone.
export function drawSchematic(svg, circuit, partById) {
  const path = chain(circuit, partById);
  svg.replaceChildren();
  if (!path) return false;
  const STEP = 74, X = 70, top = 34, W = 300, H = top + STEP * path.length + 46;
  svg.setAttribute('viewBox', `0 0 ${W} ${H}`);
  svg.classList.add('sch');
  const term = (y, plus) => {
    sv('circle', { cx: X, cy: y, r: 10, fill: plus ? '#d2483c' : '#111' }, svg);
    sv('text', { x: X, y: y + 0.5, class: 'sym-pm', 'text-anchor': 'middle', 'dominant-baseline': 'central' }, svg).textContent = plus ? '+' : '−';
    sv('text', { x: X + 22, y, class: 'sym-lbl', 'dominant-baseline': 'central' }, svg).textContent = plus ? '+5 V (power in)' : '− (ground)';
  };
  sv('line', { x1: X, y1: top, x2: X, y2: top + STEP * path.length + 12, class: 'sym-wire' }, svg);
  term(top, true);
  path.forEach(({ c, part, up }, k) => {
    const y = top + STEP * (k + 0.5) + 6;
    const g = sv('g', { transform: `translate(${X} ${y})` }, svg);
    const q = part.parameters || {};
    switch (part.type) {
      case 'resistor': sv('rect', { x: -8, y: -18, width: 16, height: 36, class: 'sym-body' }, g); break;
      case 'led': {
        const down = up === 'anode';        // the triangle points the way the current flows
        sv('line', { x1: -14, y1: down ? 10 : -10, x2: 14, y2: down ? 10 : -10, class: 'sym-line' }, g);
        sv('path', { d: down ? 'M -14 -12 L 14 -12 L 0 10 Z' : 'M -14 12 L 14 12 L 0 -10 Z', fill: LED[q.color] || '#ddd', class: 'sym-line' }, g);
        for (const dy of [-8, 2]) sv('path', { d: `M 18 ${dy} l 9 -7 m -4 0 l 4 0 l 0 4`, class: 'sym-ray' }, g);
        break;
      }
      case 'capacitor':
        sv('rect', { x: -16, y: -5, width: 32, height: 10, fill: 'var(--paper, #fff)' }, g);     // cuts the wire between the plates
        for (const dy of [-5, 5]) sv('line', { x1: -16, y1: dy, x2: 16, y2: dy, class: 'sym-plate' }, g);
        break;
      case 'connector':       // a switch or a pair of wires the customer connects: an open gap
        sv('rect', { x: -12, y: -12, width: 24, height: 24, fill: 'var(--paper, #fff)' }, g);
        sv('circle', { cx: 0, cy: -12, r: 3.5, class: 'sym-body' }, g);
        sv('circle', { cx: 0, cy: 12, r: 3.5, class: 'sym-body' }, g);
        if (/SW/.test(c.id)) sv('line', { x1: 0, y1: 9, x2: 13, y2: -10, class: 'sym-line' }, g);
        break;
      default: sv('rect', { x: -12, y: -14, width: 24, height: 28, class: 'sym-body' }, g);
    }
    sv('text', { x: 36, y: 0, class: 'sym-lbl', 'dominant-baseline': 'central' }, g).textContent = plainName(c, part);
  });
  term(top + STEP * path.length + 12, false);
  return true;
}
