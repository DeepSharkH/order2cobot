// The casing rack seen from above, as the person refilling it sees it: the front (toward the
// breadboard and the workplace) at the bottom, the back (toward the switch) on top; ↻ turns it so
// the hole columns match the real rack. A slot per leg-hole pair with the part standing in it; a tap on a slot picks what stands
// there (only parts whose leg spacing fits that column). Used by the 3D Environment's Rack card and
// by a project's shopping list (with `need`: the parts its layout still lacks, offered first).
// State: GET/POST /api/rack (data/rack_state.json, shared with rack-pick and the build).
// Geometry: Jan's latest holder (assets/models/component_casings/All_Casings_Holder_Print.stl,
// 10-27 15:33), measured from its through-holes: 142.16 x 82.5 mm, 4 columns x 5 rows; column c
// (0..3) holds legs (c + 1) x 2.54 mm apart, centre x 20.0 + 34.04 c; row r at y 10.62 + 15.3 r.
// Slot k (1..20): c = (k-1) % 4, r = (k-1) // 4.

const RACK = { w: 142.16, d: 82.5, x0: 20.0, dx: 34.04, y0: 10.62, dy: 15.3, cols: 4, pitch: 2.54 };
export const slotSpacing = (slot) => ((Number(slot.slice(1)) - 1) % RACK.cols) + 1;
const LED_COL = { red: '#e0463a', green: '#3f9a4a', yellow: '#e8c33a', blue: '#2f74c8' };
const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

// body colour, accent and a short tag per part (an inventory option: part_id, name, type, casing)
export function partLook(o) {
  if (!o) return { body: '#bbb', tag: '?' };
  const colour = o.name.match(/red|green|yellow|blue/i)?.[0].toLowerCase();
  switch (o.type) {
    case 'led': return { body: '#f3ead2', dot: LED_COL[colour] || '#ddd', tag: colour || 'LED' };
    case 'resistor': return { body: '#d8c49a', bands: true, tag: (o.name.match(/[\d.]+\s*k?\s*ohm/i)?.[0] || 'R').replace(/\s*ohm/i, 'Ω').replace(/\s+/g, '') };
    case 'capacitor': return { body: '#d9822b', tag: o.name.match(/\d+\s*[nuµp]F/i)?.[0].replace(/\s+/g, '') || 'C' };
    case 'jumper': return { body: '#c9a15d', tag: `J${o.casing.match(/\d+$/)?.[0] || ''}` };
    default: return { body: '#bbb', tag: o.type };
  }
}
export function swatch(o) {
  const l = partLook(o);
  return `<i class="rk-sw" style="background:${l.dot || l.body}"></i>`;
}

export class RackView {
  constructor(host, { onChange, turnButton } = {}) {
    this.host = host;
    this.onChange = onChange;
    this.need = [];
    this.data = null;
    try { this.turned = localStorage.getItem('rackTurned') === '1'; } catch { this.turned = false; }
    host.classList.add('rk');
    host.addEventListener('click', (e) => this.click(e));
    turnButton?.addEventListener('click', () => {
      this.turned = !this.turned;
      try { localStorage.setItem('rackTurned', this.turned ? '1' : '0'); } catch { /* storage blocked */ }
      this.render();
    });
    document.addEventListener('click', (e) => { if (!host.contains(e.target)) this.close(); });
  }

  async load() {
    const r = await fetch('/api/rack').then((x) => x.json()).catch(() => null);
    if (r) { this.data = r; this.render(); }
    return r;
  }

  get picking() { return !this.host.querySelector('.rk-pick')?.classList.contains('hidden'); }

  setNeed(need) { this.need = need || []; if (this.data) this.render(); }

  render() {
    const r = this.data;
    if (!r) return;
    const M = 6, W = RACK.w + 2 * M, H = RACK.d + 2 * M + 18, top = M + 9;
    // from the front (the person refilling it): the STL's y = 0 edge is the back (rack.glb is the STL
    // turned 180 deg about x, and data/cad/rack_scene.json has its +y toward the switch), its +x to the right
    const view = ([x, y]) => (this.turned ? [M + RACK.w - x, top + RACK.d - y] : [M + x, top + y]);
    const opt = Object.fromEntries(r.options.map((o) => [o.part_id, o]));
    const needed = new Set(this.need.map((n) => n.part_id));
    const wantCols = new Set(this.need.map((n) => opt[n.part_id]?.pin_spacing).filter(Boolean));     // columns a missing part fits
    let svg = `<svg viewBox="0 0 ${W} ${H}" class="rk-svg">
      <text x="${W / 2}" y="7" class="side">BACK</text>
      <text x="${W / 2}" y="${H - 1.5}" class="side front">FRONT</text>
      <rect x="${M}" y="${top}" width="${RACK.w}" height="${RACK.d}" rx="3" class="plate"/>`;
    // column heads: the leg spacing each column takes
    for (let c = 0; c < RACK.cols; c++) {
      const [cx] = view([RACK.x0 + c * RACK.dx, 0]);
      svg += `<text x="${cx}" y="${top - 1.5}" class="col">${c + 1}×</text>`;
    }
    const sw = 22, sh = 4;      // a casing seen from above: about its width x its 4 mm thickness
    for (const s of r.slots) {
      const k = Number(s.slice(1)), c = (k - 1) % RACK.cols, half = ((c + 1) * RACK.pitch) / 2;
      const [cx, cy] = view([RACK.x0 + c * RACK.dx, RACK.y0 + Math.floor((k - 1) / RACK.cols) * RACK.dy]);
      const it = r.contents[s], o = it && opt[it.part_id], look = it && partLook(o);
      const misfit = o && o.pin_spacing !== c + 1;       // entered before the columns were checked
      svg += `<g class="slot${it ? ' full' : ''}${it && needed.has(it.part_id) ? ' needed' : ''}${misfit ? ' misfit' : ''}${!it && wantCols.has(c + 1) ? ' wanted' : ''}" data-slot="${s}" transform="translate(${cx} ${cy})">`;
      if (misfit) svg += `<title>${esc(o.name)} has legs ${o.pin_spacing}×2.54 mm apart: it does not fit this column</title>`;
      svg += `
        <rect x="${-sw / 2 - 3}" y="${-sh / 2 - 3}" width="${sw + 6}" height="${sh + 10}" class="hit"/>
        <rect x="${-sw / 2}" y="${-sh / 2}" width="${sw}" height="${sh}" rx="1" class="pad"/>
        <circle cx="${-half}" r=".7" class="hole"/><circle cx="${half}" r=".7" class="hole"/>`;
      if (look) {
        svg += `<rect x="${-sw / 2 + 0.6}" y="${-sh / 2 + 0.3}" width="${sw - 1.2}" height="${sh - 0.6}" rx=".9" fill="${look.body}" class="body"/>`;
        if (look.bands) ['#7b4a1e', '#1a1a1a', '#d32f2f', '#c9a15d'].forEach((col, i) => { svg += `<rect x="${-6 + i * 3.4}" y="${-sh / 2 + 0.3}" width="1.4" height="${sh - 0.6}" fill="${col}"/>`; });
        if (look.dot) svg += `<circle r="1.8" fill="${look.dot}" stroke="#111" stroke-width=".3"/>`;
      }
      svg += `<text y="${sh / 2 + 4.6}" class="lbl">${s}${look ? ` · ${esc(look.tag)}` : ''}</text></g>`;
    }
    this.host.innerHTML = `${svg}</svg><div class="rk-pick hidden"></div>`;
  }

  // the chooser: the parts still needed first, then every part the robot can pick, and empty
  open(slot, at) {
    const r = this.data, cur = r.contents[slot]?.part_id, box = this.host.querySelector('.rk-pick');
    const fits = (o) => o.pin_spacing === slotSpacing(slot);      // each column is drilled for one leg spacing
    const needed = this.need.map((n) => r.options.find((o) => o.part_id === n.part_id)).filter((o) => o && fits(o));
    const groups = needed.length ? { needed } : {};
    for (const o of r.options.filter(fits)) (groups[o.type] ||= []).push(o);
    const btn = (o) => `<button data-part="${esc(o.part_id)}" class="${o.part_id === cur ? 'on' : ''}" title="${esc(o.name)} · ${esc(o.casing)}">${swatch(o)}${esc(partLook(o).tag)}</button>`;
    box.innerHTML = `<div class="rp-head">${slot} · legs ${slotSpacing(slot)}×2.54 mm<button data-part="" class="${cur ? '' : 'on'}">empty</button></div>`
      + Object.entries(groups).map(([t, os]) => `<div class="rp-group${t === 'needed' ? ' need' : ''}"><span>${t === 'needed' ? 'missing for this project' : esc(t)}</span>${os.map(btn).join('')}</div>`).join('');
    box.classList.remove('hidden');
    const hb = this.host.getBoundingClientRect();
    box.style.left = `${Math.max(0, Math.min(at.x - hb.left - 125, hb.width - 250))}px`;
    box.style.top = `${at.y - hb.top + 14}px`;
    box.dataset.slot = slot;
  }

  close() { this.host.querySelector('.rk-pick')?.classList.add('hidden'); }

  async click(e) {
    const b = e.target.closest('.rk-pick button[data-part]');
    if (b) {
      const slot = this.host.querySelector('.rk-pick').dataset.slot;
      this.close();
      const res = await fetch('/api/rack', { method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ slot, part_id: b.dataset.part || null }) }).catch(() => null);
      if (res?.ok) { this.data = await res.json(); this.render(); } else await this.load();
      this.onChange?.(this.data);
      return;
    }
    const g = e.target.closest('.slot');
    if (g) { e.stopPropagation(); this.open(g.dataset.slot, { x: e.clientX, y: e.clientY }); }
  }
}
