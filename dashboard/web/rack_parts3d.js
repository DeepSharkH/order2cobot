// The bare parts standing on the rack, for the 3D scene: what the Rack card says each slot holds,
// drawn without its casing (the casing would hide it), legs in the slot's leg holes.
// Frame: rack.glb's (assets/models/cad/rack.py: origin = centre of the footprint on the table,
// x along the long side, z up, metres); the plate's top is at z 9 mm. Hole pairs of Jan's holder
// (STL frame, mm from its min corner): column c at x 20.0 + 34.04 c, legs (c + 1) x 2.54 apart,
// row r at y 10.62 + 15.3 r; rack.glb mirrors y (turned 180 deg about x) about the centre.
// Slot numbers as engraved: S1 holds 1..5 from the front, S2 6..10, ...

import * as THREE from 'three';

const SIZE = [142.16, 82.5], TOP = 9, PITCH = 2.54;
const LED_COL = { red: 0xe0463a, green: 0x3f9a4a, yellow: 0xe8c33a, blue: 0x2f74c8, white: 0xf2f2f2 };
const BAND = [0x1a1a1a, 0x7b4a1e, 0xd32f2f, 0xf57c00, 0xfbc02d, 0x388e3c, 0x1565c0, 0x7b1fa2, 0x808080, 0xf2f2f2];
const JUMPER_COL = [0xd2483c, 0xe9b65a, 0x3f9a4a, 0x2f74c8];       // 1 .. 4 pitch
const metal = new THREE.MeshStandardMaterial({ color: 0xc8c8c8, metalness: 0.8, roughness: 0.3 });
const mat = (color, o = {}) => new THREE.MeshStandardMaterial({ color, roughness: 0.5, ...o });

// slot k (1..20) -> its two leg holes in rack.glb's frame (mm), and the leg spacing
export function slotHoles(k) {
  // engraved numbering: column-major, 1..5 in S1 from the front (STL row 4) to the back
  const c = Math.floor((k - 1) / 5), r = 4 - ((k - 1) % 5), half = ((c + 1) * PITCH) / 2;
  const x = 20.0 + 34.04 * c - SIZE[0] / 2, y = SIZE[1] / 2 - (10.62 + 15.3 * r);
  return { a: [x - half, y], b: [x + half, y], x, y, half };
}

// a wire through the points (mm), bends included
function wire(points, radius = 0.3, material = metal) {
  const g = new THREE.Group();
  for (let i = 1; i < points.length; i++) {
    const p = new THREE.Vector3(...points[i - 1]), q = new THREE.Vector3(...points[i]);
    const m = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, p.distanceTo(q), 8), material);
    m.position.copy(p).add(q).multiplyScalar(0.5);
    m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), q.clone().sub(p).normalize());
    g.add(m);
  }
  return g;
}

function bands(ohms) {
  if (!ohms) return [BAND[1], BAND[0], BAND[2]];
  const e = Math.max(0, Math.floor(Math.log10(ohms)) - 1), d = Math.round(ohms / 10 ** e);
  return [BAND[Math.floor(d / 10) % 10], BAND[d % 10], BAND[Math.min(e, 9)]];
}

function ohmsOf(name) {
  const m = name.match(/([\d.]+)\s*(k|m)?\s*ohm/i);
  return m ? Number(m[1]) * (m[2] ? (m[2].toLowerCase() === 'k' ? 1e3 : 1e6) : 1) : null;
}

// one part (an inventory option: part_id, name, type, pin_spacing) standing on its holes
function part(o, h) {
  const g = new THREE.Group(), [ax] = h.a, [bx] = h.b, y = h.y, sink = TOP - 4;
  const colour = (o.name.match(/red|green|yellow|blue|white/i)?.[0] || '').toLowerCase();
  if (o.type === 'led') {
    const lens = mat(LED_COL[colour] ?? 0xdddddd, { transparent: true, opacity: 0.85, roughness: 0.15 });
    const z0 = TOP + 9, x = h.x;
    const body = new THREE.Mesh(new THREE.CylinderGeometry(2.5, 2.5, 5.2, 24), lens);
    body.rotation.x = Math.PI / 2; body.position.set(x, y, z0 + 2.6);
    const dome = new THREE.Mesh(new THREE.SphereGeometry(2.5, 24, 12, 0, Math.PI * 2, 0, Math.PI / 2), lens);
    dome.rotation.x = Math.PI / 2; dome.position.set(x, y, z0 + 5.2);
    const rim = new THREE.Mesh(new THREE.CylinderGeometry(2.9, 2.9, 0.9, 24), lens);
    rim.rotation.x = Math.PI / 2; rim.position.set(x, y, z0 + 0.45);
    g.add(body, dome, rim,
      wire([[x - 1.27, y, z0], [x - 1.27, y, TOP + 3], [ax, y, TOP + 1.5], [ax, y, sink]]),
      wire([[x + 1.27, y, z0], [x + 1.27, y, TOP + 3], [bx, y, TOP + 1.5], [bx, y, sink]]));
  } else if (o.type === 'resistor') {
    const body = new THREE.Group(), L = 6.3;
    body.add(new THREE.Mesh(new THREE.CylinderGeometry(1.15, 1.15, L, 18), mat(0xd8c49a)));
    [...bands(ohmsOf(o.name)), 0xc9a15d].forEach((c, i) => {
      const b = new THREE.Mesh(new THREE.CylinderGeometry(1.2, 1.2, 0.55, 18), mat(c));
      b.position.y = -1.9 + i * 1.15 + (i === 3 ? 0.8 : 0);
      body.add(b);
    });
    if (bx - ax >= 7.6) {           // lying: body across the holes, leads bent down at both ends
      const z = TOP + 3.5;
      body.rotation.z = Math.PI / 2; body.position.set(h.x, y, z);
      g.add(body, wire([[h.x - L / 2, y, z], [ax, y, z], [ax, y, sink]]), wire([[h.x + L / 2, y, z], [bx, y, z], [bx, y, sink]]));
    } else {                        // standing on the first hole, the second lead looped back down
      const z = TOP + 2 + L / 2;
      body.rotation.x = Math.PI / 2; body.position.set(ax, y, z);
      g.add(body, wire([[ax, y, sink], [ax, y, TOP + 2]]),
        wire([[ax, y, z + L / 2], [ax, y, z + L / 2 + 1.5], [bx, y, z + L / 2 + 1.5], [bx, y, sink]]));
    }
  } else if (o.type === 'capacitor') {
    const z = TOP + 7;
    const disc = new THREE.Mesh(new THREE.CylinderGeometry(2.6, 2.6, 1.3, 24), mat(0xd9822b));     // axis y: stands upright, faces front and back
    disc.position.set(h.x, y, z);
    g.add(disc, wire([[h.x - 1.27, y, z - 2], [ax, y, TOP + 1.5], [ax, y, sink]]), wire([[h.x + 1.27, y, z - 2], [bx, y, TOP + 1.5], [bx, y, sink]]));
  } else if (o.type === 'jumper') {
    const n = Math.round((bx - ax) / PITCH), z = TOP + 3.5;
    const ins = mat(JUMPER_COL[(n - 1) % JUMPER_COL.length]);
    g.add(wire([[ax, y, sink], [ax, y, z]]), wire([[bx, y, sink], [bx, y, z]]),
      wire([[ax, y, z], [bx, y, z]], 0.55, ins));
  } else {
    g.add(wire([[ax, y, sink], [ax, y, TOP + 4], [bx, y, TOP + 4], [bx, y, sink]]));
  }
  return g;
}

// the parts of /api/rack's contents, as one group in rack.glb's frame (metres)
export function rackParts(r) {
  const out = new THREE.Group();
  out.name = 'rack-parts';
  const opt = Object.fromEntries((r?.options || []).map((o) => [o.part_id, o]));
  for (const [slot, it] of Object.entries(r?.contents || {})) {
    const o = opt[it.part_id];
    if (!o) continue;
    out.add(part(o, slotHoles(Number(slot.slice(1)))));
  }
  out.scale.setScalar(0.001);
  return out;
}
