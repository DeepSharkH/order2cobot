// Background of the new-project screen: kit parts (breadboard, LED, ESP32, resistor, capacitor,
// jumper) pop up with a little bounce, spin, drop and fade. Away from the edges and the centre,
// where the prompt is. Only drawn while the screen shows.

import * as THREE from 'three';

const CREAM = 0xfef5da, BRASS = 0xc9a15d, RED = 0xd2483c, INK = 0x1a1a1a, METAL = 0xb8b8b8;
const mat = (color, o = {}) => new THREE.MeshStandardMaterial({ color, roughness: 0.55, metalness: 0, ...o });
const box = (w, h, d, m) => new THREE.Mesh(new THREE.BoxGeometry(w, h, d), m);
const cyl = (r, h, m, seg = 20) => new THREE.Mesh(new THREE.CylinderGeometry(r, r, h, seg), m);

function holesTexture(cols, rows) {
  const c = document.createElement('canvas'), s = 16;
  c.width = cols * s; c.height = rows * s;
  const g = c.getContext('2d');
  g.fillStyle = '#f3ecd8'; g.fillRect(0, 0, c.width, c.height);
  g.fillStyle = '#d9d0b8'; g.fillRect(0, c.height / 2 - s * 0.6, c.width, s * 1.2);
  g.fillStyle = '#2a2a2a';
  for (let x = 0; x < cols; x++) for (let y = 0; y < rows; y++) {
    if (Math.abs(y - (rows - 1) / 2) < 1) continue;
    g.fillRect(x * s + 5, y * s + 5, 6, 6);
  }
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

function lead(len, x, z = 0) {
  const m = cyl(0.35, len, mat(METAL, { metalness: 0.8, roughness: 0.3 }), 8);
  m.position.set(x, -len / 2, z);
  return m;
}

const MODELS = {
  breadboard() {
    const g = new THREE.Group();
    const side = mat(0xe9e1c9);
    const top = new THREE.MeshStandardMaterial({ map: holesTexture(30, 12), roughness: 0.7 });
    g.add(new THREE.Mesh(new THREE.BoxGeometry(60, 6, 24), [side, side, top, side, side, side]));
    return g;
  },
  led() {
    const g = new THREE.Group(), red = mat(RED, { transparent: true, opacity: 0.85, roughness: 0.2 });
    const body = cyl(2.5, 5, red); body.position.y = 2.5;
    const dome = new THREE.Mesh(new THREE.SphereGeometry(2.5, 20, 12, 0, Math.PI * 2, 0, Math.PI / 2), red); dome.position.y = 5;
    const rim = cyl(2.9, 0.8, red); rim.position.y = 0.4;
    g.add(body, dome, rim, lead(14, -1.2), lead(17, 1.2));
    g.scale.setScalar(1.6);
    return g;
  },
  esp32() {
    const g = new THREE.Group();
    g.add(box(48, 1.6, 26, mat(0x1d2b3a)));
    const shield = box(17, 2.4, 18, mat(METAL, { metalness: 0.9, roughness: 0.25 })); shield.position.set(-10, 2, 0);
    const ant = box(7, 0.4, 18, mat(0x2b2b2b)); ant.position.set(-22, 1, 0);
    const usb = box(6, 2.6, 8, mat(METAL, { metalness: 0.9, roughness: 0.3 })); usb.position.set(23, 1.8, 0);
    g.add(shield, ant, usb);
    for (const z of [-11.5, 11.5]) {
      const hdr = box(40, 2.4, 2.4, mat(INK)); hdr.position.set(2, -2, z); g.add(hdr);
      for (let i = 0; i < 15; i++) { const p = box(0.6, 6, 0.6, mat(BRASS, { metalness: 0.8 })); p.position.set(-15 + i * 2.54, -4.5, z); g.add(p); }
    }
    return g;
  },
  resistor() {
    const g = new THREE.Group(), body = new THREE.Group();
    const c = cyl(2, 9, mat(0xd8c49a)); body.add(c);
    [0x7b4a1e, 0x1a1a1a, RED, BRASS].forEach((col, i) => { const b = cyl(2.08, 0.9, mat(col)); b.position.y = -3 + i * 1.8 + (i === 3 ? 0.6 : 0); body.add(b); });
    body.rotation.z = Math.PI / 2;
    const l = cyl(0.35, 30, mat(METAL, { metalness: 0.8 }), 8); l.rotation.z = Math.PI / 2;
    g.add(l, body);
    g.scale.setScalar(1.5);
    return g;
  },
  capacitor() {
    const g = new THREE.Group();
    const disc = cyl(4, 1.6, mat(0xd9822b)); disc.rotation.x = Math.PI / 2; disc.position.y = 5;
    g.add(disc, lead(10, -1.3), lead(10, 1.3));
    g.scale.setScalar(1.7);
    return g;
  },
  jumper() {
    const curve = new THREE.CatmullRomCurve3([new THREE.Vector3(-14, 0, 0), new THREE.Vector3(-12, 9, 0), new THREE.Vector3(12, 9, 0), new THREE.Vector3(14, 0, 0)]);
    const g = new THREE.Group();
    g.add(new THREE.Mesh(new THREE.TubeGeometry(curve, 32, 0.9, 8), mat(BRASS)));
    for (const x of [-14, 14]) g.add(lead(4, x));
    g.scale.setScalar(1.8);
    return g;
  },
};
const KINDS = Object.keys(MODELS);

export function startBackground(host) {
  if (matchMedia('(prefers-reduced-motion: reduce)').matches) return;
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 1.5));
  renderer.domElement.className = 'bg3d';
  host.prepend(renderer.domElement);
  const scene = new THREE.Scene();
  scene.add(new THREE.AmbientLight(0xffffff, 1.4));
  const sun = new THREE.DirectionalLight(0xffffff, 2.2);
  sun.position.set(-1, 2, 3);
  scene.add(sun);
  const camera = new THREE.OrthographicCamera(0, 1, 1, 0, -500, 500);
  let W = 1, H = 1;
  const resize = () => {
    W = host.clientWidth || 1; H = host.clientHeight || 1;
    renderer.setSize(W, H, false);
    Object.assign(camera, { left: -W / 2, right: W / 2, top: H / 2, bottom: -H / 2 });
    camera.updateProjectionMatrix();
  };
  new ResizeObserver(resize).observe(host);
  resize();

  const live = [];
  let next = 0.4, last = performance.now(), kindAt = Math.floor(Math.random() * KINDS.length);
  function spawn() {
    for (let tries = 0; tries < 30; tries++) {
      const x = (Math.random() - 0.5) * W * 0.84, y = (Math.random() - 0.5) * H * 0.7 - 30;
      if ((x / Math.max(W * 0.36, 400)) ** 4 + ((y + 20) / Math.max(H * 0.34, 230)) ** 4 < 1) continue;      // the prompt's area
      if (live.some((p) => Math.hypot(p.x0 - x, p.y0 - y) < 170)) continue;
      const obj = MODELS[KINDS[kindAt++ % KINDS.length]]();
      obj.traverse((o) => { if (o.material) o.material = [o.material].flat().map((m) => Object.assign(m.clone(), { transparent: true })); });
      const size = new THREE.Box3().setFromObject(obj).getSize(new THREE.Vector3());
      const s = (90 + Math.random() * 30) / Math.max(size.x, size.y, size.z);
      const pivot = new THREE.Group();
      pivot.add(obj);
      pivot.rotation.set(0.5 + Math.random() * 0.5, Math.random() * Math.PI * 2, (Math.random() - 0.5) * 0.4);
      scene.add(pivot);
      live.push({ pivot, obj, s, x0: x, y0: y, t: 0, vy: 85 + Math.random() * 30, spin: (Math.random() < 0.5 ? -1 : 1) * (0.5 + Math.random() * 0.5) });
      return;
    }
  }
  const G = 62, FALL = 90;      // px/s², px below the start before it is gone
  function frame(now) {
    const dt = Math.min((now - last) / 1000, 0.05);
    last = now;
    if (!host.hidden && !document.hidden) {
      next -= dt;
      if (next <= 0 && live.length < 6) { spawn(); next = 1.6 + Math.random() * 1.6; }
      for (let i = live.length - 1; i >= 0; i--) {
        const p = live[i];
        p.t += dt;
        const y = p.y0 + p.vy * p.t - 0.5 * G * p.t * p.t;
        const end = (p.vy + Math.sqrt(p.vy * p.vy + 2 * G * FALL)) / G;      // time it reaches y0 − FALL
        const pop = Math.min(p.t / 0.45, 1), back = 1 + 2.2 * (pop - 1) ** 3 + 1.2 * (pop - 1) ** 2;      // ease-out-back
        const fade = Math.min(1, Math.max(0, (end - p.t) / 1.1));
        p.pivot.position.set(p.x0, y, 0);
        p.pivot.scale.setScalar(p.s * back);
        p.pivot.rotation.y += p.spin * dt;
        p.pivot.rotation.x += p.spin * 0.25 * dt;
        p.obj.traverse((o) => { if (o.material) for (const m of [o.material].flat()) m.opacity = (m.userData.o ??= m.opacity) * fade; });
        if (p.t >= end) {
          scene.remove(p.pivot);
          p.obj.traverse((o) => { o.geometry?.dispose(); for (const m of [o.material || []].flat()) { m.map?.dispose(); m.dispose(); } });
          live.splice(i, 1);
        }
      }
      renderer.render(scene, camera);
    }
    requestAnimationFrame(frame);
  }
  requestAnimationFrame(frame);
}
