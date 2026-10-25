// TP-Link LS108G for the Port QA tab, built procedurally (no public 3D model):
// steel case 158 x 99.1 x 25 mm (retail spec sheets), 8 shielded RJ45 jacks in
// a row on the port face, two LEDs per jack. Units mm, z up, port face toward -y,
// port 1 on the left seen from the front. Jacks are tinted by QA status; the
// port under test gets the robot's plug + grip shell and lit LEDs.

import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';
import { CSS2DRenderer, CSS2DObject } from './vendor/CSS2DRenderer.js';

const W = 158, D = 99.1, H = 25;
const PITCH = 16, JACK_Z = 11;           // jack row: 8 x 16 mm, centred
const FACE = -D / 2;
const COL = { bg: 0x111111, cream: 0xfef5da, brass: 0xc9a15d, red: 0xd2483c, steel: 0x9aa0a6,
  ink: 0x111111, shell: 0xe2711d, cable: 0x3b6fd6, green: 0x5cff7a, amber: 0xffb347 };

// RJ45 jack opening seen from the front, latch slot at the bottom
function jackOpening(cx = 0, cz = 0) {
  const s = new THREE.Shape();
  const w = 11.8, h = 8.6, nw = 6.2, nh = 2.2;
  s.moveTo(cx - w / 2, cz - h / 2 + nh);
  s.lineTo(cx - nw / 2, cz - h / 2 + nh);
  s.lineTo(cx - nw / 2, cz - h / 2);
  s.lineTo(cx + nw / 2, cz - h / 2);
  s.lineTo(cx + nw / 2, cz - h / 2 + nh);
  s.lineTo(cx + w / 2, cz - h / 2 + nh);
  s.lineTo(cx + w / 2, cz + h / 2);
  s.lineTo(cx - w / 2, cz + h / 2);
  s.closePath();
  return s;
}

function roundedRect(w, d, r) {
  const s = new THREE.Shape();
  s.moveTo(-w / 2 + r, -d / 2);
  s.lineTo(w / 2 - r, -d / 2); s.quadraticCurveTo(w / 2, -d / 2, w / 2, -d / 2 + r);
  s.lineTo(w / 2, d / 2 - r); s.quadraticCurveTo(w / 2, d / 2, w / 2 - r, d / 2);
  s.lineTo(-w / 2 + r, d / 2); s.quadraticCurveTo(-w / 2, d / 2, -w / 2, d / 2 - r);
  s.lineTo(-w / 2, -d / 2 + r); s.quadraticCurveTo(-w / 2, -d / 2, -w / 2 + r, -d / 2);
  return s;
}

// a shape drawn in the x/z plane (shape y -> world z), extruded toward -y:
// the mesh spans y in [-depth, 0], so position.y is its back face
function faceExtrude(shape, depth, mat) {
  const g = new THREE.ExtrudeGeometry(shape, { depth, bevelEnabled: false });
  g.rotateX(Math.PI / 2);
  return new THREE.Mesh(g, mat);
}

function label(text, cls) {
  const el = document.createElement('div');
  el.className = cls;
  el.innerHTML = text;
  return { el, obj: new CSS2DObject(el) };
}

export class SwitchView {
  constructor(container, { onPick } = {}) {
    this.container = container;
    this.onPick = onPick;
    const renderer = this.renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.setClearColor(COL.bg);
    container.appendChild(renderer.domElement);
    this.labels = new CSS2DRenderer();
    Object.assign(this.labels.domElement.style, { position: 'absolute', top: '0', pointerEvents: 'none' });
    container.appendChild(this.labels.domElement);

    const scene = this.scene = new THREE.Scene();
    const cam = this.camera = new THREE.PerspectiveCamera(32, 1, 5, 3000);
    cam.up.set(0, 0, 1);
    this.home = { pos: new THREE.Vector3(-70, -250, 135), target: new THREE.Vector3(0, -20, 8) };
    cam.position.copy(this.home.pos);
    this.controls = new OrbitControls(cam, renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.target.copy(this.home.target);

    scene.add(new THREE.HemisphereLight(0xffffff, 0x2a2622, 1.6));
    const key = new THREE.DirectionalLight(0xffffff, 1.6);
    key.position.set(-120, -200, 260);
    scene.add(key);
    const rim = new THREE.DirectionalLight(0xfef5da, 0.6);
    rim.position.set(200, 150, 80);
    scene.add(rim);
    const grid = new THREE.GridHelper(600, 30, 0x2f2f2f, 0x1d1d1d);
    grid.rotation.x = Math.PI / 2;
    grid.position.z = -2.5;
    scene.add(grid);

    this._buildCase();
    this._buildJacks();
    this._buildPlug();
    this._buildVlmMarker();

    this.ray = new THREE.Raycaster();
    renderer.domElement.addEventListener('click', (e) => this._click(e));
    new ResizeObserver(() => this.resize()).observe(container);
    this.resize();
    const loop = () => {
      requestAnimationFrame(loop);
      if (!container.offsetParent) return;        // tab hidden
      this.controls.update();
      this._animate();
      renderer.render(scene, cam);
      this.labels.render(scene, cam);
    };
    loop();
  }

  _buildCase() {
    const steel = new THREE.MeshStandardMaterial({ color: 0x23272d, metalness: 0.55, roughness: 0.42 });
    const body = new THREE.Mesh(new THREE.ExtrudeGeometry(roundedRect(W - 1.6, D - 1.6, 3.2),
      { depth: H - 1.6, bevelEnabled: true, bevelThickness: 0.8, bevelSize: 0.8, bevelSegments: 3 }), steel);
    body.position.z = 0.8;
    this.scene.add(body);
    // seam where the U-shaped top cover meets the base, 3 mm up
    const seam = new THREE.Mesh(new THREE.BoxGeometry(W + 0.1, D + 0.1, 0.4),
      new THREE.MeshStandardMaterial({ color: 0x0b0c0e, roughness: 0.9 }));
    seam.position.z = 3;
    this.scene.add(seam);
    const rubber = new THREE.MeshStandardMaterial({ color: 0x0a0a0a, roughness: 1 });
    for (const [x, y] of [[-1, -1], [1, -1], [-1, 1], [1, 1]]) {
      const f = new THREE.Mesh(new THREE.CylinderGeometry(4, 4, 2, 20), rubber);
      f.rotation.x = Math.PI / 2;
      f.position.set(x * (W / 2 - 12), y * (D / 2 - 12), -1);
      this.scene.add(f);
    }
    // DC barrel jack on the opposite face
    const dc = new THREE.Mesh(new THREE.CylinderGeometry(4, 4, 1.2, 28),
      new THREE.MeshStandardMaterial({ color: 0x050505, roughness: 0.6 }));
    dc.position.set(W / 2 - 18, D / 2 + 0.5, 11);
    this.scene.add(dc);
    const pin = new THREE.Mesh(new THREE.CylinderGeometry(1, 1, 1.4, 12),
      new THREE.MeshStandardMaterial({ color: 0xc0c0c0, metalness: 0.8, roughness: 0.3 }));
    pin.position.copy(dc.position);
    this.scene.add(pin);
    const tag = label('LS108G <small>8-port gigabit</small>', 'label3d cam');
    tag.obj.position.set(-W / 2 + 22, -D / 2 + 14, H + 0.5);
    this.scene.add(tag.obj);
  }

  _buildJacks() {
    this.jacks = [];
    const cavity = new THREE.MeshBasicMaterial({ color: 0x030303 });
    const gold = new THREE.MeshStandardMaterial({ color: 0xd4a83a, metalness: 0.9, roughness: 0.3 });
    for (let i = 0; i < 8; i++) {
      const x = (i - 3.5) * PITCH;
      const port = i + 1;
      // metal shield: 15 x 13.4 plate with the RJ45 opening cut out
      const plate = new THREE.Shape();
      plate.moveTo(x - 7.5, JACK_Z - 6.7); plate.lineTo(x + 7.5, JACK_Z - 6.7);
      plate.lineTo(x + 7.5, JACK_Z + 6.7); plate.lineTo(x - 7.5, JACK_Z + 6.7); plate.closePath();
      plate.holes.push(jackOpening(x, JACK_Z));
      const shieldMat = new THREE.MeshStandardMaterial({ color: COL.steel, metalness: 0.85, roughness: 0.28,
        emissive: 0x000000 });
      const shield = faceExtrude(plate, 0.8, shieldMat);
      shield.position.y = FACE;                     // 0.8 mm proud of the face
      const hole = faceExtrude(jackOpening(x, JACK_Z), 14, cavity);
      hole.position.y = FACE + 14 - 0.05;           // 14 mm deep, front just ahead of the face
      // the 8 contact fingers along the top of the opening
      const contacts = new THREE.Group();
      for (let k = 0; k < 8; k++) {
        const c = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.3, 1.2), gold);
        c.position.set(x - 4.55 + k * 1.3, FACE + 3, JACK_Z + 3.4);
        contacts.add(c);
      }
      const leds = [-1, 1].map((s) => {
        const m = new THREE.MeshStandardMaterial({ color: 0x1a1a1a, emissive: 0x000000, roughness: 0.4 });
        const led = new THREE.Mesh(new THREE.BoxGeometry(2.6, 0.6, 1.6), m);
        led.position.set(x + s * 5.2, FACE - 0.5, JACK_Z + 8.6);
        return led;
      });
      for (const o of [shield, hole]) o.userData.port = port;
      this.scene.add(shield, hole, contacts, ...leds);
      const num = label(String(port), 'label3d');
      num.obj.position.set(x, FACE - 1, JACK_Z - 10.5);
      this.scene.add(num.obj);
      this.jacks.push({ port, x, shield, shieldMat, hole, leds, num });
    }
  }

  // robot cable: plug in the jack, orange fat-H grip shell, cable trailing out
  _buildPlug() {
    const g = this.plug = new THREE.Group();
    const plugMat = new THREE.MeshStandardMaterial({ color: 0xdfe6ea, roughness: 0.25, transparent: true, opacity: 0.75 });
    const shellMat = new THREE.MeshStandardMaterial({ color: COL.shell, roughness: 0.6 });
    const box = (w, d, h, x, y, z, m) => {
      const b = new THREE.Mesh(new THREE.BoxGeometry(w, d, h), m);
      b.position.set(x, y, z);
      g.add(b);
    };
    // y = 0 at the port face, -y out of the switch, z = 0 on the jack centre;
    // plug 15 of its 21.5 mm inside, shell dimensions from rj45_grip.py
    box(11.7, 21.5, 7.9, 0, 15 - 21.5 / 2, 0, plugMat);
    const s0 = -0.5;                                  // shell front, 0.5 mm off the face
    box(14.5, 41.9, 10.6, 0, s0 - 41.9 / 2, 0.55, shellMat);
    for (const ly of [-5, -33.4]) box(24, 4, 22.6, 0, ly, 6.55, shellMat);
    box(8, 24.4, 12, 0, -19.2, 11.85, shellMat);
    const curve = new THREE.CatmullRomCurve3([
      new THREE.Vector3(0, s0 - 41, 0), new THREE.Vector3(0, s0 - 75, -2),
      new THREE.Vector3(10, s0 - 120, -12), new THREE.Vector3(40, s0 - 160, -14)]);
    g.add(new THREE.Mesh(new THREE.TubeGeometry(curve, 40, 3, 12, false),
      new THREE.MeshStandardMaterial({ color: COL.cable, roughness: 0.55 })));
    g.visible = false;
    this.scene.add(g);
  }

  _buildVlmMarker() {
    const g = this.vlm = new THREE.Group();
    const cone = new THREE.Mesh(new THREE.ConeGeometry(3.2, 8, 4),
      new THREE.MeshStandardMaterial({ color: COL.cream, emissive: 0x3a3424 }));
    cone.rotation.x = -Math.PI / 2;
    g.add(cone);
    this.vlmLabel = label('', 'label3d dark');
    this.vlmLabel.obj.position.set(0, 0, 9);
    g.add(this.vlmLabel.obj);
    g.visible = false;
    this.scene.add(g);
  }

  // ports: [{port, status}], sel: selected port, active: port under test,
  // link: {up, speed}, vlm: {port, confidence, fresh} | null
  update({ ports, sel, active, testing, link, vlm }) {
    this.t = this.t || 0;
    this.testing = testing;
    for (const j of this.jacks) {
      const st = ports[j.port - 1]?.status;
      const isActive = j.port === active;
      const c = isActive && testing ? COL.brass : st === 'pass' ? COL.cream : st === 'fail' ? COL.red : COL.steel;
      j.shieldMat.color.setHex(c);
      j.shieldMat.metalness = st === 'pass' || st === 'fail' || (isActive && testing) ? 0.2 : 0.85;
      j.num.el.className = j.port === sel ? 'label3d dark' : 'label3d';
      const lit = isActive && link?.up;
      j.leds[0].material.emissive.setHex(lit ? (link.speed >= 1000 ? COL.green : COL.amber) : 0x000000);
      j.leds[1].material.emissive.setHex(lit ? COL.green : 0x000000);
    }
    const a = this.jacks[active - 1];
    this.plug.visible = !!a && (link?.up || testing);
    if (a) this.plug.position.set(a.x, FACE, JACK_Z);
    const v = vlm && this.jacks[vlm.port - 1];
    this.vlm.visible = !!v;
    if (v) {
      this.vlm.position.set(v.x, FACE - 4, H + 14);
      this.vlmLabel.el.innerHTML = `VLM <small>${vlm.confidence != null ? Math.round(vlm.confidence * 100) + '%' : ''}${vlm.fresh ? '' : ' · stale'}</small>`;
    }
  }

  _animate() {
    this.t = (this.t || 0) + 1;
    if (this.vlm.visible) this.vlm.children[0].position.z = Math.sin(this.t / 18) * 1.5;
    if (this.testing) {
      for (const j of this.jacks) {
        if (j.shieldMat.color.getHex() === COL.brass) j.shieldMat.emissive.setHex(Math.sin(this.t / 8) > 0 ? 0x3a2a10 : 0x000000);
        else j.shieldMat.emissive.setHex(0x000000);
      }
    }
  }

  _click(e) {
    const r = this.renderer.domElement.getBoundingClientRect();
    const p = new THREE.Vector2(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
    this.ray.setFromCamera(p, this.camera);
    const hit = this.ray.intersectObjects(this.jacks.flatMap((j) => [j.shield, j.hole]))[0];
    if (hit && this.onPick) this.onPick(hit.object.userData.port);
  }

  resetView() {
    this.camera.position.copy(this.home.pos);
    this.controls.target.copy(this.home.target);
  }

  resize() {
    const w = this.container.clientWidth, h = this.container.clientHeight;
    if (!w || !h) return;
    this.renderer.setSize(w, h);
    this.labels.setSize(w, h);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }
}
