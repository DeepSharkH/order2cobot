// Port QA 3D view: the TP-Link LS108G, loaded from the CAD build in models/cad/
// (assets/models/cad/*.py). The switch model follows the official spec + photos (latch
// slot up, one Link/Act LED top-left of each jack) with the pitch we measured.
// Units mm in this view (the GLBs are metres), z up, port face toward -y, port 1 on the
// left seen from the front. Jacks are tinted by QA status; the port under test gets the
// robot's plug + grip shell and a lit LED.

import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';
import { CSS2DRenderer, CSS2DObject } from './vendor/CSS2DRenderer.js';
import { GLTFLoader } from './vendor/GLTFLoader.js';

const CAD = './models/cad/';
const COL = { bg: 0x111111, cream: 0xfef5da, brass: 0xc9a15d, red: 0xd2483c, idle: 0x1f1f21,
  shell: 0xe2711d, cable: 0x3b6fd6, green: 0x5cff7a, amber: 0xffb347 };
const SEL_GLOW = new THREE.Color(0x2a2418), TEST_GLOW = new THREE.Color(0x4a3410), NONE = new THREE.Color(0);

function label(text, cls) {
  const el = document.createElement('div');
  el.className = cls;
  el.innerHTML = text;
  return { el, obj: new CSS2DObject(el) };
}

async function loadCad(name) {
  const [gltf, spec] = await Promise.all([
    new GLTFLoader().loadAsync(`${CAD}${name}.glb`),
    fetch(`${CAD}${name}.json`).then((r) => r.json())]);
  const g = gltf.scene;
  g.scale.setScalar(1000);                       // m -> mm
  return { g, spec };
}

export class SwitchView {
  constructor(container, { onPick } = {}) {
    this.container = container;
    this.onPick = onPick;
    this.jacks = [];
    const renderer = this.renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.setClearColor(COL.bg);
    container.appendChild(renderer.domElement);
    this.labels = new CSS2DRenderer();
    Object.assign(this.labels.domElement.style, { position: 'absolute', top: '0', pointerEvents: 'none' });
    container.appendChild(this.labels.domElement);

    const scene = this.scene = new THREE.Scene();
    const cam = this.camera = new THREE.PerspectiveCamera(32, 1, 40, 2000);
    cam.up.set(0, 0, 1);
    this.home = { pos: new THREE.Vector3(-30, -300, 150), target: new THREE.Vector3(0, -15, 8) };
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
    const grid = new THREE.GridHelper(700, 35, 0x2f2f2f, 0x1d1d1d);
    grid.rotation.x = Math.PI / 2;
    grid.position.set(0, 0, -0.5);
    scene.add(grid);

    this._buildPlug();
    this._buildVlmMarker();
    this.ready = this._loadSwitch()
      .then(() => this._last && this.update(this._last))
      .catch((e) => console.error('CAD models', e));

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

  async _loadSwitch() {
    const { g, spec } = await loadCad('ls108g');
    this.sw = spec;
    this.scene.add(g);
    const pickMat = new THREE.MeshBasicMaterial({ visible: false });
    for (let k = 1; k <= 8; k++) {
      const p = spec.ports[k];
      const mouth = g.getObjectByName(`jack_${k}`);
      const led = g.getObjectByName(`led_${k}`);
      // the jack and LED faces lie on the switch's front face: pull them forward in depth,
      // or the two fight and the ports flicker as the view moves
      mouth.material = Object.assign(mouth.material.clone(), { polygonOffset: true, polygonOffsetFactor: -2, polygonOffsetUnits: -2 });
      led.material = Object.assign(led.material.clone(), { polygonOffset: true, polygonOffsetFactor: -2, polygonOffsetUnits: -2 });
      // invisible box over the opening for clicks
      const pick = new THREE.Mesh(new THREE.BoxGeometry(spec.pitch - 1, 4, spec.opening.bbox[1] + 2), pickMat);
      pick.position.set(p.centre[0], p.centre[1] + 1, p.centre[2]);
      pick.userData.port = k;
      this.scene.add(pick);
      this.jacks.push({ port: k, x: p.centre[0], z: p.centre[2], mouth, led, pick });
    }
  }

  // robot cable: plug in the jack (latch side up), orange fat-H grip shell, cable trailing out
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
    // plug 13 of its 21.5 mm inside (seat depth), shell dimensions from rj45_grip.py
    box(11.7, 21.5, 7.9, 0, 13 - 21.5 / 2, -0.4, plugMat);
    const s0 = -0.5 - 2;                              // shell front, 2.5 mm off the face when seated
    box(14.5, 41.9, 10.6, 0, s0 - 41.9 / 2, 0.15, shellMat);
    for (const ly of [-7, -35.4]) box(24, 4, 22.6, 0, ly, 6.15, shellMat);
    box(8, 24.4, 12, 0, -21.2, 11.45, shellMat);
    const curve = new THREE.CatmullRomCurve3([
      new THREE.Vector3(0, s0 - 41, 0), new THREE.Vector3(0, s0 - 75, -2),
      new THREE.Vector3(10, s0 - 120, -12), new THREE.Vector3(40, s0 - 160, -12)]);
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
  update(state) {
    this._last = state;
    const { ports, sel, active, testing, link, vlm } = state;
    this.testing = testing;
    this.sel = sel;
    if (!this.sw) return;                         // models still loading
    for (const j of this.jacks) {
      const st = ports[j.port - 1]?.status;
      const isActive = j.port === active;
      const c = isActive && testing ? COL.brass : st === 'pass' ? COL.cream : st === 'fail' ? COL.red : COL.idle;
      j.mouth.material.color.setHex(c);
      const lit = isActive && link?.up;
      j.led.material.emissive.setHex(lit ? (link.speed >= 1000 ? COL.green : COL.amber) : 0x000000);
    }
    const a = this.jacks[active - 1];
    this.plug.visible = !!a && (link?.up || testing);
    if (a) this.plug.position.set(a.x, this.sw.face_y, a.z);
    const v = vlm && this.jacks[vlm.port - 1];
    this.vlm.visible = !!v;
    if (v) {
      this.vlm.position.set(v.x, this.sw.face_y - 4, this.sw.size[2] + 14);
      this.vlmLabel.el.innerHTML = `VLM <small>${vlm.confidence != null ? Math.round(vlm.confidence * 100) + '%' : ''}${vlm.fresh ? '' : ' · stale'}</small>`;
    }
  }

  _animate() {
    this.t = (this.t || 0) + 1;
    if (this.vlm.visible) this.vlm.children[0].position.z = Math.sin(this.t / 18) * 1.5;
    // the port under test breathes slowly, the selected one glows faintly: no on/off strobe
    const breath = 0.5 + 0.5 * Math.sin(this.t / 30);
    for (const j of this.jacks) {
      const testing = this.testing && j.mouth.material.color.getHex() === COL.brass;
      const e = j.mouth.material.emissive;
      if (testing) e.copy(NONE).lerp(TEST_GLOW, breath);
      else e.copy(j.port === this.sel ? SEL_GLOW : NONE);
    }
  }

  _click(e) {
    const r = this.renderer.domElement.getBoundingClientRect();
    const p = new THREE.Vector2(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
    this.ray.setFromCamera(p, this.camera);
    const hit = this.ray.intersectObjects(this.jacks.map((j) => j.pick))[0];
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
