// Objects tab: the scene in the world frame (checkerboard centre, z up, metres) built from
// both cameras' depth clouds, with the bounding box of a located object (dashboard/objects.py,
// agents/vision/locate.py): the box, its model instances, the cloud-only fit when the image
// fit moved it, and a label with its size. Clouds are placed with the camera pose they
// arrive with and can be coloured by height above the plate the object rests on.

import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';
import { CSS2DRenderer, CSS2DObject } from './vendor/CSS2DRenderer.js';

const COL = { bg: 0x111111, cream: 0xfef5da, brass: 0xc9a15d, grey: 0x8c8577 };
const EDGES = [[0, 1], [1, 2], [2, 3], [3, 0], [4, 5], [5, 6], [6, 7], [7, 4], [0, 4], [1, 5], [2, 6], [3, 7]];
const HOME = { pos: new THREE.Vector3(0.45, -0.75, 0.6), target: new THREE.Vector3(0, 0, 0) };

// height above the plate -> colour: ink at the plate, brass at the object top, cream above
function heightColour(h, top, out, i) {
  const t = Math.max(0, Math.min(1, h / Math.max(top, 1e-3)));
  const a = [0.17, 0.16, 0.15], b = [0.79, 0.63, 0.36], c = [0.996, 0.96, 0.855];
  const [p, q, s] = t < 1 ? [a, b, t] : [b, c, Math.min(1, (h - top) / (2 * top))];
  for (let k = 0; k < 3; k++) out[i + k] = p[k] + (q[k] - p[k]) * s;
}

function boxLines(corners, color, dashed) {
  const pts = [];
  for (const [a, b] of EDGES) pts.push(new THREE.Vector3(...corners[a]), new THREE.Vector3(...corners[b]));
  const g = new THREE.BufferGeometry().setFromPoints(pts);
  const m = dashed
    ? new THREE.LineDashedMaterial({ color, dashSize: 0.004, gapSize: 0.003, depthTest: false, transparent: true, opacity: 0.85 })
    : new THREE.LineBasicMaterial({ color, depthTest: false, transparent: true });
  const l = new THREE.LineSegments(g, m);
  if (dashed) l.computeLineDistances();
  l.renderOrder = 2;
  return l;
}

export class ObjectsView {
  constructor(container) {
    this.container = container;
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setClearColor(COL.bg);
    container.appendChild(this.renderer.domElement);
    this.labels = new CSS2DRenderer();
    Object.assign(this.labels.domElement.style, { position: 'absolute', top: '0', pointerEvents: 'none' });
    container.appendChild(this.labels.domElement);

    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(40, 1, 0.005, 20);
    this.camera.up.set(0, 0, 1);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.resetView();

    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x2a2622, 1.6));
    const grid = new THREE.GridHelper(1, 20, 0x2f2f2f, 0x1d1d1d);   // 5 cm cells
    grid.rotation.x = Math.PI / 2;
    this.scene.add(grid);
    this.scene.add(new THREE.AxesHelper(0.05));

    // one cloud per camera, stored in world coordinates (transformed on arrival)
    this.clouds = [0, 1].map((i) => {
      const p = new THREE.Points(new THREE.BufferGeometry(),
        new THREE.PointsMaterial({ size: i === 0 ? 0.003 : 0.0018, vertexColors: true, sizeAttenuation: true }));
      p.frustumCulled = false;
      this.scene.add(p);
      return { pts: p, xyz: null, rgb: null };
    });
    this.colour = 'rgb';
    this.plate = { z: 0, top: 0.0095 };

    this.boxG = new THREE.Group();
    this.scene.add(this.boxG);
    this.key = null;

    new ResizeObserver(() => this.resize()).observe(container);
    const loop = () => {
      requestAnimationFrame(loop);
      if (!this.container.offsetParent) return;       // tab hidden: don't render
      this.controls.update();
      this.renderer.render(this.scene, this.camera);
      this.labels.render(this.scene, this.camera);
    };
    loop();
  }

  resize() {
    const w = this.container.clientWidth, h = this.container.clientHeight;
    if (!w || !h) return;
    this.renderer.setSize(w, h);
    this.labels.setSize(w, h);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  resetView() {
    this.camera.position.copy(HOME.pos);
    this.controls.target.copy(HOME.target);
  }

  // look at the located box from the front-right, close enough to see its edges
  focus() {
    if (!this.result) return this.resetView();
    const [x, y, z] = this.result.box.centre;
    const r = Math.max(...this.result.box.size_mm) / 1000;
    this.controls.target.set(x, y, z);
    this.camera.position.set(x + 0.9 * r, y - 1.5 * r, z + 1.3 * r);
  }

  // cloud message [u8 2][u8 cam][2 pad][u32 n] + n*3 f32 xyz (camera optical frame) + n*3 u8 rgb
  setCloud(idx, buf, T) {
    const c = this.clouds[idx];
    if (!c || !T) return;
    const n = new DataView(buf).getUint32(4, true);
    const src = new Float32Array(buf, 8, n * 3);
    const xyz = new Float32Array(n * 3);
    for (let i = 0; i < n * 3; i += 3) {
      const a = src[i], b = src[i + 1], d = src[i + 2];
      xyz[i] = T[0][0] * a + T[0][1] * b + T[0][2] * d + T[0][3];
      xyz[i + 1] = T[1][0] * a + T[1][1] * b + T[1][2] * d + T[1][3];
      xyz[i + 2] = T[2][0] * a + T[2][1] * b + T[2][2] * d + T[2][3];
    }
    c.xyz = xyz;
    c.rgb = new Uint8Array(buf, 8 + n * 12, n * 3).slice();
    c.pts.geometry.setAttribute('position', new THREE.BufferAttribute(xyz, 3));
    this.paint(c);
  }

  paint(c) {
    if (!c.xyz) return;
    const n = c.xyz.length / 3;
    const col = new Float32Array(n * 3);
    if (this.colour === 'height') {
      for (let i = 0; i < n * 3; i += 3) heightColour(c.xyz[i + 2] - this.plate.z, this.plate.top, col, i);
    } else {
      for (let i = 0; i < n * 3; i++) col[i] = c.rgb[i] / 255;
    }
    c.pts.geometry.setAttribute('color', new THREE.BufferAttribute(col, 3));
  }

  setColour(mode) {
    this.colour = mode;
    this.clouds.forEach((c) => this.paint(c));
  }

  showCloud(on) { this.clouds.forEach((c) => { c.pts.visible = on; }); }

  // the located object (a result from /api/objects), or null to clear
  setResult(r) {
    const key = r ? `${r.object}:${r.t}` : null;
    if (key === this.key) return;
    this.key = key;
    this.result = r;
    for (const o of [...this.boxG.children]) {
      if (o.element) o.element.remove();
      this.boxG.remove(o);
    }
    if (!r) return;
    const b = r.box;
    this.plate = { z: b.z_range[0], top: b.z_range[1] - b.z_range[0] };
    this.clouds.forEach((c) => this.paint(c));

    // translucent body + outline
    const [L, W, H] = b.size_mm.map((v) => v / 1000);
    const body = new THREE.Mesh(new THREE.BoxGeometry(L, W, H),
      new THREE.MeshBasicMaterial({ color: COL.brass, transparent: true, opacity: 0.18, depthWrite: false }));
    body.position.set(...b.centre);
    body.rotation.z = (b.pose.yaw_deg * Math.PI) / 180;
    this.boxG.add(body);
    this.boxG.add(boxLines(b.corners, COL.brass, false));
    for (const inst of (b.instance_corners || []).length > 1 ? b.instance_corners : []) this.boxG.add(boxLines(inst, COL.grey, true));
    if (r.cloud_fit?.corners) this.boxG.add(boxLines(r.cloud_fit.corners, COL.cream, true));

    // long axis arrow on the top face
    const yaw = (b.pose.yaw_deg * Math.PI) / 180;
    const arrow = new THREE.ArrowHelper(new THREE.Vector3(Math.cos(yaw), Math.sin(yaw), 0),
      new THREE.Vector3(b.centre[0], b.centre[1], b.z_range[1] + 0.001), L / 2 + 0.015, COL.cream, 0.01, 0.006);
    arrow.renderOrder = 3;
    this.boxG.add(arrow);

    const el = document.createElement('div');
    el.className = 'label3d';
    el.innerHTML = `<i style="background:#c9a15d"></i>${r.label} <small>${b.size_mm[0].toFixed(0)} × ${b.size_mm[1].toFixed(0)} × ${b.size_mm[2].toFixed(1)} mm</small>`;
    const lab = new CSS2DObject(el);
    lab.position.set(b.centre[0], b.centre[1], b.z_range[1] + 0.03);
    this.boxG.add(lab);
  }
}
