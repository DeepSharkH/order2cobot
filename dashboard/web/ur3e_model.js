// UR3e visual model: the official meshes (Universal_Robots_ROS2_Description,
// BSD-3, see models/ur3e/LICENSE) on the URDF joint chain from
// config/ur3e/default_kinematics.yaml + visual_parameters.yaml. The root is
// the controller's base frame (URDF base_link_inertia), z up.

import * as THREE from 'three';
import { ColladaLoader } from './vendor/ColladaLoader.js';

const PI = Math.PI;
// [joint origin xyz, rpy] parent -> link, then mesh offset [xyz, rpy]
const CHAIN = [
  { mesh: 'base', joint: null, vis: [[0, 0, 0], [0, 0, PI]] },
  { mesh: 'shoulder', joint: [[0, 0, 0.15185], [0, 0, 0]], vis: [[0, 0, 0], [0, 0, PI]] },
  { mesh: 'upperarm', joint: [[0, 0, 0], [PI / 2, 0, 0]], vis: [[0, 0, 0.120], [PI / 2, 0, -PI / 2]] },
  { mesh: 'forearm', joint: [[-0.24355, 0, 0], [0, 0, 0]], vis: [[0, 0, 0.027], [PI / 2, 0, -PI / 2]] },
  { mesh: 'wrist1', joint: [[-0.2132, 0, 0.13105], [0, 0, 0]], vis: [[0, 0, -0.104], [PI / 2, 0, 0]] },
  { mesh: 'wrist2', joint: [[0, -0.08535, 0], [PI / 2, 0, 0]], vis: [[0, 0, -0.08535], [0, 0, 0]] },
  { mesh: 'wrist3', joint: [[0, 0.0921, 0], [PI / 2, PI, PI]], vis: [[0, 0, -0.0921], [PI / 2, 0, 0]] },
];
const FLANGE = [[0, 0, 0], [0, -PI / 2, -PI / 2]];   // wrist_3 -> flange
const TOOL0 = [[0, 0, 0], [PI / 2, 0, PI / 2]];      // flange -> tool0 (= controller flange)

// URDF origin: translate, then R = Rz(yaw) Ry(pitch) Rx(roll)
function origin([xyz, rpy]) {
  const m = new THREE.Matrix4().makeRotationFromEuler(new THREE.Euler(rpy[0], rpy[1], rpy[2], 'ZYX'));
  m.setPosition(xyz[0], xyz[1], xyz[2]);
  return m;
}

export class UR3eModel {
  constructor() {
    this.root = new THREE.Group();
    this.links = [];     // per link: fixed joint origin group -> rotating group
    let parent = this.root;
    for (const l of CHAIN) {
      const fixed = new THREE.Group();
      fixed.matrixAutoUpdate = false;
      if (l.joint) fixed.matrix.copy(origin(l.joint));
      parent.add(fixed);
      const rot = new THREE.Group();
      fixed.add(rot);
      const vis = new THREE.Group();
      vis.matrixAutoUpdate = false;
      vis.matrix.copy(origin(l.vis));
      rot.add(vis);
      this.links.push({ ...l, rot, vis });
      parent = rot;
    }
    this.tool0 = new THREE.Group();
    this.tool0.matrixAutoUpdate = false;
    this.tool0.matrix.copy(origin(FLANGE).multiply(origin(TOOL0)));
    parent.add(this.tool0);
  }

  load(base = 'models/ur3e/') {
    const loader = new ColladaLoader();
    return Promise.all(this.links.map((l) => new Promise((ok, fail) => {
      loader.load(`${base}${l.mesh}.dae`, (c) => {
        c.scene.rotation.set(0, 0, 0);        // the loader turns Z_UP files to Y_UP; we stay z up
        c.scene.updateMatrix();
        l.vis.add(c.scene);
        ok();
      }, undefined, fail);
    })));
  }

  setJoints(q) {
    // revolute joints rotate about their own z (the base link has none)
    for (let i = 0; i < 6; i++) this.links[i + 1].rot.rotation.set(0, 0, q[i]);
  }
}
