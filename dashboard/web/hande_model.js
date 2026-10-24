// Robotiq Hand-E on the UR3e flange: meshes and joint chain from
// robotiq_hande_description (Apache-2.0, see models/hande/LICENSE). The root
// is the flange (tool0): z along the tool axis.
//   I/O coupler 11 mm, Hand-E body 99 mm (radius 37.5 mm), fingers slide
//   along ±x from the body top, 0..25 mm each (50 mm stroke).

import * as THREE from 'three';
import { ColladaLoader } from './vendor/ColladaLoader.js';

const COUPLER_H = 0.011;
const HANDE_H = 0.099;
export const FINGER_MAX = 0.025;

export class HandEModel {
  constructor() {
    this.root = new THREE.Group();
    this.coupler = new THREE.Group();
    this.body = new THREE.Group();
    this.body.position.z = COUPLER_H;
    this.left = new THREE.Group();
    this.right = new THREE.Group();
    for (const f of [this.left, this.right]) {
      f.position.z = HANDE_H;
      this.body.add(f);
    }
    this.root.add(this.coupler, this.body);
    this.setOpening(FINGER_MAX / 2);
  }

  // per-finger travel from closed, metres
  setOpening(d) {
    const x = Math.min(Math.max(d, 0), FINGER_MAX);
    this.left.position.x = x;
    this.right.position.x = -x;
  }

  load(base = 'models/hande/') {
    const loader = new ColladaLoader();
    const put = (file, parent, yaw = 0) => new Promise((ok, fail) => {
      loader.load(`${base}${file}`, (c) => {
        c.scene.rotation.set(0, 0, yaw);      // the loader turns Z_UP files to Y_UP; we stay z up
        parent.add(c.scene);
        ok();
      }, undefined, fail);
    });
    return Promise.all([put('io_coupler.dae', this.coupler), put('hande.dae', this.body),
      put('finger.dae', this.left), put('finger.dae', this.right, Math.PI)]);
  }
}
