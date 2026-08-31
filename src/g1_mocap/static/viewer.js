"use strict";
// 面板的 3D 场景：把 URDF 的关节树搭成 Object3D 嵌套，贴上 STL，再用关节角驱动。
//
// **正运动学在这里算，不在后端**：three.js 每帧本来就要合成矩阵，后端再用 pinocchio
// 算一遍纯属白花钱。所以 /state 只回 29 个关节角加一个根姿态。
//
// 人的骨架用线段画在机器人旁边，同一尺度、同一视角，姿态差异一眼可见。
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";

const canvas = document.getElementById("md-canvas");
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x12151a);

const camera = new THREE.PerspectiveCamera(45, 1, 0.02, 60);
camera.up.set(0, 0, 1);            // ROS 是 Z 上；不转场景，只改相机的「上」
camera.position.set(2.2, -2.2, 1.3);

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.outputColorSpace = THREE.SRGBColorSpace;

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.12;
controls.target.set(0, -0.55, 0.8);

scene.add(new THREE.HemisphereLight(0xb0d4f1, 0x33373d, 1.0));
const keyLight = new THREE.DirectionalLight(0xffffff, 1.15);
keyLight.position.set(2, -3, 3);
scene.add(keyLight);
const grid = new THREE.GridHelper(4, 20, 0x3a4450, 0x252b34);
grid.rotation.x = Math.PI / 2;     // GridHelper 默认躺在 XZ 面，转到 XY
scene.add(grid);

// 机器人固定画在原点，人的骨架挪到旁边。两边同一尺度才好比姿态。
const robotRoot = new THREE.Group();
scene.add(robotRoot);
const humanRoot = new THREE.Group();
humanRoot.position.set(0, -1.1, 0);
scene.add(humanRoot);

const stlLoader = new STLLoader();
const meshMaterial = new THREE.MeshStandardMaterial(
  { color: 0x9fb4c4, metalness: 0.2, roughness: 0.65 });

let links = {};        // link 名 -> Object3D
let joints = [];       // 可动关节，按 URDF 的深度优先序
let jointByName = {};
let humanLine = null;
let humanParents = [];
let pendingMeshes = 0;

export function isReady() { return pendingMeshes === 0 && joints.length > 0; }

function placement(xyz, quat) {
  // 后端已按 URDF 的固定轴 rpy 转好四元数。别在这儿用 THREE.Euler：
  // 它默认是内旋 "XYZ"，和 URDF 的外旋正好反过来。
  return {
    pos: new THREE.Vector3(xyz[0], xyz[1], xyz[2]),
    quat: new THREE.Quaternion(quat[0], quat[1], quat[2], quat[3]),
  };
}

export function buildRobot(model) {
  links = { [model.base]: robotRoot };
  joints = [];
  jointByName = {};
  for (const j of model.joints) {
    const parent = links[j.parent];
    if (!parent) continue;         // 后端按 DFS 序发，正常到不了这里
    const { pos, quat } = placement(j.xyz, j.quat);
    const hinge = new THREE.Object3D();
    hinge.position.copy(pos);
    hinge.quaternion.copy(quat);
    parent.add(hinge);
    const child = new THREE.Object3D();
    hinge.add(child);
    links[j.child] = child;
    if (j.type !== "fixed") {
      const entry = {
        name: j.name, obj: hinge,
        axis: new THREE.Vector3(j.axis[0], j.axis[1], j.axis[2]).normalize(),
        originQuat: quat.clone(),
      };
      joints.push(entry);
      jointByName[j.name] = entry;
    }
  }

  pendingMeshes = 0;
  for (const link of model.links) {
    const host = links[link.name];
    if (!host) continue;
    for (const visual of link.visuals) {
      pendingMeshes++;
      stlLoader.load(visual.url, (geometry) => {
        const mesh = new THREE.Mesh(geometry, meshMaterial);
        mesh.position.set(visual.xyz[0], visual.xyz[1], visual.xyz[2]);
        mesh.quaternion.set(visual.quat[0], visual.quat[1], visual.quat[2], visual.quat[3]);
        // scale 的负号不是笔误，是 URDF 里镜像用的，照抄别动
        mesh.scale.set(visual.scale[0], visual.scale[1], visual.scale[2]);
        host.add(mesh);
        pendingMeshes--;
      }, undefined, () => { pendingMeshes--; });
    }
  }
}

export function buildHuman(parents) {
  humanParents = parents;
  const count = parents.filter((p) => p >= 0).length;
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position",
    new THREE.BufferAttribute(new Float32Array(count * 6), 3));
  humanLine = new THREE.LineSegments(
    geometry, new THREE.LineBasicMaterial({ color: 0x8b96ad }));
  humanRoot.add(humanLine);
}

const tmpQuat = new THREE.Quaternion();

export function applyAngles(names, values) {
  for (let i = 0; i < names.length; i++) {
    const joint = jointByName[names[i]];
    if (!joint) continue;
    tmpQuat.setFromAxisAngle(joint.axis, values[i]);
    joint.obj.quaternion.copy(joint.originQuat).multiply(tmpQuat);
  }
}

export function applyRootQuat(wxyz) {
  // 后端给的是 wxyz，three.js 要 xyzw
  robotRoot.quaternion.set(wxyz[1], wxyz[2], wxyz[3], wxyz[0]);
}

export function applyHuman(points) {
  if (!humanLine || !points) return;
  const array = humanLine.geometry.attributes.position.array;
  // 人和 G1 的绝对位置不同源，按骨盆对齐才好比姿态
  const root = points[0];
  let k = 0;
  for (let i = 1; i < humanParents.length; i++) {
    const parent = humanParents[i];
    if (parent < 0) continue;
    array[k++] = points[parent][0] - root[0];
    array[k++] = points[parent][1] - root[1];
    array[k++] = points[parent][2] - root[2];
    array[k++] = points[i][0] - root[0];
    array[k++] = points[i][1] - root[1];
    array[k++] = points[i][2] - root[2];
  }
  humanLine.geometry.attributes.position.needsUpdate = true;
}

export function setVisible(which, on) {
  (which === "human" ? humanRoot : robotRoot).visible = on;
}

export function resize() {
  const box = canvas.parentElement.getBoundingClientRect();
  renderer.setPixelRatio(window.devicePixelRatio || 1);
  renderer.setSize(box.width, box.height, false);
  camera.aspect = box.width / Math.max(box.height, 1);
  camera.updateProjectionMatrix();
}

export function start() {
  const tick = () => {
    controls.update();
    renderer.render(scene, camera);
    requestAnimationFrame(tick);
  };
  resize();
  tick();
}
