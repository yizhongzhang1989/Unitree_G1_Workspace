"use strict";
// 双臂监控的 3D 场景。只做三件事：搭关节树、贴 mesh、放两对末端标记。
//
// **正运动学在这里算，不在后端**：URDF 的关节树搭成 Object3D 嵌套之后，
// three.js 每帧本来就要合成矩阵，后端再用 pinocchio 算一遍纯属白花钱。
// 于是 /api/state 不回整棵 link 变换树，只回关节角、低层电机状态和 4 个位姿。
//
// 渲染是**按需**的：没有新数据、相机也没动就不调 renderer.render()，
// 页面挂着不看时基本不占 GPU/CPU。
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";

const canvas = document.getElementById("viewer");
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0f1419);

const camera = new THREE.PerspectiveCamera(50, 1, 0.01, 50);
camera.up.set(0, 0, 1);                 // ROS 是 Z 上；不转场景，只改相机的“上”
camera.position.set(0.75, -0.75, 0.35);

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.outputColorSpace = THREE.SRGBColorSpace;

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.12;

scene.add(new THREE.HemisphereLight(0xb0d4f1, 0x33373d, 1.0));
const key = new THREE.DirectionalLight(0xffffff, 1.1);
key.position.set(2, -3, 3);
scene.add(key);
const grid = new THREE.GridHelper(1.6, 16, 0x3a4450, 0x272d35);
grid.rotation.x = Math.PI / 2;          // GridHelper 默认躺在 XZ 面，转到 XY
grid.position.z = -0.55;                // 大致到腰下，纯粹当地面参考
scene.add(grid);

// base_frame（torso_link）就是模型根，末端位姿也是相对它的，所以标记直接挂这里。
const base = new THREE.Group();
scene.add(base);
base.add(new THREE.AxesHelper(0.12));

const stlLoader = new STLLoader();
const meshMat = new THREE.MeshStandardMaterial(
  { color: 0x9fb4c4, metalness: 0.2, roughness: 0.65 });

const SIDES = ["left", "right"];
const show = { mesh: true, command: true, limited: true, measured: true };

let links = {};        // link 名 -> Object3D
let joints = [];       // 可动关节，按 URDF 的 DFS 序
let meshes = [];       // 已加载的 mesh，用于开关与定视角
let needsRender = true;
let fitted = false;

function invalidate() { needsRender = true; }

// ---- 末端标记 --------------------------------------------------------------
function marker(color, shape) {
  const group = new THREE.Group();
  group.visible = false;
  group.userData.known = false;
  const geometry = shape === "sphere" ? new THREE.SphereGeometry(0.012, 16, 12)
    : shape === "diamond" ? new THREE.OctahedronGeometry(0.016)
      : new THREE.TorusGeometry(0.016, 0.003, 8, 20);
  group.add(new THREE.Mesh(geometry, new THREE.MeshBasicMaterial({ color })));
  group.add(new THREE.AxesHelper(0.07));
  return group;
}

const markers = {
  command: { left: marker(0x39d353, "sphere"), right: marker(0x39d353, "sphere") },
  limited: { left: marker(0xf2c94c, "diamond"), right: marker(0xf2c94c, "diamond") },
  measured: { left: marker(0xf0a45c, "ring"), right: marker(0xf0a45c, "ring") },
};
for (const kind of ["command", "limited"])
  for (const side of SIDES) base.add(markers[kind][side]);

// ---- 建模 ------------------------------------------------------------------

function placement(xyz, quat) {
  // 后端已经按 URDF 的固定轴 rpy 转好四元数了。别在这儿用 THREE.Euler 转：
  // 它默认是内旋 "XYZ"，和 URDF 的外旋正好反过来（见 dashboard_node.rpy_to_quat）。
  return {
    pos: new THREE.Vector3(xyz[0], xyz[1], xyz[2]),
    quat: new THREE.Quaternion(quat[0], quat[1], quat[2], quat[3]),
  };
}

export function setModel(model) {
  links = { [model.base]: base };
  joints = [];
  for (const j of model.joints) {
    const parent = links[j.parent];
    if (!parent) continue;              // 后端按 DFS 序发，正常到不了这里
    const { pos, quat } = placement(j.xyz, j.quat);
    const obj = new THREE.Object3D();
    obj.position.copy(pos);
    obj.quaternion.copy(quat);
    parent.add(obj);
    const child = new THREE.Object3D();
    obj.add(child);
    links[j.child] = child;
    if (j.type !== "fixed") {
      joints.push({
        name: j.name, type: j.type, obj, mimic: j.mimic || null, limit: j.limit || null,
        axis: new THREE.Vector3(j.axis[0], j.axis[1], j.axis[2]).normalize(),
        originPos: pos.clone(), originQuat: quat.clone(),
      });
    }
  }
  // 实测环挂在由 /joint_states 驱动的末端 link 上，因此天然和模型严丝合缝。
  for (const side of SIDES) {
    const tip = links[`${side}_gripper_base`];
    if (tip) {
      tip.add(markers.measured[side]);
      markers.measured[side].visible = show.measured;
    }
  }

  for (const link of model.links) {
    const parent = links[link.name];
    if (!parent) continue;
    for (const v of link.visuals) {
      const { pos, quat } = placement(v.xyz, v.quat);
      stlLoader.load(v.url, (geometry) => {
        geometry.computeVertexNormals();
        const mesh = new THREE.Mesh(geometry, meshMat);
        mesh.position.copy(pos);
        mesh.quaternion.copy(quat);
        mesh.scale.set(v.scale[0], v.scale[1], v.scale[2]);
        mesh.visible = show.mesh;
        parent.add(mesh);
        meshes.push(mesh);
        fitted = false;                 // mesh 陆续到齐，包围盒会变
        invalidate();
      });
    }
  }
  invalidate();
  return { links: model.links.length, joints: joints.length };
}

// ---- 每次轮询更新 -----------------------------------------------------------

const _q = new THREE.Quaternion();
const _v = new THREE.Vector3();
// 复用：每次轮询都新建一个 80 键的对象是白给 GC 找事。关节按 DFS 序遍历，
// 每个键都先写后读，不会读到上一帧的值。
const resolved = {};

export function setJoints(values) {
  if (!joints.length || !values || !Object.keys(values).length) {
    for (const side of SIDES) markers.measured[side].visible = false;
    return {};
  }
  // 单遍解算：URDF 里 mimic 的源（eccentric）永远排在从动关节之前，
  // 而后端是按 DFS 序发的，所以一遍就够，不需要拓扑排序。
  for (const j of joints) {
    let value = values[j.name];
    if (value === undefined && j.mimic) {
      const source = resolved[j.mimic.joint];
      if (source !== undefined) value = j.mimic.multiplier * source + j.mimic.offset;
    }
    if (value === undefined) value = 0;
    // 必须裁到限位：夹爪的 spline mimic 链是分段线性拟合，每段只在自己的区间里
    // 有效，不裁的话中段会算出区间外的值，滑块和连杆会飞出手掌。
    if (j.limit) value = Math.min(j.limit[1], Math.max(j.limit[0], value));
    resolved[j.name] = value;
    if (j.type === "prismatic") {
      j.obj.position.copy(j.originPos).add(
        _v.copy(j.axis).applyQuaternion(j.originQuat).multiplyScalar(value));
    } else {
      j.obj.quaternion.copy(j.originQuat).multiply(_q.setFromAxisAngle(j.axis, value));
    }
  }
  base.updateMatrixWorld(true);
  const measured = {};
  for (const side of SIDES) {
    const tip = links[`${side}_gripper_base`];
    if (!tip) continue;
    const position = tip.getWorldPosition(new THREE.Vector3());
    const quaternion = tip.getWorldQuaternion(new THREE.Quaternion());
    measured[side] = [position.x, position.y, position.z,
      quaternion.x, quaternion.y, quaternion.z, quaternion.w];
    markers.measured[side].userData.known = true;
    markers.measured[side].visible = show.measured;
  }
  invalidate();
  return measured;
}

export function setMarkers(kind, poses) {
  for (const side of SIDES) {
    const group = markers[kind][side];
    const p = poses[side];
    group.userData.known = Array.isArray(p) && p.length === 7;
    if (group.userData.known) {
      group.position.set(p[0], p[1], p[2]);
      group.quaternion.set(p[3], p[4], p[5], p[6]);
    }
    group.visible = group.userData.known && show[kind];
  }
  invalidate();
}

export function setVisible(what, on) {
  show[what] = on;
  if (what === "mesh") meshes.forEach((m) => { m.visible = on; });
  else if (what === "measured") SIDES.forEach((side) => {
    markers.measured[side].visible = on && markers.measured[side].userData.known;
  });
  else SIDES.forEach((side) => {
    const group = markers[what][side];
    group.visible = on && group.userData.known;
  });
  invalidate();
}

// ---- 视角与渲染 -------------------------------------------------------------

export function fit() {
  const box = new THREE.Box3().setFromObject(base);
  if (box.isEmpty()) return;
  const size = box.getSize(new THREE.Vector3()).length() || 1;
  const center = box.getCenter(new THREE.Vector3());
  controls.target.copy(center);
  camera.position.copy(center).add(
    new THREE.Vector3(0.75, -0.75, 0.35).setLength(size * 0.9));
  camera.near = size / 200;
  camera.far = size * 20;
  camera.updateProjectionMatrix();
  controls.update();
  fitted = true;
  invalidate();
}

function resize() {
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (!w || !h) return;
  renderer.setPixelRatio(devicePixelRatio);
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
  invalidate();
}
// 只在事件里改尺寸。放到渲染循环里等于每帧读一次 clientWidth，而那会强制重排。
addEventListener("resize", resize);
resize();

let drawn = 0, rate = 0, mark = performance.now();
export function drawRate() { return rate; }

(function animate() {
  requestAnimationFrame(animate);
  const moving = controls.update();     // 阻尼收敛之前会一直返回 true
  const now = performance.now();
  if (now - mark >= 1000) { rate = drawn; drawn = 0; mark = now; }
  if (!moving && !needsRender) return;
  if (!fitted && meshes.length) fit();  // mesh 全到齐前先不定视角
  renderer.render(scene, camera);
  needsRender = false;
  drawn++;
})();
