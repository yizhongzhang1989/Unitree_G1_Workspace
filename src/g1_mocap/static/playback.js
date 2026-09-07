import * as THREE from 'three';
import * as viewer from './viewer.js';

export function createPlayback(updateJoints, say) {
  const element = id => document.getElementById(id);
  const list = element('md-recordings');
  const timeline = element('md-timeline');
  const transport = element('md-transport');
  const play = element('md-play');
  const title = element('md-playing-title');
  let entries = [], signature = '', motion = null, selected = '', requestId = 0;
  let active = false, playing = false, position = 0, previousTime = null;
  const rootQuat = new THREE.Quaternion(), nextQuat = new THREE.Quaternion();
  const angles = new Array(29), root = new Array(3);
  let playIcon = '';

  function updateControls() {
    const icon = playing ? 'pause' : 'play';
    if (icon !== playIcon) {
      playIcon = icon;
      play.innerHTML = `<i data-lucide="${icon}"></i>`;
      lucide.createIcons();
    }
    play.title = play.ariaLabel = playing ? '暂停' : '播放';
    timeline.value = String(position);
    const duration = motion ? (motion.num_frames - 1) / motion.fps : 0;
    element('md-play-time').textContent = `${position.toFixed(2)} / ${duration.toFixed(2)} s`;
  }

  function renderFrame() {
    if (!motion) return;
    const offset = Math.min(motion.num_frames - 1, position * motion.fps);
    const index = Math.floor(offset), alpha = offset - index;
    const first = motion.frames[index], second = motion.frames[Math.min(index + 1, motion.num_frames - 1)];
    for (let axis = 0; axis < 3; axis++) root[axis] = THREE.MathUtils.lerp(first[axis], second[axis], alpha);
    for (let joint = 0; joint < 29; joint++) angles[joint] = THREE.MathUtils.lerp(first[joint + 7], second[joint + 7], alpha);
    rootQuat.fromArray(first, 3).slerp(nextQuat.fromArray(second, 3), alpha);
    viewer.applyRootPosition(root);
    viewer.applyRootQuat([rootQuat.w, rootQuat.x, rootQuat.y, rootQuat.z]);
    viewer.applyAngles(motion.joint_names, angles);
    updateJoints(motion.joint_names, angles);
    updateControls();
  }

  function markSelection() {
    for (const button of list.querySelectorAll('button')) {
      const current = button.dataset.name === selected;
      button.classList.toggle('md-selected', current);
      button.setAttribute('aria-pressed', String(current));
    }
  }

  function showList() {
    const query = element('md-motion-search').value.toLowerCase();
    list.replaceChildren();
    for (const entry of entries.filter(item => item.file_name.toLowerCase().includes(query))) {
      const button = document.createElement('button');
      button.type = 'button';
      button.className = 'md-motion-item';
      button.dataset.name = entry.file_name;
      const name = document.createElement('span');
      name.textContent = entry.file_name;
      const detail = document.createElement('small');
      detail.textContent = `${new Date(entry.modified_ns / 1e6).toLocaleString()} · ${(entry.size_bytes / 1024).toFixed(0)} KB`;
      button.append(name, detail);
      button.addEventListener('click', () => load(entry.file_name));
      list.appendChild(button);
    }
    if (!list.childElementCount) list.textContent = entries.length ? '无匹配动作' : '暂无录制';
    element('md-motion-count').textContent = String(entries.length);
    markSelection();
  }

  async function refresh() {
    try {
      const response = await fetch('/motions', {cache: 'no-store'});
      const data = await response.json();
      if (!response.ok) throw new Error(data.error || '读取列表失败');
      element('md-motion-directory').textContent = data.directory;
      const nextSignature = JSON.stringify(data.motions);
      if (nextSignature !== signature) {
        signature = nextSignature;
        entries = data.motions;
        showList();
      }
    } catch (error) { say(error.message, 'err'); }
  }

  async function load(name) {
    const ticket = ++requestId;
    active = true;
    playing = false;
    motion = null;
    selected = name;
    markSelection();
    transport.hidden = false;
    title.textContent = name;
    play.disabled = timeline.disabled = true;
    element('md-play-time').textContent = '加载中…';
    element('md-do-cal').disabled = true;
    element('md-show-human').disabled = true;
    viewer.setVisible('human', false);
    try {
      const response = await fetch('/motion?name=' + encodeURIComponent(name), {cache: 'no-store'});
      const data = await response.json();
      if (ticket !== requestId) return;
      if (!response.ok) throw new Error(data.error || '读取动作失败');
      motion = data;
      position = 0;
      timeline.max = String((motion.num_frames - 1) / motion.fps);
      play.disabled = timeline.disabled = false;
      previousTime = null;
      playing = true;
      element('md-show-robot').checked = true;
      viewer.setVisible('robot', true);
      viewer.frameMotion(motion.frames);
      renderFrame();
      say(`${name} · ${motion.num_frames} 帧 · ${motion.fps} Hz`, 'good');
    } catch (error) {
      if (ticket === requestId) {
        element('md-play-time').textContent = '加载失败';
        say(error.message, 'err');
      }
    }
  }

  element('md-live').addEventListener('click', () => {
    requestId++;
    active = playing = false;
    motion = null;
    selected = '';
    transport.hidden = true;
    element('md-do-cal').disabled = false;
    element('md-show-human').disabled = false;
    viewer.setVisible('human', element('md-show-human').checked);
    viewer.resetFraming();
    markSelection();
    say('实时预览', 'good');
  });
  play.addEventListener('click', () => {
    if (!motion) return;
    if (position >= Number(timeline.max)) position = 0;
    playing = !playing;
    previousTime = null;
    renderFrame();
  });
  timeline.addEventListener('input', () => { position = Number(timeline.value); previousTime = null; renderFrame(); });
  element('md-motion-search').addEventListener('input', showList);
  element('md-refresh').addEventListener('click', refresh);
  window.addEventListener('resize', () => { if (motion) viewer.frameMotion(motion.frames); });
  document.addEventListener('visibilitychange', () => { previousTime = null; });

  function tick(now) {
    if (active && motion && playing && !document.hidden) {
      if (previousTime !== null) position += (now - previousTime) / 1000 * Number(element('md-speed').value);
      const end = Number(timeline.max);
      if (position >= end) {
        if (element('md-loop').checked) position %= end;
        else { position = end; playing = false; }
      }
      renderFrame();
    }
    previousTime = now;
    requestAnimationFrame(tick);
  }
  requestAnimationFrame(tick);
  refresh();
  setInterval(refresh, 5000);
  return {isActive: () => active};
}