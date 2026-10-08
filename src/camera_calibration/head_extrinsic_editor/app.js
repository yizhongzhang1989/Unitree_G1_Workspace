const $ = (id) => document.getElementById(id);
let hasPhoto = false, timer = 0, request = 0;

function message(text, bad = false) {
  $('status').textContent = text;
  $('status').classList.toggle('bad', bad);
}

function showPhoto(photo) {
  hasPhoto = true;
  $('empty').hidden = true;
  $('preview').hidden = false;
  $('capture-meta').textContent = `${photo.samples} 帧 · 最大变化 ${photo.movement_rad} rad`;
  refreshPreview();
}

function values() {
  return Object.fromEntries(['x', 'y', 'z', 'roll', 'pitch', 'yaw']
    .map((id) => [id, $(id).value.trim()]));
}

function schedulePreview() {
  clearTimeout(timer);
  timer = setTimeout(refreshPreview, 120);
}

function refreshPreview() {
  if (!hasPhoto) return;
  const transform = values();
  if (Object.values(transform).some((value) => !value)) {
    message('XYZ/RPY 必须都有值', true);
    return;
  }
  const serial = Date.now();
  request = serial;
  const params = new URLSearchParams({...transform, _: serial});
  const image = new Image();
  document.body.classList.add('busy');
  message('正在重渲染轮廓…');
  image.onload = () => {
    if (serial !== request) return;
    $('preview').src = image.src;
    document.body.classList.remove('busy');
    message('修改 XYZ/RPY 会立即刷新当前照片的轮廓');
  };
  image.onerror = async () => {
    if (serial !== request) return;
    document.body.classList.remove('busy');
    try {
      const error = await fetch(`/api/preview?${params}`).then((response) => response.json());
      message(error.error || '渲染失败', true);
    } catch (_) {
      message('渲染失败', true);
    }
  };
  image.src = `/api/preview?${params}`;
}

async function capture() {
  $('capture').disabled = true;
  message('正在拍照并采集静止关节角…');
  try {
    const response = await fetch('/api/capture', {method: 'POST'});
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || '拍照失败');
    showPhoto(result);
  } catch (error) {
    message(error.message, true);
  } finally {
    $('capture').disabled = false;
  }
}

['x', 'y', 'z', 'roll', 'pitch', 'yaw'].forEach(
  (id) => $(id).addEventListener('input', schedulePreview));
$('capture').onclick = capture;

fetch('/api/state').then(async (response) => {
  const state = await response.json();
  if (!response.ok) throw new Error(state.error || '读取失败');
  ['x', 'y', 'z'].forEach((id, index) => $(id).value = state.xyz[index]);
  ['roll', 'pitch', 'yaw'].forEach((id, index) => $(id).value = state.rpy[index]);
  message('尚无照片，点击“拍新照片”');
}).catch((error) => message(error.message, true));