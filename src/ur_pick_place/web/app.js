'use strict';
const $ = id => document.getElementById(id);
const axes = ['x', 'y', 'z', 'rx', 'ry', 'rz'];
const token = document.querySelector('meta[name="control-token"]').content;
let state = null, renameId = null, posesKey = '', initialized = false, requestPending = 0;
let pollFailed = false, toastTimer;
const degrees = value => value * 180 / Math.PI;
const displayPose = pose => pose.slice(0, 3).map(v => v * 1000).concat(pose.slice(3).map(degrees));
const fixed = (value, digits = 2) => Number.isFinite(value) ? value.toFixed(digits) : '—';
function showError(message) { $('error').textContent = message; $('error').classList.toggle('hidden', !message); }
function toast(message) { $('toast').textContent = message; $('toast').classList.remove('hidden'); clearTimeout(toastTimer); toastTimer = setTimeout(() => $('toast').classList.add('hidden'), 2600); }
async function post(path, body = {}) {
  const response = await fetch(path, {method: 'POST', headers: {'Content-Type': 'application/json', 'X-Control-Token': token}, body: JSON.stringify(body)});
  const data = await response.json();
  if (!response.ok) throw new Error(data.error || 'Request failed');
  return data;
}
async function command(path, body = {}, success) {
  if (requestPending && path !== '/api/stop') return;
  requestPending += 1; updateControls(); showError('');
  try { await post(path, body); if (success) toast(success); await poll(); }
  catch (error) { showError(error.message); }
  finally { requestPending -= 1; updateControls(); }
}
function updateControls() {
  const connected = !!state?.connected;
  const busy = requestPending || !!state?.pending || !!state?.moving || pollFailed;
  const fresh = connected && state?.feedback && state.feedback.age_s < .5;
  const safe = fresh && [1, 2].includes(state.feedback.safety_mode);
  document.querySelectorAll('.motion').forEach(button => { button.disabled = !safe || busy; });
  $('connect').disabled = connected || busy;
  $('disconnect').disabled = !connected || requestPending || !!state?.pending;
  $('robot-ip').disabled = connected || busy;
  $('record').disabled = !fresh || busy;
  $('copy-current').disabled = !fresh;
  $('stop').disabled = !connected;
  $('save-settings').disabled = connected || busy;
  $('settings-form').querySelectorAll('input').forEach(input => { input.disabled = connected || busy; });
}
function loadTarget(pose) { displayPose(pose).forEach((v, i) => { $(`target-${axes[i]}`).value = v.toFixed(3); }); }
function renderPoses(poses) {
  const key = JSON.stringify(poses);
  if (key === posesKey) return;
  posesKey = key;
  $('pose-count').textContent = poses.length;
  if (!poses.length) return;
  $('saved-poses').replaceChildren();
  for (const pose of poses) {
    const card = document.createElement('article'); card.className = 'saved-card';
    const head = document.createElement('div'); head.className = 'saved-card-head';
    const name = document.createElement('h4'); name.textContent = pose.name;
    const rename = document.createElement('button'); rename.className = 'text-button'; rename.textContent = 'Rename'; rename.setAttribute('aria-label', `Rename ${pose.name}`);
    rename.onclick = () => { renameId = pose.id; $('rename-name').value = pose.name; $('rename-dialog').showModal(); $('rename-name').focus(); };
    head.append(name, rename);
    const coordinates = document.createElement('div'); coordinates.className = 'pose-coordinates';
    const values = displayPose(pose.tcp_pose_m_rad);
    const xyz = document.createElement('div'); xyz.textContent = `XYZ  ${values.slice(0, 3).map(v => fixed(v, 1)).join(' / ')} mm`;
    const rotation = document.createElement('div'); rotation.textContent = `Rvec ${values.slice(3).map(v => fixed(v, 1)).join(' / ')} deg`;
    coordinates.append(xyz, rotation);
    const date = document.createElement('time'); date.textContent = new Date(pose.recorded_at).toLocaleString(); date.dateTime = pose.recorded_at;
    const actions = document.createElement('div'); actions.className = 'pose-actions';
    const load = document.createElement('button'); load.textContent = 'Load target'; load.onclick = () => { loadTarget(pose.tcp_pose_m_rad); toast(`Loaded ${pose.name} into TCP target`); };
    const tcp = document.createElement('button'); tcp.className = 'motion'; tcp.textContent = 'Move TCP'; tcp.onclick = () => command('/api/move-saved', {id: pose.id, mode: 'tcp'});
    const joint = document.createElement('button'); joint.className = 'motion'; joint.textContent = 'Move joints'; joint.onclick = () => command('/api/move-saved', {id: pose.id, mode: 'joint'});
    actions.append(load, tcp, joint); card.append(head, coordinates, date, actions); $('saved-poses').append(card);
  }
}
function render(data) {
  state = data;
  $('demo-banner').classList.toggle('hidden', !data.demo);
  $('mode').textContent = data.demo ? 'DEMO' : 'LOCAL · RTDE'; $('mode').classList.toggle('demo', data.demo);
  $('connection').classList.toggle('online', data.connected);
  $('connection').replaceChildren(Object.assign(document.createElement('i'), {}), document.createTextNode(data.connected ? (data.moving ? 'Moving' : 'Connected') : 'Disconnected'));
  $('activity').textContent = data.pending === 'connect' ? 'Connecting to RTDE…' : data.message;
  const feedback = data.feedback;
  if (feedback) {
    const values = displayPose(feedback.tcp_pose_m_rad);
    values.forEach((value, i) => { $(`live-${axes[i]}`).textContent = fixed(value); });
    const safety = {1: 'Normal', 2: 'Reduced', 3: 'Protective stop', 5: 'Safeguard stop', 7: 'Emergency stop'};
    $('safety').textContent = `Safety · ${safety[feedback.safety_mode] || `mode ${feedback.safety_mode}`}`;
    $('feedback-age').textContent = `Feedback ${Math.round(feedback.age_s * 1000)} ms`;
    $('joints').replaceChildren(...feedback.joints_rad.map((v, i) => { const span = document.createElement('span'); span.textContent = `J${i + 1} ${fixed(degrees(v), 1)}°`; return span; }));
    if (!$('target-x').value) loadTarget(feedback.tcp_pose_m_rad);
  } else {
    axes.forEach(axis => { $(`live-${axis}`).textContent = '—'; }); $('safety').textContent = 'Safety —'; $('feedback-age').textContent = 'No feedback'; $('joints').textContent = 'Connect to read joints';
  }
  if (!initialized) {
    const settings = data.settings;
    $('robot-ip').value = settings.robot_ip;
    $('tcp-speed').value = settings.tcp_speed_mm_s;
    $('joint-speed').value = settings.joint_speed_rad_s;
    ['x', 'y', 'z'].forEach((axis, i) => { $(`min-${axis}`).value = settings.workspace_min_m[i] * 1000; $(`max-${axis}`).value = settings.workspace_max_m[i] * 1000; });
    initialized = true;
  }
  if (data.error) showError(data.error);
  renderPoses(data.poses); updateControls();
}
async function poll() {
  try {
    const response = await fetch('/api/state', {cache: 'no-store'});
    if (!response.ok) throw new Error('Backend is unavailable');
    const data = await response.json();
    if (pollFailed) showError('');
    pollFailed = false; render(data);
  } catch (error) {
    pollFailed = true; showError('Connection to the local backend was lost. Movement controls are disabled.'); updateControls();
  }
}
$('connect').onclick = () => command('/api/connect', {robot_ip: $('robot-ip').value});
$('disconnect').onclick = () => command('/api/disconnect');
$('stop').onclick = () => command('/api/stop');
$('copy-current').onclick = () => { if (state?.feedback) loadTarget(state.feedback.tcp_pose_m_rad); };
document.querySelectorAll('[data-axis]').forEach(button => {
  button.onclick = () => {
    const axis = button.dataset.axis;
    const size = Number($(axis.startsWith('r') ? 'rotation-step' : 'position-step').value);
    if (!Number.isFinite(size) || size <= 0 || size > 10) return showError('Choose an increment greater than 0 and at most 10.');
    command('/api/jog', {axis, step: size * Number(button.dataset.sign)});
  };
});
$('target-form').onsubmit = event => { event.preventDefault(); const pose = axes.map(axis => Number($(`target-${axis}`).value)); if (pose.every(Number.isFinite)) command('/api/move-tcp', {pose_mm_deg: pose}); };
$('record-form').onsubmit = async event => { event.preventDefault(); await command('/api/poses', {name: $('pose-name').value}, 'Pose recorded'); };
$('rename-form').onsubmit = async event => {
  event.preventDefault();
  try { await post('/api/poses/rename', {id: renameId, name: $('rename-name').value}); $('rename-dialog').close(); await poll(); toast('Pose renamed'); }
  catch (error) { $('rename-dialog').close(); showError(error.message); }
};
$('cancel-rename').onclick = () => $('rename-dialog').close();
$('settings-form').onsubmit = async event => {
  event.preventDefault();
  const settings = {...state.settings, robot_ip: $('robot-ip').value,
    tcp_speed_mm_s: Number($('tcp-speed').value), joint_speed_rad_s: Number($('joint-speed').value),
    workspace_min_m: ['x', 'y', 'z'].map(axis => Number($(`min-${axis}`).value) / 1000),
    workspace_max_m: ['x', 'y', 'z'].map(axis => Number($(`max-${axis}`).value) / 1000)};
  await command('/api/settings', settings, 'Settings saved');
};
setInterval(poll, 250);
setInterval(() => post('/api/heartbeat').catch(() => {}), 500);
post('/api/heartbeat').catch(() => {});
poll();
