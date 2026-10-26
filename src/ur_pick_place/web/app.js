'use strict';
const $ = id => document.getElementById(id);
const axes = ['x', 'y', 'z', 'rx', 'ry', 'rz'];
const token = document.querySelector('meta[name="control-token"]').content;
let state = null, renameId = null, posesKey = '', initialized = false, requestPending = 0;
let pollFailed = false, toastTimer;
let activeJog = null;
const degrees = value => value * 180 / Math.PI;
const displayPose = pose => pose.slice(0, 3).map(v => v * 1000).concat(pose.slice(3).map(degrees));
const fixed = (value, digits = 2) => Number.isFinite(value) ? value.toFixed(digits) : '—';
function showPage(page, navigate = false) {
  endJog();
  page = page === 'testing' ? 'testing' : 'waypoints';
  $('waypoint-page').hidden = page !== 'waypoints';
  $('testing-page').hidden = page !== 'testing';
  document.querySelectorAll('[data-page]').forEach(link => {
    if (link.dataset.page === page) link.setAttribute('aria-current', 'page');
    else link.removeAttribute('aria-current');
  });
  $('page-eyebrow').textContent = page === 'waypoints' ? 'PAGE 1 · TEACH' : 'PAGE 2 · TEST';
  $('page-title').textContent = page === 'waypoints' ? 'Record waypoints' : 'Joint & force movement';
  $('page-description').textContent = page === 'waypoints'
    ? 'Position the TCP, then record, name and save your robot waypoints.'
    : 'Choose your gripper and waypoints, test a joint move, or run a force-guided cycle.';
  document.title = `${page === 'waypoints' ? 'Record waypoints' : 'Movement tests'} · Robot Bench`;
  if (navigate) {
    if (location.pathname !== `/${page}`) history.pushState({page}, '', `/${page}`);
    $('page-title').focus({preventScroll:true});
    window.scrollTo(0, 0);
  }
}
function pageFromURL() { return location.pathname.replace(/\/$/, '') === '/testing' ? 'testing' : 'waypoints'; }
document.querySelectorAll('[data-page]').forEach(link => {
  link.onclick = event => {
    if (event.button !== 0 || event.ctrlKey || event.metaKey || event.shiftKey || event.altKey) return;
    event.preventDefault(); showPage(link.dataset.page, true);
  };
});
window.addEventListener('popstate', () => showPage(pageFromURL()));
showPage(pageFromURL());
if (location.pathname === '/') history.replaceState({page:'waypoints'}, '', '/waypoints');
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
  const busy = requestPending || !!state?.pending || !!state?.moving || pollFailed || !!activeJog;
  const fresh = connected && state?.feedback && state.feedback.age_s < .5;
  const safe = fresh && [1, 2].includes(state.feedback.safety_mode);
  document.querySelectorAll('.motion').forEach(button => { button.disabled = !safe || (!!busy && button !== activeJog?.button); });
  $('position-speed').disabled = !!busy;
  $('rotation-speed').disabled = !!busy;
  $('connect').disabled = connected || busy;
  $('disconnect').disabled = !connected || requestPending || !!state?.pending;
  $('robot-ip').disabled = connected || busy;
  $('record').disabled = !fresh || busy;
  document.querySelectorAll('.pose-update').forEach(button => { button.disabled = !fresh || !!busy; });
  updateRecordMode();
  $('copy-current').disabled = !fresh;
  $('stop').disabled = !connected;
  $('save-settings').disabled = connected || busy;
  $('settings-form').querySelectorAll('input').forEach(input => { input.disabled = connected || busy; });
  $('joint-test-pose').disabled = !!busy;
  $('joint-test-move').disabled = !safe || !!busy || !$('joint-test-pose').value;
  updateSequenceControls(connected, busy, safe);
}
function loadTarget(pose) { displayPose(pose).forEach((v, i) => { $(`target-${axes[i]}`).value = v.toFixed(3); }); }
function matchingPose() {
  const name = $('pose-name').value.trim().toLowerCase();
  return state?.poses.find(pose => pose.name.toLowerCase() === name);
}
function updateRecordMode() {
  const existing = matchingPose();
  $('record').textContent = existing ? 'Update saved waypoint' : '＋ Record current pose';
  $('record-mode').textContent = existing
    ? `Replace “${existing.name}” with the current TCP pose and joint angles.`
    : 'Enter an existing name to update that waypoint, or a new name to create one.';
}
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
    const update = document.createElement('button'); update.className = 'pose-update'; update.textContent = 'Update from current';
    update.setAttribute('aria-label', `Update ${pose.name} from current robot position`);
    update.onclick = () => command('/api/poses', {id:pose.id}, `Updated ${pose.name}`);
    actions.append(load, tcp, joint, update); card.append(head, coordinates, date, actions); $('saved-poses').append(card);
  }
}
function render(data) {
  state = data;
  if (activeJog && (!data.connected || data.error || ![1, 2].includes(data.feedback?.safety_mode))) endJog();
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
    $('test-live-position').textContent = values.slice(0, 3).map(v => fixed(v)).join(' / ');
    $('test-live-rotation').textContent = values.slice(3).map(v => fixed(v)).join(' / ');
    $('test-live-joints').textContent = feedback.joints_rad.map(v => fixed(degrees(v), 1)).join(' / ');
    if (!$('target-x').value) loadTarget(feedback.tcp_pose_m_rad);
  } else {
    axes.forEach(axis => { $(`live-${axis}`).textContent = '—'; }); $('safety').textContent = 'Safety —'; $('feedback-age').textContent = 'No feedback'; $('joints').textContent = 'Connect to read joints';
    ['test-live-position', 'test-live-rotation', 'test-live-joints'].forEach(id => { $(id).textContent = '—'; });
  }
  if (!initialized) {
    const settings = data.settings;
    $('position-speed').value = Math.min(20, settings.tcp_speed_mm_s);
    $('robot-ip').value = settings.robot_ip;
    $('tcp-speed').value = settings.tcp_speed_mm_s;
    $('joint-speed').value = settings.joint_speed_rad_s;
    ['x', 'y', 'z'].forEach((axis, i) => { $(`min-${axis}`).value = settings.workspace_min_m[i] * 1000; $(`max-${axis}`).value = settings.workspace_max_m[i] * 1000; });
    initialized = true;
  }
  updateJogSpeeds();
  if (data.error) showError(data.error);
  renderPoses(data.poses); renderSequence(data); updateControls();
}
async function poll() {
  try {
    const response = await fetch('/api/state', {cache: 'no-store'});
    if (!response.ok) throw new Error('Backend is unavailable');
    const data = await response.json();
    if (!data.sequence) throw new Error('Restart the Robot Bench backend, then refresh this page to load the sequence controls.');
    if (pollFailed) showError('');
    pollFailed = false; render(data);
  } catch (error) {
    endJog();
    pollFailed = true; showError(error.message.startsWith('Restart the Robot Bench') ? error.message : 'Connection to the local backend was lost. Movement controls are disabled.'); updateControls();
  }
}
$('connect').onclick = () => command('/api/connect', {robot_ip: $('robot-ip').value});
$('disconnect').onclick = () => command('/api/disconnect');
$('stop').onclick = () => { endJog(); command('/api/stop'); };
$('joint-test-pose').onchange = updateControls;
$('joint-test-move').onclick = () => command('/api/move-saved', {id:$('joint-test-pose').value, mode:'joint'});
$('copy-current').onclick = () => { if (state?.feedback) loadTarget(state.feedback.tcp_pose_m_rad); };
function endJog() {
  const jog = activeJog;
  if (!jog) return;
  requestPending += 1;
  activeJog = null;
  clearInterval(jog.timer);
  jog.button.classList.remove('jog-active');
  // Stop immediately, then once more after start is acknowledged, so a quick
  // release cannot race a delayed start request.
  post('/api/stop').catch(error => showError(error.message));
  jog.start.finally(() => post('/api/stop').catch(error => showError(error.message)))
    .finally(() => { requestPending -= 1; updateControls(); });
  updateControls();
}
function beginJog(button, key = null) {
  if (button.disabled || activeJog || requestPending) return;
  const axis = button.dataset.axis;
  const speed = Number($(axis.startsWith('r') ? 'rotation-speed' : 'position-speed').value);
  const limit = axis.startsWith('r') ? 10 : Math.min(100, state?.settings?.tcp_speed_mm_s ?? 20);
  if (!Number.isFinite(speed) || speed <= 0 || speed > limit) return showError(`Choose a jog speed greater than 0 and at most ${limit}.`);
  const jog = {button, key, id: crypto.randomUUID()};
  activeJog = jog;
  button.classList.add('jog-active');
  showError('');
  jog.start = post('/api/jog/start', {axis, speed: speed * Number(button.dataset.sign), jog_id: jog.id})
    .catch(error => { showError(error.message); if (activeJog === jog) endJog(); });
  jog.timer = setInterval(() => {
    post('/api/heartbeat', {jog_id: jog.id}).catch(() => { if (activeJog === jog) endJog(); });
  }, 150);
  updateControls();
}
function updateJogSpeeds() {
  const limit = Math.min(100, state?.settings?.tcp_speed_mm_s ?? 20);
  $('position-speed').max = limit;
  $('position-speed').value = Math.min(Number($('position-speed').value), limit);
  $('position-speed-value').textContent = `${Number($('position-speed').value).toFixed(1)} mm/s`;
  $('position-speed-limit').textContent = `${limit} mm/s`;
  $('rotation-speed-value').textContent = `${Number($('rotation-speed').value).toFixed(1)} deg/s`;
}
$('position-speed').addEventListener('input', updateJogSpeeds);
$('rotation-speed').addEventListener('input', updateJogSpeeds);
document.querySelectorAll('[data-axis]').forEach(button => {
  button.addEventListener('pointerdown', event => {
    if (event.button !== 0 || activeJog) return;
    event.preventDefault();
    button.focus();
    button.setPointerCapture(event.pointerId);
    beginJog(button);
  });
  for (const name of ['pointerup', 'pointercancel', 'lostpointercapture']) button.addEventListener(name, endJog);
  button.addEventListener('keydown', event => {
    if (![' ', 'Enter'].includes(event.key)) return;
    event.preventDefault();
    if (!event.repeat) beginJog(button, event.key);
  });
  button.addEventListener('keyup', event => {
    if (event.key === activeJog?.key) { event.preventDefault(); endJog(); }
  });
  button.addEventListener('blur', endJog);
  button.addEventListener('contextmenu', event => { event.preventDefault(); endJog(); });
});
window.addEventListener('pointerup', endJog);
window.addEventListener('pointercancel', endJog);
window.addEventListener('blur', endJog);
window.addEventListener('pagehide', endJog);
document.addEventListener('visibilitychange', () => { if (document.hidden) endJog(); });
document.addEventListener('keydown', event => { if (event.key === 'Escape') endJog(); });
$('target-form').onsubmit = event => { event.preventDefault(); const pose = axes.map(axis => Number($(`target-${axis}`).value)); if (pose.every(Number.isFinite)) command('/api/move-tcp', {pose_mm_deg: pose}); };
$('pose-name').addEventListener('input', updateRecordMode);
$('record-form').onsubmit = async event => {
  event.preventDefault();
  const existing = matchingPose();
  await command('/api/poses', existing ? {id:existing.id} : {name:$('pose-name').value}, existing ? `Updated ${existing.name}` : 'Pose recorded');
};
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

// Sequence controls use the same connection and worker as manual teaching.
let sequenceInitialized = false, sequencePoseKey = '', sequencePreviewId = null, previewPayloadKey = null;
function sequenceNumber(id) {
  const raw = $(id).value;
  if (raw === '' || !Number.isFinite(Number(raw))) throw new Error('Fill all active sequence settings with finite numbers.');
  return Number(raw);
}
function invalidateSequence() {
  sequencePreviewId = previewPayloadKey = null;
  $('sequence-summary').classList.add('hidden');
  $('sequence-status').textContent = 'Settings changed · preview again';
  updateControls();
}
function sequencePayload() {
  const defaults = state.sequence.defaults;
  const cameraText = $('seq-camera-pose').value.trim();
  const kind = $('sequence-gripper').value;
  const gripper = {...defaults.gripper, kind, settle_s: sequenceNumber('seq-gripper-settle')};
  // Save both drivers' settings, including currently hidden controls, so
  // switching the selected driver does not erase the other configuration.
  const optional = id => $(id).value === '' ? null : sequenceNumber(id);
  gripper.open_width_mm = optional('seq-open-width') ?? defaults.gripper.open_width_mm;
  gripper.grip_width_mm = optional('seq-grip-width') ?? defaults.gripper.grip_width_mm;
  gripper.grip_force_n = optional('seq-grip-force') ?? defaults.gripper.grip_force_n;
  gripper.serial_port = $('seq-serial-port').value.trim() || null;
  gripper.grip_command = $('seq-grip-command').value;
  gripper.grip_min_raw = [optional('seq-grip-min-1'), optional('seq-grip-min-2')];
  gripper.release_max_raw = [optional('seq-release-max-1'), optional('seq-release-max-2')];
  if (kind !== defaults.gripper.kind) gripper.timeout_s = kind === 'arduino' ? 2 : 8;
  function probe(role) {
    return {...defaults[`${role}_probe`], contact_force_n: sequenceNumber(`seq-${role}-contact`),
      command_force_n: sequenceNumber(`seq-${role}-command`), max_descent_mm: sequenceNumber(`seq-${role}-descent`),
      speed_mm_s: sequenceNumber(`seq-${role}-speed`)};
  }
  return {home_id: $('sequence-home').value, pick_id: $('sequence-pick').value, place_id: $('sequence-place').value,
    camera_pose: cameraText ? JSON.parse(cameraText) : null,
    options: {gripper, pick_approach_mm: sequenceNumber('seq-pick-approach'), place_approach_mm: sequenceNumber('seq-place-approach'),
      pick_backoff_mm: sequenceNumber('seq-backoff'), lift_mm: sequenceNumber('seq-lift'), retract_mm: sequenceNumber('seq-retract'),
      grasp_offset_mm: ['x', 'y', 'z'].map(axis => sequenceNumber(`seq-offset-${axis}`)),
      pick_probe: probe('pick'), place_probe: probe('place')}};
}
function initializeSequence(data) {
  const saved = data.sequence.selection || {};
  const base = data.sequence.defaults;
  const config = {...base, ...saved.options};
  const gripper = {...base.gripper, ...config.gripper};
  $('sequence-gripper').value = gripper.kind;
  const values = {'seq-open-width': gripper.open_width_mm, 'seq-grip-width': gripper.grip_width_mm,
    'seq-grip-force': gripper.grip_force_n, 'seq-gripper-settle': gripper.settle_s,
    'seq-serial-port': gripper.serial_port || '/dev/ttyUSB0', 'seq-grip-command': gripper.grip_command,
    'seq-grip-min-1': gripper.grip_min_raw[0], 'seq-grip-min-2': gripper.grip_min_raw[1],
    'seq-release-max-1': gripper.release_max_raw[0], 'seq-release-max-2': gripper.release_max_raw[1],
    'seq-pick-approach': config.pick_approach_mm, 'seq-place-approach': config.place_approach_mm,
    'seq-backoff': config.pick_backoff_mm, 'seq-lift': config.lift_mm, 'seq-retract': config.retract_mm};
  for (const [id, value] of Object.entries(values)) $(id).value = value ?? '';
  ['x', 'y', 'z'].forEach((axis, i) => { $(`seq-offset-${axis}`).value = config.grasp_offset_mm[i]; });
  for (const role of ['pick', 'place']) {
    const probe = {...base[`${role}_probe`], ...config[`${role}_probe`]};
    for (const [field, key] of Object.entries({contact:'contact_force_n', command:'command_force_n', descent:'max_descent_mm', speed:'speed_mm_s'})) {
      $(`seq-${role}-${field}`).value = probe[key];
    }
  }
  if (saved.camera_pose) $('seq-camera-pose').value = JSON.stringify(saved.camera_pose);
  sequenceInitialized = true;
}
function renderSequence(data) {
  if (!sequenceInitialized) initializeSequence(data);
  const seq = data.sequence;
  const key = JSON.stringify(data.poses.map(p => [p.id, p.name]));
  if (key !== sequencePoseKey) {
    sequencePoseKey = key;
    const preferred = {home:['home_a', 'home', 'demo_home'], pick:['pick', 'positionA', 'demo_pick'], place:['drop', 'place', 'demo_place']};
    for (const role of ['home', 'pick', 'place']) {
      const select = $(`sequence-${role}`);
      const selected = select.value || seq.selection?.[`${role}_id`];
      select.replaceChildren(new Option('Choose a saved pose…', ''));
      for (const pose of data.poses) select.add(new Option(pose.name, pose.id));
      if (data.poses.some(p => p.id === selected)) select.value = selected;
      else {
        const match = preferred[role].map(name => data.poses.find(p => p.name === name)).find(Boolean);
        if (match) select.value = match.id;
      }
    }
    const jointSelect = $('joint-test-pose');
    const jointSelected = jointSelect.value || $('sequence-home').value;
    jointSelect.replaceChildren(new Option('Choose a saved pose…', ''));
    for (const pose of data.poses) jointSelect.add(new Option(pose.name, pose.id));
    if (data.poses.some(p => p.id === jointSelected)) jointSelect.value = jointSelected;
  }
  $('demo-poses').classList.toggle('hidden', !data.demo);
  if (!seq.preview) sequencePreviewId = null;
  $('sequence-status').textContent = seq.status === 'previewed' && !sequencePreviewId ? 'Preview again to run this cycle' : seq.message;
  const index = seq.steps.findIndex(step => step[0] === seq.step);
  document.querySelectorAll('#sequence-steps li').forEach((item, i) => {
    item.classList.toggle('done', seq.status === 'completed' || i < index);
    item.classList.toggle('active', seq.running && i === index);
    item.classList.toggle('stopped', ['stopped', 'failed'].includes(seq.status) && i === index);
  });
}
function updateSequenceControls(connected, busy, safe) {
  const kind = $('sequence-gripper').value;
  $('rg2-fields').classList.toggle('hidden', kind !== 'rg2');
  $('arduino-fields').classList.toggle('hidden', kind !== 'arduino');
  $('sequence-form').querySelectorAll('input, select, textarea').forEach(input => { input.disabled = !!busy; });
  $(kind === 'rg2' ? 'arduino-fields' : 'rg2-fields').querySelectorAll('input, select').forEach(input => { input.disabled = true; });
  if ($('seq-camera-pose').value.trim()) $('sequence-pick').disabled = true;
  $('sequence-preview').disabled = !!busy;
  $('demo-poses').disabled = !!busy;
  $('sequence-run').disabled = !safe || !!busy || !sequencePreviewId || state?.sequence.preview?.id !== sequencePreviewId;
}
$('sequence-form').addEventListener('input', invalidateSequence);
$('sequence-form').addEventListener('change', invalidateSequence);
$('sequence-form').onsubmit = async event => {
  event.preventDefault();
  let payload;
  try { payload = sequencePayload(); } catch (error) { return showError(error.message); }
  requestPending += 1; updateControls(); showError('');
  try {
    const preview = await post('/api/sequence/preview', payload);
    sequencePreviewId = preview.id; previewPayloadKey = JSON.stringify(payload);
    $('sequence-summary').textContent = `${preview.demo ? 'DEMO' : preview.robot_ip} · ${preview.gripper === 'rg2' ? 'OnRobot RG2' : 'Arduino gripper'} · one cycle\n${preview.names.home} → ${preview.names.pick} → ${preview.names.home} → ${preview.names.place}\nGrasp offset: [${preview.grasp_offset_mm.join(', ')}] mm · pick/place contact: ${preview.pick_force_n}/${preview.place_force_n} N · transit: ${preview.tcp_speed_mm_s} mm/s\nPreview checks the workspace envelope; whole robot paths must have clearance.`;
    $('sequence-summary').classList.remove('hidden');
    await poll();
  } catch (error) { sequencePreviewId = null; showError(error.message); }
  finally { requestPending -= 1; updateControls(); }
};
$('sequence-run').onclick = async () => {
  try {
    if (!sequencePreviewId || JSON.stringify(sequencePayload()) !== previewPayloadKey) throw new Error('Settings changed. Preview again before running.');
    const id = sequencePreviewId;
    sequencePreviewId = null;
    await command('/api/sequence/run', {preview_id:id});
  } catch (error) { showError(error.message); }
};
$('demo-poses').onclick = async () => {
  try {
    const ids = await post('/api/sequence/demo-poses');
    await poll();
    for (const role of ['home', 'pick', 'place']) $(`sequence-${role}`).value = ids[`${role}_id`];
    invalidateSequence(); toast('Demo home, pick and place poses loaded');
  } catch (error) { showError(error.message); }
};
$('seq-camera-file').onchange = async event => {
  try {
    const file = event.target.files[0];
    if (!file) return;
    if (file.size > 32768) throw new Error('Target JSON file is too large.');
    $('seq-camera-pose').value = JSON.stringify(JSON.parse(await file.text()));
    invalidateSequence();
  } catch (error) { showError(error.message); }
};
