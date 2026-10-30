'use strict';
const $ = id => document.getElementById(id);
const token = document.querySelector('meta[name="teach-token"]').content;
let defaults, state, dirty = false, posting = false, previousResult = '', imageVersion = 0;
let connectionOK = true, validationImageId = '';
const steps = ['configure', 'capture', 'teach', 'locate', 'validate'];
let activeStep = 0;
const stepNames = ['Set up', 'Capture marker', 'Teach pick', 'Read & export', 'Validation'];
const guidance = [
  ['Prepare your setup', 'Use the calibration for this camera and its unchanged wrist mounting. Confirm the gripper TCP against the pendant.'],
  ['Frame the whole marker', 'Keep the camera and board still during capture. Review the detected axes and quality before continuing.'],
  ['Keep the marker fixed', 'Only move the robot. Teach the final pick position and orientation, then stop before saving.'],
  ['Return to the capture pose', 'Use the saved observation joints. The marker can move within your configured limits. Re-read for every pick.'],
  ['Test your taught relationship', 'Capture a repositioned marker, review the new target, then explicitly test a slow approach move. Repeat with different positions and rotations.']
];
function availableStep() {
  const f=state?.project?.flow||{};
  return dirty||!state?.project ? 0 : f.exported ? 4 : f.taught ? 3 : f.captured ? 2 : 1;
}
function showStep(index) {
  if(state?.busy||posting||index<0||index>availableStep()) return;
  activeStep=index; render();
  $('title-'+steps[index]).focus({preventScroll:true});
}
function renderWizard(ready,busy) {
  if(activeStep>availableStep()) activeStep=availableStep();
  steps.forEach((step,index)=>{
    $('step-'+step).hidden=index!==activeStep;
    const nav=$('nav-'+step), light=$('light-'+step);
    nav.disabled=busy||index>availableStep();
    nav.setAttribute('aria-current',index===activeStep?'step':'false');
    nav.classList.toggle('done',!dirty&&ready[step]);
    nav.classList.toggle('failed',failedStep()===step&&!!state.error||step==='validate'&&state.project?.validation?.visual_result==='misaligned');
    $('nav-state-'+step).textContent=light.textContent.replace('● ','');
  });
  $('step-counter').textContent=`Step ${activeStep+1} of ${steps.length}`;
  $('back').disabled=busy||activeStep===0;
  $('next').hidden=activeStep===steps.length-1;
  $('next').disabled=busy||activeStep>=availableStep();
  $('next').textContent='Next: '+(stepNames[activeStep+1]||'')+' →';
  $('guide-title').textContent=guidance[activeStep][0];
  $('guide-text').textContent=guidance[activeStep][1];
  $('setup-details').hidden=!state.project||dirty;
  $('camera-placeholder').hidden=!$('capture-image').hidden;
}
function message(text, error=false) { $('message').textContent=text; $('message').classList.toggle('error',error); }
function targetFields() {
  for (const kind of ['charuco','aruco']) document.querySelectorAll('.'+kind).forEach(el=>el.hidden=$('kind').value!==kind);
}
function tcpText(tcp) {
  return tcp.map((v,i)=>Number((i<3?v*1000:v).toPrecision(12))).join(', ');
}
function showCalibrationTCP() {
  const calibration=defaults.calibrations.find(c=>c.path===$('calibration').value);
  $('calibration-tcp').textContent=calibration
    ? 'Calibration TCP (XYZ mm, rotation-vector rad): '+tcpText(calibration.tcp)
    : 'This TCP must match the TCP recorded in your selected calibration.';
}
function fill(s) {
  for (const id of ['project','calibration','robot_ip','motion_model','approach_mm']) $(id).value=s[id];
  $('max_shift_mm').value=s.limits?.max_shift_mm??30; $('max_rotation_deg').value=s.limits?.max_rotation_deg??15;
  $('tcp').value=tcpText(s.tcp); $('kind').value=s.target.kind;
  if(s.target.kind==='charuco') { $('manifest').value=s.target.board_manifest; $('board').value=s.target.board; }
  else { for(const id of ['dictionary','marker_id','marker_length_mm']) $(id).value=s.target[id]; }
  targetFields(); showCalibrationTCP();
}
function settings() {
  const kind=$('kind').value;
  const tcp=$('tcp').value.trim().split(/[\s,]+/).map((v,i)=>i<3?Number(v)/1000:Number(v));
  if(tcp.length!==6||!tcp.every(Number.isFinite))
    throw new Error('Active gripper TCP needs six numbers: X, Y, Z in mm, then RX, RY, RZ in rotation-vector radians.');
  return {project:$('project').value,calibration:$('calibration').value,robot_ip:$('robot_ip').value,
    motion_model:$('motion_model').value,approach_mm:Number($('approach_mm').value),
    tcp,tcp_confirmed:$('tcp_confirmed').checked,
    limits:{max_shift_mm:Number($('max_shift_mm').value),max_rotation_deg:Number($('max_rotation_deg').value),max_tilt_change_deg:10,max_height_change_mm:10,max_tool_tilt_deg:20},
    target:kind==='charuco'?{kind,board_manifest:$('manifest').value,board:$('board').value}:
      {kind,dictionary:$('dictionary').value,marker_id:Number($('marker_id').value),marker_length_mm:Number($('marker_length_mm').value)}};
}
async function request(path,body) {
  const response=await fetch(path,body===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json','X-Teach-Token':token},body:JSON.stringify(body)});
  const value=await response.json(); if(!response.ok) throw new Error(value.error||'Request failed'); return value;
}
function restoreCapture() {
  if(state?.project?.flow.captured) {
    $('capture-image').src='/api/image?v='+(++imageVersion);
    $('capture-image').hidden=false;
  }
}
function poseText(pose) { return pose ? '['+pose.map(v=>v.toFixed(6)).join(', ')+']' : ''; }
function failedStep() { return ['capture_validation','validate_move'].includes(state?.failed)?'validate':state?.failed; }
function validationPose(pose) { return pose ? pose.map((v,i)=>`${['X','Y','Z','RX','RY','RZ'][i]}: ${(i<3?v*1000:v).toFixed(i<3?2:5)} ${i<3?'mm':'rad'}`).join('\n') : ''; }
function renderValidation(p,busy) {
  const v=dirty?null:p?.validation;
  const fresh=v?.status==='captured'&&Date.now()/1000-v.timestamp>=0&&Date.now()/1000-v.timestamp<=60;
  $('validation-capture').disabled=busy||dirty||!connectionOK||!p?.flow.taught;
  $('validation-move').disabled=busy||!connectionOK||!fresh||!$('validation-confirmed').checked;
  $('validation-confirmed').disabled=busy||!fresh;
  for(const id of ['validation-speed','validation-workspace-min','validation-workspace-max']) $(id).disabled=busy;
  $('validation-preview').hidden=!v;
  $('validation-result').hidden=!v||v.status==='captured'||v.status==='moving';
  $('validation-aligned').disabled=$('validation-misaligned').disabled=busy||v?.status!=='completed';
  $('validation-observe-joints').textContent=p?.observation?'Observation joints (degrees): '+p.observe_joints.map(q=>(q*180/Math.PI).toFixed(2)).join(', '):'';
  if(v) {
    $('validation-marker-pose').textContent=validationPose(v.marker_tcp_pose_m_rad);
    $('validation-pick-pose').textContent=validationPose(v.pick_tcp_pose_m_rad);
    $('validation-approach-pose').textContent=validationPose(v.approach_tcp_pose_m_rad);
    const r=v.validation;
    $('validation-change').textContent=`Pick shift: ${r.shift_from_taught_mm.toFixed(2)} mm\nPick rotation: ${r.rotation_from_taught_deg.toFixed(2)}°\nMarker yaw: ${r.marker_yaw_change_deg.toFixed(2)}°`;
    $('validation-quality').textContent=`${v.quality.frames_accepted} accepted frames · fit ${v.quality.reprojection_rmse_px.toFixed(3)} px`;
    $('validation-freshness').textContent=v.status==='captured'?(fresh?`Ready for one attempt · ${Math.max(0,Math.floor(60-(Date.now()/1000-v.timestamp)))} seconds remaining`:'Capture expired. Capture again before motion.'):'Capture used · capture again for another motion test.';
    $('validation-motion-result').textContent=v.status+(v.message?' · '+v.message:'')+(v.error?`\nTCP error: ${v.error.position_mm.toFixed(2)} mm / ${v.error.rotation_deg.toFixed(3)}°`:'')+(v.visual_result?'\nVisual alignment: '+v.visual_result:'');
    $('validation-actual-pose').textContent=v.actual_tcp_pose_m_rad?'Measured TCP:\n'+validationPose(v.actual_tcp_pose_m_rad):'';
    if(validationImageId!==v.id) {
      validationImageId=v.id; $('validation-confirmed').checked=false;
      $('validation-move').disabled=true;
      $('validation-image').src='/api/validation/image?v='+encodeURIComponent(v.id);
      $('validation-image').hidden=false;
    }
  } else {
    $('validation-image').hidden=true; $('validation-confirmed').checked=false; validationImageId='';
  }
  $('validation-placeholder').hidden=!$('validation-image').hidden;
  $('validation-history').textContent=(!dirty&&p?.validation_history?.length)?p.validation_history.slice().reverse().map((t,i)=>`${p.validation_history.length-i}. ${new Date(t.timestamp*1000).toLocaleString()} · ${t.status}${t.error?` · ${t.error.position_mm.toFixed(2)} mm / ${t.error.rotation_deg.toFixed(3)}°`:''} · visual: ${t.visual_result||'unreviewed'}${t.message?'\n'+t.message:''}`).join('\n'):'No motion tests yet.';
}
function render() {
  if(!state) return;
  $('version').textContent='v'+state.version;
  const p=state.project, f=p?.flow||{}, busy=!!state.busy||posting;
  const ready={configure:!!p,capture:!!f.captured,teach:!!f.taught,locate:!!f.exported,validate:p?.validation?.status==='completed'&&p.validation.visual_result==='aligned'};
  for(const step of steps) {
    const light=$('light-'+step); let status=dirty?'pending':ready[step]?'ready':'pending';
    if(state.busy===step||step==='validate'&&['capture_validation','validate_move'].includes(state.busy)) status='running';
    if(state.error&&failedStep()===step) status='failed';
    if(step==='validate'&&p?.validation?.visual_result==='misaligned') status='failed';
    light.className='light '+status; light.textContent='● '+({pending:'Pending',ready:'Ready',running:'Running',failed:'Failed'}[status]);
  }
  $('configure').disabled=busy; $('open').disabled=busy;
  for(const input of $('setup').querySelectorAll('input,select')) input.disabled=busy;
  $('capture').disabled=busy||dirty||!p;
  $('teach').disabled=busy||dirty||!f.captured||!$('marker_unchanged').checked;
  $('locate').disabled=busy||dirty||!f.taught;
  $('marker_unchanged').disabled=busy||!f.captured;
  $('export-result').hidden=dirty||!f.exported; $('saved-links').hidden=dirty||!f.taught;
  renderValidation(p,busy);
  renderWizard(ready,busy);
  if(p&&!dirty) {
    $('movement-limits').textContent=`Configured limits: ${p.settings.limits.max_shift_mm} mm target shift and ${p.settings.limits.max_rotation_deg}° rotation.`;
    $('setup-result').textContent='Calibration '+p.calibration.verdict.toUpperCase()+' · '+p.target+'\nCamera serial: '+(p.calibration.camera_serial||'not recorded')+'\n'+p.project;
    const o=p.observation;
    $('capture-result').textContent=o?`${o.quality.frames_accepted} accepted frames · ${o.quality.corner_count} corners · fit ${o.quality.reprojection_rmse_px.toFixed(3)} px` : '';
    $('observe-joints').textContent=o?'Saved observation joints (degrees): '+p.observe_joints.map(q=>(q*180/Math.PI).toFixed(2)).join(', '):'Saved observation joints appear after capture.';
    $('teach-result').textContent=f.taught?'Saved gripper TCP endpoint (base m/rad):\n'+poseText(p.taught.taught_tcp_in_base)+'\nMarker-relative relationship saved in grasp_pick.json.':'';
    $('storage').textContent='Project files: '+p.project;
    if(p.exported) { $('pick-pose').textContent=poseText(p.exported.pick_tcp_pose_m_rad); $('approach-pose').textContent=poseText(p.exported.approach_tcp_pose_m_rad); }
  }
}
async function post(action,body) {
  posting=true;
  try {
    previousResult='';
    render();
    await request('/api/'+action,body); dirty=false;
    if(action==='capture') $('marker_unchanged').checked=false;
    message(action==='configure'?'Setup ready. Continue to step 2.':action==='validation/move'?'Validation motion requested. Keep this tab active; Stop motion is available above.':'Working — keep the robot stationary.');
    await poll();
    if(action==='configure'&&!state.error) { activeStep=availableStep(); restoreCapture(); }
  } catch(e) { message(e.message,true); }
  finally { posting=false; render(); }
}
async function poll() {
  try {
    state=await request('/api/state');
    connectionOK=true;
    const resultKey=JSON.stringify([state.completed,state.failed,state.error]);
    if(!state.busy && resultKey!==previousResult && !dirty) {
      if(state.error) message(state.error,true);
      else if(state.completed) message({configure:'Setup ready. Continue to step 2.',capture:'Marker captured. Keep it fixed and teach the pick endpoint in step 3.',teach:'Picking position saved. Return to the saved observation joints for step 4.',locate:'Fresh poses exported. Continue to step 5 to validate with an approach move.',capture_validation:'Validation capture ready. Review the target and clear path before motion.',validate_move:'Approach motion completed. Check visual alignment and record the result.'}[state.completed]);
      if(['capture','locate'].includes(state.completed)||['capture','locate'].includes(state.failed)) {
        $('capture-image').src='/api/image?v='+(++imageVersion); $('capture-image').hidden=false;
      }
    }
    previousResult=resultKey; render();
  } catch(e) { connectionOK=false; render(); message('Teacher connection lost: '+e.message,true); $('capture').disabled=$('teach').disabled=$('locate').disabled=true; }
}
steps.forEach((step,index)=>$('nav-'+step).addEventListener('click',()=>showStep(index)));
$('back').addEventListener('click',()=>showStep(activeStep-1));
$('next').addEventListener('click',()=>showStep(activeStep+1));
$('setup').addEventListener('submit',async e=>{
  e.preventDefault();
  try {
    // Handle validation here so a field inside closed advanced settings cannot
    // silently prevent the submit event and leave the user without feedback.
    const invalid=$('setup').querySelector('input:invalid, select:invalid');
    if(invalid) {
      const details=invalid.closest('details'); if(details) details.open=true;
      const label=invalid.labels?.[0]?.firstChild?.textContent.trim()
        ||invalid.labels?.[0]?.textContent.trim()||invalid.id;
      activeStep=0; render();
      message('Check '+label+': '+invalid.validationMessage,true);
      invalid.focus(); invalid.reportValidity(); return;
    }
    await post('configure',settings());
  } catch(e) { message(e.message,true); }
});
$('open').addEventListener('click',async()=>{
  try {
    const saved=await request('/api/open',{project:$('project').value});
    fill(saved); dirty=true; activeStep=0; $('tcp_confirmed').checked=false;
    message('Saved project loaded. Check its active TCP on the pendant, then complete setup to continue.'); render();
  } catch(e) { message(e.message,true); }
});
$('setup').addEventListener('input',()=>{dirty=true;activeStep=0;targetFields();$('tcp_confirmed').checked=false;message('Setup changed. Check the TCP and complete step 1 again.');render();});
// Checking the declaration itself must not immediately uncheck it.
$('tcp_confirmed').addEventListener('input',e=>{e.stopPropagation();render();});
$('calibration').addEventListener('change',()=>{const item=defaults.calibrations.find(c=>c.path===$('calibration').value);if(item){$('tcp').value=tcpText(item.tcp);$('robot_ip').value=item.robot_ip;}showCalibrationTCP();});
$('kind').addEventListener('change',targetFields);
$('marker_unchanged').addEventListener('change',render);
$('capture').addEventListener('click',()=>post('capture',{}));
$('teach').addEventListener('click',()=>post('teach',{marker_unchanged:$('marker_unchanged').checked}));
$('locate').addEventListener('click',()=>post('locate',{}));
$('validation-capture').addEventListener('click',()=>{ $('validation-confirmed').checked=false; post('validation/capture',{}); });
$('validation-confirmed').addEventListener('change',render);
for(const id of ['validation-speed','validation-workspace-min','validation-workspace-max']) $(id).addEventListener('input',()=>{$('validation-confirmed').checked=false;render();});
$('validation-move').addEventListener('click',async()=>{
  try {
    function workspace(id) {
      const text=$(id).value.trim(), values=text.split(/[\s,]+/).map(Number);
      if(!text||values.length!==3||!values.every(Number.isFinite)) throw new Error('Workspace needs three XYZ numbers in millimetres.');
      return values.map(v=>v/1000);
    }
    const body={capture_id:state.project.validation.id,path_confirmed:$('validation-confirmed').checked,
      speed_mm_s:Number($('validation-speed').value),workspace_min_m:workspace('validation-workspace-min'),workspace_max_m:workspace('validation-workspace-max')};
    await request('/api/validation/heartbeat',{});
    $('validation-confirmed').checked=false;
    await post('validation/move',body);
  } catch(e) { message(e.message,true); }
});
async function stopValidation() {
  $('validation-confirmed').checked=false;
  try { await request('/api/validation/stop',{}); await poll(); message('Stop requested. Capture again before another motion attempt.'); }
  catch(e) { message('Stop request could not reach the teacher. Use the pendant stop: '+e.message,true); }
}
$('validation-stop').addEventListener('click',stopValidation);
for(const [id,result] of [['validation-aligned','aligned'],['validation-misaligned','misaligned']]) $(id).addEventListener('click',async()=>{
  try { await request('/api/validation/review',{capture_id:state.project.validation.id,result}); await poll(); message(result==='aligned'?'Alignment recorded. Reposition and capture again to test another pose.':'Alignment needs adjustment recorded. Review teaching and calibration before another test.'); }
  catch(e) { message(e.message,true); }
});
$('validation-image').addEventListener('error',()=>{$('validation-image').hidden=true;$('validation-placeholder').hidden=false;});
document.addEventListener('visibilitychange',()=>{if(document.hidden&&state?.busy==='validate_move') stopValidation();});
$('capture-image').addEventListener('error',()=>{$('capture-image').hidden=true;render();});
(async()=>{
  try {
    defaults=await request('/api/defaults');
    for(const [id,items] of [['calibrations',defaults.calibrations.map(c=>c.path)],['manifests',defaults.manifests]])
      for(const item of items){const option=document.createElement('option');option.value=item;$(id).append(option);}
    await poll();
    fill(state.project?{...state.project.settings,project:state.project.project}:defaults.settings);
    activeStep=availableStep();
    restoreCapture();
    render();
    setInterval(poll,700);
    setInterval(()=>{if(!document.hidden&&state?.busy==='validate_move') request('/api/validation/heartbeat',{}).catch(()=>{});},300);
  } catch(e){message(e.message,true);}
})();
