'use strict';
const $ = id => document.getElementById(id);
const token = document.querySelector('meta[name="teach-token"]').content;
let defaults, state, dirty = false, posting = false, previousResult = '', imageVersion = 0;
const steps = ['configure', 'capture', 'teach', 'locate'];
let activeStep = 0;
const stepNames = ['Set up', 'Capture marker', 'Teach pick', 'Read & export'];
const guidance = [
  ['Prepare your setup', 'Use the calibration for this camera and its unchanged wrist mounting. Confirm the gripper TCP against the pendant.'],
  ['Frame the whole marker', 'Keep the camera and board still during capture. Review the detected axes and quality before continuing.'],
  ['Keep the marker fixed', 'Only move the robot. Teach the final pick position and orientation, then stop before saving.'],
  ['Return to the capture pose', 'Use the saved observation joints. The marker can move within your configured limits. Re-read for every pick.']
];
function availableStep() {
  const f=state?.project?.flow||{};
  return dirty||!state?.project ? 0 : f.taught ? 3 : f.captured ? 2 : 1;
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
    nav.classList.toggle('failed',state.failed===step&&!!state.error);
    $('nav-state-'+step).textContent=light.textContent.replace('● ','');
  });
  $('step-counter').textContent=`Step ${activeStep+1} of 4`;
  $('back').disabled=busy||activeStep===0;
  $('next').hidden=activeStep===3;
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
function fill(s) {
  for (const id of ['project','calibration','robot_ip','motion_model','approach_mm']) $(id).value=s[id];
  $('max_shift_mm').value=s.limits?.max_shift_mm??30; $('max_rotation_deg').value=s.limits?.max_rotation_deg??15;
  $('tcp').value=s.tcp.join(', '); $('kind').value=s.target.kind;
  if(s.target.kind==='charuco') { $('manifest').value=s.target.board_manifest; $('board').value=s.target.board; }
  else { for(const id of ['dictionary','marker_id','marker_length_mm']) $(id).value=s.target[id]; }
  targetFields();
}
function settings() {
  const kind=$('kind').value;
  return {project:$('project').value,calibration:$('calibration').value,robot_ip:$('robot_ip').value,
    motion_model:$('motion_model').value,approach_mm:Number($('approach_mm').value),
    tcp:$('tcp').value.trim().split(/[\s,]+/).map(Number),tcp_confirmed:$('tcp_confirmed').checked,
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
function render() {
  if(!state) return;
  $('version').textContent='v'+state.version;
  const p=state.project, f=p?.flow||{}, busy=!!state.busy||posting;
  const ready={configure:!!p,capture:!!f.captured,teach:!!f.taught,locate:!!f.exported};
  for(const step of steps) {
    const light=$('light-'+step); let status=dirty?'pending':ready[step]?'ready':'pending';
    if(state.busy===step) status='running';
    if(state.error&&state.failed===step) status='failed';
    light.className='light '+status; light.textContent='● '+({pending:'Pending',ready:'Ready',running:'Running',failed:'Failed'}[status]);
  }
  $('configure').disabled=busy; $('open').disabled=busy;
  for(const input of $('setup').querySelectorAll('input,select')) input.disabled=busy;
  $('capture').disabled=busy||dirty||!p;
  $('teach').disabled=busy||dirty||!f.captured||!$('marker_unchanged').checked;
  $('locate').disabled=busy||dirty||!f.taught;
  $('marker_unchanged').disabled=busy||!f.captured;
  $('export-result').hidden=dirty||!f.exported; $('saved-links').hidden=dirty||!f.taught;
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
  posting=true; render();
  try {
    await request('/api/'+action,body); dirty=false;
    if(action==='capture') $('marker_unchanged').checked=false;
    message(action==='configure'?'Setup ready. Continue to step 2.':'Working — keep the robot stationary.');
    await poll();
    if(action==='configure'&&!state.error) { activeStep=availableStep(); restoreCapture(); }
  } catch(e) { message(e.message,true); }
  finally { posting=false; render(); }
}
async function poll() {
  try {
    state=await request('/api/state');
    const resultKey=JSON.stringify([state.completed,state.failed,state.error]);
    if(!state.busy && resultKey!==previousResult && !dirty) {
      if(state.error) message(state.error,true);
      else if(state.completed) message({configure:'Setup ready. Continue to step 2.',capture:'Marker captured. Keep it fixed and teach the pick endpoint in step 3.',teach:'Picking position saved. Return to the saved observation joints for step 4.',locate:'Fresh poses exported. Review them before using your picking program.'}[state.completed]);
      if(['capture','locate'].includes(state.completed)||['capture','locate'].includes(state.failed)) {
        $('capture-image').src='/api/image?v='+(++imageVersion); $('capture-image').hidden=false;
      }
    }
    previousResult=resultKey; render();
  } catch(e) { message('Teacher connection lost: '+e.message,true); $('capture').disabled=$('teach').disabled=$('locate').disabled=true; }
}
steps.forEach((step,index)=>$('nav-'+step).addEventListener('click',()=>showStep(index)));
$('back').addEventListener('click',()=>showStep(activeStep-1));
$('next').addEventListener('click',()=>showStep(activeStep+1));
$('setup').addEventListener('submit',e=>{e.preventDefault();post('configure',settings());});
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
$('calibration').addEventListener('change',()=>{const item=defaults.calibrations.find(c=>c.path===$('calibration').value);if(item){$('tcp').value=item.tcp.join(', ');$('robot_ip').value=item.robot_ip;}});
$('kind').addEventListener('change',targetFields);
$('marker_unchanged').addEventListener('change',render);
$('capture').addEventListener('click',()=>post('capture',{}));
$('teach').addEventListener('click',()=>post('teach',{marker_unchanged:$('marker_unchanged').checked}));
$('locate').addEventListener('click',()=>post('locate',{}));
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
  } catch(e){message(e.message,true);}
})();
