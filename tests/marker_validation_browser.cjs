// Real Chrome + real HTTP and child-process vision workers, isolated fake hardware.
const assert = require('node:assert/strict');
const {spawn} = require('node:child_process');
const fs = require('node:fs');
const path = require('node:path');
const os = require('node:os');
const readline = require('node:readline');
const {once} = require('node:events');

const delay = ms => new Promise(resolve => setTimeout(resolve,ms));
const directory = fs.mkdtempSync(path.join(os.tmpdir(),'marker-validation-chrome-'));
const fixture = spawn(process.env.PYTHON || path.join(__dirname,'../.venv/bin/python'),[path.join(__dirname,'marker_validation_fixture.py')]);
const chrome = spawn(process.env.CHROME_BINARY || '/usr/bin/google-chrome',[
  '--headless=new','--no-sandbox','--disable-gpu','--disable-dev-shm-usage','--no-first-run',
  '--no-default-browser-check','--disable-background-networking','--remote-debugging-pipe',
  '--user-data-dir='+directory
],{stdio:['ignore','ignore','pipe','pipe','pipe']});
let log='',nextId=0,buffer='',session,target,url;
fixture.stderr.on('data',chunk=>{log+=chunk;});
chrome.stderr.on('data',chunk=>{log+=chunk;});
const pending=new Map(), jsErrors=[];
chrome.stdio[4].on('data',chunk=>{
  buffer+=chunk.toString();
  for(let end;(end=buffer.indexOf('\0'))>=0;){
    const value=JSON.parse(buffer.slice(0,end));buffer=buffer.slice(end+1);
    if(value.id){const pair=pending.get(value.id);pending.delete(value.id);if(pair)value.error?pair[1](new Error(JSON.stringify(value.error))):pair[0](value.result);}
    if(value.method==='Runtime.exceptionThrown')jsErrors.push(value.params.exceptionDetails.text+' '+JSON.stringify(value.params.exceptionDetails.exception));
  }
});
function send(method,params={},sessionId){
  return new Promise((resolve,reject)=>{
    const id=++nextId;pending.set(id,[resolve,reject]);
    chrome.stdio[3].write(JSON.stringify({id,method,params,...(sessionId?{sessionId}:{})})+'\0');
  });
}
async function evaluate(expression){
  const result=await send('Runtime.evaluate',{expression,returnByValue:true,awaitPromise:true,userGesture:true},session);
  if(result.exceptionDetails)throw new Error(JSON.stringify(result.exceptionDetails));
  return result.result.value;
}
async function wait(expression,label,timeout=20000){
  const deadline=Date.now()+timeout;
  while(Date.now()<deadline){if(await evaluate(expression))return;await delay(50);}
  throw new Error('Timed out: '+label+'\n'+await evaluate('document.body.innerText'));
}
async function click(id){
  assert.equal(await evaluate(`(()=>{const e=document.getElementById(${JSON.stringify(id)});if(!e||e.disabled||e.closest('[hidden]'))return false;e.click();return true;})()`),true,'Clickable: '+id);
}
async function api(endpoint,body){
  const options=body===undefined?'{}':`{method:'POST',headers:{'Content-Type':'application/json','X-Teach-Token':document.querySelector('meta[name="teach-token"]').content},body:${JSON.stringify(JSON.stringify(body))}}`;
  return evaluate(`(async()=>{const response=await fetch(${JSON.stringify(endpoint)},${options});return {status:response.status,body:await response.json()};})()`);
}
async function openPage(){
  ({targetId:target}=await send('Target.createTarget',{url:'about:blank'}));
  ({sessionId:session}=await send('Target.attachToTarget',{targetId:target,flatten:true}));
  await send('Page.enable',{},session);await send('Runtime.enable',{},session);
  await send('Emulation.setDeviceMetricsOverride',{width:1280,height:960,deviceScaleFactor:1,mobile:false},session);
  await send('Page.navigate',{url},session);
  await wait('document.getElementById("robot_ip")?.value==="simulated-robot"','page initialization');
}
function errorBetween(a,b){
  const quaternion=r=>{const angle=Math.hypot(...r);return angle<1e-12?[1,0,0,0]:[Math.cos(angle/2),...r.map(x=>x*Math.sin(angle/2)/angle)];};
  const qa=quaternion(a.slice(3)),qb=quaternion(b.slice(3));
  return [Math.hypot(...a.slice(0,3).map((x,i)=>x-b[i]))*1000,2*Math.acos(Math.min(1,Math.abs(qa.reduce((sum,x,i)=>sum+x*qb[i],0))))];
}
async function capture(scene){
  const scenario=await api('/__test__/scene',scene);assert.equal(scenario.status,200);
  await click('validation-capture');
  await wait('document.getElementById("validation-preview").hidden===false&&!document.getElementById("validation-capture").disabled','validation capture');
  const value=(await api('/api/state')).body.project.validation;
  assert.equal(value.status,'captured');
  await wait('document.getElementById("validation-image").complete&&document.getElementById("validation-image").naturalWidth>0','annotated image');
  for(const key of ['marker','pick','approach'])assert.match(await evaluate(`document.getElementById('validation-${key}-pose').textContent`),/X:.*mm[\s\S]*RZ:.*rad/);
  const error=errorBetween(value.approach_tcp_pose_m_rad,scenario.body.expected_approach);
  assert.ok(error[0]<2,'computed position matches marker ground truth: '+error[0]);
  assert.ok(error[1]<.02,'computed rotation matches marker ground truth: '+error[1]);
  assert.equal(await evaluate('document.getElementById("validation-move").disabled'),true);
  return value;
}
async function move(){
  await click('validation-confirmed');
  await wait('!document.getElementById("validation-move").disabled','motion confirmation');
  await click('validation-move');
}
async function result(status){
  await wait(`!document.getElementById('validation-result').hidden&&document.getElementById('validation-motion-result').textContent.startsWith(${JSON.stringify(status)})`,'motion '+status);
  assert.equal(await evaluate('document.getElementById("validation-move").disabled'),true);
  return (await api('/api/state')).body.project.validation;
}
async function screenshot(filename){
  const layout=await send('Page.getLayoutMetrics',{},session);
  const picture=await send('Page.captureScreenshot',{format:'png',captureBeyondViewport:true,clip:{x:0,y:0,width:layout.cssContentSize.width,height:layout.cssContentSize.height,scale:1}},session);
  fs.writeFileSync(filename,Buffer.from(picture.data,'base64'));
}

(async()=>{
  const lines=readline.createInterface({input:fixture.stdout});
  const ready=await Promise.race([once(lines,'line'),new Promise((_,reject)=>setTimeout(()=>reject(new Error('Fixture startup failed: '+log)),10000))]);
  ({url}=JSON.parse(ready[0]));
  await openPage();
  await click('tcp_confirmed');await click('configure');
  await wait('!document.getElementById("step-capture").hidden','setup complete');
  await click('capture');await wait('document.getElementById("light-capture").textContent==="● Ready"','reference capture');
  await click('next');await click('marker_unchanged');await click('teach');
  await wait('document.getElementById("light-teach").textContent==="● Ready"','teach complete');
  await click('next');await click('locate');
  await wait('document.getElementById("light-locate").textContent==="● Ready"','read/export complete');
  const original=(await api('/__test__/metrics')).body;
  await click('next');assert.equal(await evaluate('document.getElementById("step-counter").textContent'),'Step 5 of 5');

  const first=await capture({dx_mm:8,dy_mm:-6,yaw_deg:5});
  assert.ok(Math.abs(first.validation.marker_yaw_change_deg-5)<.4);
  assert.equal(await evaluate('fetch("/api/validation/move",{method:"POST",headers:{"Content-Type":"application/json"},body:"{}"}).then(r=>r.status)'),403);
  await move();const completed=await result('completed');
  assert.ok(completed.error.position_mm<=1);assert.ok(completed.error.rotation_deg<=.287);
  assert.equal(await evaluate('document.getElementById("light-validate").textContent'),'● Pending');
  await click('validation-aligned');await wait('document.getElementById("light-validate").textContent==="● Ready"','visual pass');
  assert.match(await evaluate('document.getElementById("validation-history").textContent'),/visual: aligned/);
  if(process.env.MARKER_VALIDATION_SCREENSHOT){
    await screenshot(process.env.MARKER_VALIDATION_SCREENSHOT.replace(/\.png$/,'-success.png'));
    await send('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true},session);
    assert.equal(await evaluate('document.documentElement.scrollWidth<=window.innerWidth'),true,'mobile page fits viewport');
    await screenshot(process.env.MARKER_VALIDATION_SCREENSHOT.replace(/\.png$/,'-mobile.png'));
    await send('Emulation.setDeviceMetricsOverride',{width:1280,height:960,deviceScaleFactor:1,mobile:false},session);
  }
  const replay=await api('/api/validation/move',{capture_id:first.id,path_confirmed:true,speed_mm_s:10,workspace_min_m:[-.9,-.9,.02],workspace_max_m:[.9,.9,1.2]});assert.equal(replay.status,400);

  const second=await capture({dx_mm:-6,dy_mm:3,yaw_deg:-4});assert.notEqual(first.id,second.id);
  await move();await result('completed');await click('validation-misaligned');
  await wait('document.getElementById("light-validate").textContent==="● Failed"','visual mismatch');
  const history=(await api('/api/state')).body.project.validation_history;
  assert.equal(history.length,2);assert.equal(history[0].visual_result,'aligned');assert.equal(history[1].visual_result,'misaligned');
  const metrics=(await api('/__test__/metrics')).body;
  assert.equal(metrics.reference_sha256,original.reference_sha256);assert.equal(metrics.grasp_sha256,original.grasp_sha256);assert.equal(metrics.target_sha256,original.target_sha256);
  assert.equal(metrics.moves.length,2);assert.ok(errorBetween(metrics.moves[0],metrics.moves[1])[1]>.1,'robot orientation follows marker rotation');

  await capture({yaw_deg:3});await api('/__test__/scene',{expire_capture:true});
  await wait('document.getElementById("validation-freshness").textContent.includes("expired")','capture expiry');
  assert.equal(await evaluate('document.getElementById("validation-move").disabled'),true);

  await capture({yaw_deg:2,mode:'slow'});await move();
  await wait('fetch("/__test__/metrics").then(r=>r.json()).then(m=>m.moves.length===3)','active move before stop');
  await click('validation-stop');await result('stopped');
  await capture({yaw_deg:-2,mode:'wrong_tcp'});await move();const refused=await result('failed');assert.match(refused.message,/TCP/);
  assert.equal((await api('/__test__/metrics')).body.moves.length,3);

  await capture({yaw_deg:1,mode:'slow'});await move();
  await wait('fetch("/__test__/metrics").then(r=>r.json()).then(m=>m.moves.length===4)','active move before browser network loss');
  await send('Network.enable',{},session);
  await send('Network.emulateNetworkConditions',{offline:true,latency:0,downloadThroughput:-1,uploadThroughput:-1},session);
  await wait('document.getElementById("message").textContent.includes("connection lost")','network-loss feedback');
  assert.equal(await evaluate('document.getElementById("validation-move").disabled'),true);
  let state;
  for(let attempt=0;attempt<80;attempt++){
    state=await (await fetch(url+'/api/state')).json();
    if(!state.busy)break;await delay(100);
  }
  assert.equal(state.busy,null);assert.equal(state.project.validation.status,'failed');assert.match(state.error,/heartbeat/i);
  await send('Network.emulateNetworkConditions',{offline:false,latency:0,downloadThroughput:-1,uploadThroughput:-1},session);
  await result('failed');
  await send('Target.closeTarget',{targetId:target});
  await openPage();await wait('!document.getElementById("step-validate").hidden','restore validation panel');
  assert.equal(await evaluate('document.getElementById("validation-move").disabled'),true);
  assert.equal(state.project.validation_history.length,5);
  assert.deepEqual(jsErrors,[],'no browser JavaScript errors');
  if(process.env.MARKER_VALIDATION_SCREENSHOT)await screenshot(process.env.MARKER_VALIDATION_SCREENSHOT);
  console.log('Browser verified: real setup/capture/teach/export; rotated markers; two approach moves; measured TCP; visual pass/fail; preserved teaching/exports; replay/expiry rejection; Stop; wrong TCP; network-loss heartbeat stop; restored history; no JavaScript errors.');
})().catch(error=>{console.error(error.stack+'\n'+log);process.exitCode=1;}).finally(async()=>{
  async function close(child){if(child.exitCode!==null||child.signalCode!==null)return;const exited=once(child,'exit');child.kill('SIGTERM');await exited;}
  await Promise.all([close(fixture),close(chrome)]);
  fs.rmSync(directory,{recursive:true,force:true});
});
