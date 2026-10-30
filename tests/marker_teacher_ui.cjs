const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const root=require('node:path').join(__dirname,'../src/ur_pick_place/web/');
const html=fs.readFileSync(root+'teacher.html','utf8');
assert.match(html,/<form id="setup" novalidate>/);
const ids=[...html.matchAll(/id="([^"]+)"/g)].map(x=>x[1]);
let invalidField=null,configureError='';
const elements=Object.fromEntries(ids.map(id=>[id,{id,value:'',textContent:'',checked:false,hidden:false,disabled:false,events:{},classList:{toggle(){}},addEventListener(k,fn){this.events[k]=fn;},append(){},setAttribute(k,v){this[k]=v;},focus(){this.focused=true;},querySelector(){return invalidField;},querySelectorAll(){return [];},closest(){return elements['setup-advanced'];},reportValidity(){return false;}}]));
const settings={project:'/tmp/project',calibration:'/tmp/calibration.json',robot_ip:'fake',tcp:[0,0,.07,0,0,0],target:{kind:'charuco',board_manifest:'/tmp/board.json',board:'front'},motion_model:'planar',approach_mm:100,limits:{max_shift_mm:30,max_rotation_deg:15}};
const project={settings,project:settings.project,flow:{captured:false,taught:false,exported:false},calibration:{verdict:'good',camera_serial:'fake'},target:'Test board',observe_joints:[0,0,0,0,0,0]};
let locateAttempts=0,configuredTCP;
let state={version:'0.1.0',project:null,busy:null,error:'',completed:null};
async function fetch(path,options){
 if(path==='/api/defaults')return{ok:true,json:async()=>({settings,calibrations:[{path:settings.calibration,tcp:settings.tcp,robot_ip:settings.robot_ip}],manifests:[]})};
 if(path==='/api/state')return{ok:true,json:async()=>structuredClone(state)};
 if(path==='/api/open')return{ok:true,json:async()=>structuredClone(settings)};
 if(path==='/api/configure'){
  if(configureError)return{ok:false,json:async()=>({error:configureError})};
  configuredTCP=JSON.parse(options.body).tcp;state.project=project;state.completed='configure';
 }
 if(path==='/api/capture'){project.flow.captured=true;project.observation={quality:{frames_accepted:10,corner_count:24,reprojection_rmse_px:.2}};state.completed='capture';}
 if(path==='/api/teach'){project.flow.taught=true;project.taught={taught_tcp_in_base:[.4,.1,.2,0,3.14,0]};state.completed='teach';}
 if(path==='/api/locate'){
  if(locateAttempts++===0){state.completed=null;state.failed='locate';state.error='Test: unstable marker';}
  else{state.completed='locate';state.failed=null;state.error='';project.flow.exported=true;project.exported={pick_tcp_pose_m_rad:[.4,.1,.2,0,3.14,0],approach_tcp_pose_m_rad:[.4,.1,.3,0,3.14,0]};}
 }
 return{ok:true,json:async()=>({accepted:true})};
}
const document={getElementById:id=>elements[id],querySelector:()=>({content:'token'}),querySelectorAll:()=>[],createElement:()=>({})};
vm.runInNewContext(fs.readFileSync(root+'teacher.js','utf8'),{document,fetch,console,setInterval(){}});
const tick=()=>new Promise(resolve=>setImmediate(resolve));
(async()=>{
 await tick();await tick();
 assert.equal(elements.robot_ip.value,'fake');
 assert.equal(elements.tcp.value,'0, 0, 70, 0, 0, 0');
 assert.match(elements['calibration-tcp'].textContent,/XYZ mm.*0, 0, 70, 0, 0, 0/);
 elements.tcp.value='1, 2, 3, 0, 0, 0';
 elements.calibration.events.change();
 assert.equal(elements.tcp.value,'0, 0, 70, 0, 0, 0');
 assert.equal(elements.capture.disabled,true);
 assert.equal(elements['step-configure'].hidden,false);
 for(const id of ['capture','teach','locate']) assert.equal(elements['step-'+id].hidden,true);
 assert.equal(elements['nav-teach'].disabled,true);
 invalidField=elements.project;invalidField.validationMessage='Please fill out this field.';
 await elements.setup.events.submit({preventDefault(){}});
 assert.equal(elements['setup-advanced'].open,true);
 assert.equal(elements.project.focused,true);
 assert.match(elements.message.textContent,/Check project: Please fill out this field/);
 assert.equal(configuredTCP,undefined);
 invalidField=null;
 elements.tcp.value='0, 0, 228.6';
 await elements.setup.events.submit({preventDefault(){}});
 assert.match(elements.message.textContent,/TCP needs six numbers/);
 assert.equal(configuredTCP,undefined);
 // Copying small PolyScope millimetre readings must not produce metre-sized offsets.
 elements.tcp.value='0.03, 0.08, 0.01, 0.1, -0.2, 0.3';
 elements.tcp_confirmed.checked=true;
 configureError='Test: calibration TCP mismatch';
 await elements.setup.events.submit({preventDefault(){}});
 assert.match(elements.message.textContent,/Test: calibration TCP mismatch/);
 assert.equal(elements.configure.disabled,false);
 assert.equal(elements['step-configure'].hidden,false);
 configureError='';
 await elements.setup.events.submit({preventDefault(){}});await tick();await tick();
 [0.00003,0.00008,0.00001].forEach((v,i)=>assert.ok(Math.abs(configuredTCP[i]-v)<1e-15));
 assert.deepEqual(configuredTCP.slice(3),[0.1,-0.2,0.3]);
 assert.equal(elements.capture.disabled,false);
 assert.equal(elements['step-capture'].hidden,false);
 assert.equal(elements['step-configure'].hidden,true);
 assert.equal(elements.next.disabled,true);

 elements.capture.events.click();await tick();await tick();
 assert.equal(elements['light-capture'].textContent,'● Ready');
 assert.equal(elements['capture-image'].hidden,false); // instant completion before first busy poll
 assert.match(elements.message.textContent,/Marker captured/);
 assert.equal(elements.next.disabled,false);
 elements.next.events.click();
 assert.equal(elements['step-teach'].hidden,false);
 assert.equal(elements['step-capture'].hidden,true);
 elements.marker_unchanged.checked=true;elements.marker_unchanged.events.change();
 assert.equal(elements.teach.disabled,false);
 elements.teach.events.click();await tick();await tick();
 assert.equal(elements.locate.disabled,false);
 elements.next.events.click();
 assert.equal(elements['step-locate'].hidden,false);
 assert.equal(elements.next.hidden,true);
 elements.locate.events.click();await tick();await tick();
 assert.match(elements.message.textContent,/unstable marker/); // instant failure before first busy poll
 assert.equal(elements['light-locate'].textContent,'● Failed');
 elements.locate.events.click();await tick();await tick();
 assert.equal(elements['light-locate'].textContent,'● Ready');
 assert.equal(elements['export-result'].hidden,false);
 elements.back.events.click();
 assert.equal(elements['step-teach'].hidden,false);
 elements['nav-configure'].events.click();
 assert.equal(elements['step-configure'].hidden,false);
 elements.setup.events.input();
 assert.equal(elements.capture.disabled,true);
 assert.equal(elements['saved-links'].hidden,true);
 assert.equal(elements.tcp_confirmed.checked,false);
 elements.open.events.click();await tick();await tick();
 assert.equal(elements.tcp.value,'0, 0, 70, 0, 0, 0'); // saved JSON stays in metres
 elements.tcp_confirmed.checked=true;
 await elements.setup.events.submit({preventDefault(){}});await tick();await tick();
 assert.deepEqual(configuredTCP,settings.tcp); // opening and saving does not rescale the stored TCP
 console.log('Wizard passed: single-panel navigation, step gates, capture, teach, failed read/retry, exports and settings invalidation.');
})().catch(e=>{console.error(e);process.exitCode=1;});
