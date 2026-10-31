// SWAS_TWO_VISION_ROWS_V1
// SWAS_VISION_APPLICATIONS_PANEL_V1
// SWAS_OPERATOR_HELP_V1
import {OperatorHelp} from './operator-help.js';
import React,{useEffect,useRef,useState} from 'react';
import {createRoot} from 'react-dom/client';
import {useRobot} from './robot';
import {Camera} from './camera';
import {TOPICS,CAMERAS,WORKFLOWS,CONTROL} from './config';
import './style.css';
import {ApplicationsPanel} from './applications-panel.js';
import {MarkerApproachPanel} from './marker-approach-panel.js';
import './applications.css';
const h=React.createElement;
const fmt=(v,p=2)=>typeof v==='number'&&Number.isFinite(v)?v.toFixed(p):'—';
const nice=v=>v===undefined||v===null?'Unavailable':String(v).replaceAll('_',' ');
const topicOf=key=>TOPICS.find(t=>t[0]===key)?.[1];
const sleep=ms=>new Promise(resolve=>setTimeout(resolve,ms));
function Pill({children,good=false}){return h('span',{className:'pill '+(good?'good':'')},children);}
function Metric({title,value,note}){return h('section',{className:'card metric'},h('span',{className:'eyebrow'},title),h('strong',null,value),h('small',null,note));}
function loadProfiles(){const host=window.location.hostname||'localhost',defaults={sim:{bridge:`ws://${host}:9090`,video:`http://${host}:8080`},hardware:{bridge:'',video:''}};try{return {...defaults,...JSON.parse(localStorage.getItem('swas.profiles')||'{}')};}catch{return defaults;}}
function App(){
 const robot=useRobot(),r=robot.state;
 const [tab,setTab]=useState('Overview'),[env,setEnv]=useState('sim'),[profiles,setProfiles]=useState(loadProfiles),[draft,setDraft]=useState(profiles[env]);
 const [workflow,setWorkflow]=useState('demo_markers'),[cameraId,setCameraId]=useState('auto'),[cameras,setCameras]=useState(['front','dock0Overlay','dock1Overlay']),[fps,setFps]=useState(4);
 const [now,setNow]=useState(Date.now()),[busy,setBusy]=useState(''),[result,setResult]=useState(''),[armed,setArmed]=useState(false),[ack,setAck]=useState(false),[scale,setScale]=useState(50),[vector,setVector]=useState([0,0,0]),[filter,setFilter]=useState('');
 const motion=useRef({armed:false,v:[0,0,0],began:0});

 // SWAS camera workspace update: ten views and discovered Image topics.
 // Keep friendly configured cameras, then add other actual raw Image interfaces.
 const cameraList=[...CAMERAS,...r.topics.filter((topic,index)=>
  r.types[index]==='sensor_msgs/msg/Image' && !CAMERAS.some(camera=>camera[2]===topic)
 ).map(topic=>['discovered:'+topic,topic,topic])];
 const row=key=>r.data[key==='lift'&&r.services.includes('/bxu/platform/manual')?'/bxu/lift/state':topicOf(key)];const value=key=>row(key)?.value;
 const recent=key=>r.link==='Connected'&&row(key)&&now-row(key).at<2000;
 const has=key=>r.topics.includes(topicOf(key));const service=name=>r.services.includes(name);
 const wf=WORKFLOWS.find(w=>w.id===workflow)||WORKFLOWS[0],state=value(wf.stateKey)?.data;
 // SWAS_PLATFORM_STOP_ROUTING_V1
 const platformOnly=service('/bxu/platform/manual');
 const stopService=platformOnly?CONTROL.stop:(wf.stop||WORKFLOWS[0].stop);
 const clockActive=recent('clock')&&now-row('clock').changedAt<2000;
 const connected=r.link==='Connected',healthy=connected&&now-r.discoveryAt<10000;
 const freshState=recent(wf.stateKey),gateMode=value('gate')?.data;
 const release=()=>{if(motion.current.armed)robot.velocity(CONTROL.teleop,[0,0,0]);motion.current.v=[0,0,0];setVector([0,0,0]);};
 const disarm=()=>{release();motion.current.armed=false;setArmed(false);};
 useEffect(()=>{const tick=setInterval(()=>{
  const t=Date.now();setNow(t);const m=motion.current;if(!m.armed)return;
  const s=robot.state,c=s.data[topicOf('clock')],g=s.data[topicOf('gate')];
  if(s.link!=='Connected'||document.hidden||!c||t-c.at>2000||t-c.changedAt>2000||!g||t-g.at>1200||g.value.data!=='MANUAL'){disarm();return;}
  if(!robot.velocity(CONTROL.teleop,m.v))disarm();
 },100);
 const stop=()=>disarm(),key=e=>{if(e.key==='Escape'){e.preventDefault();disarm();}};
 const hide=()=>{if(document.hidden)disarm();};
 window.addEventListener('pointerup',release);window.addEventListener('pointercancel',stop);window.addEventListener('blur',stop);window.addEventListener('pagehide',stop);window.addEventListener('keydown',key);document.addEventListener('visibilitychange',hide);
 return()=>{clearInterval(tick);window.removeEventListener('pointerup',release);window.removeEventListener('pointercancel',stop);window.removeEventListener('blur',stop);window.removeEventListener('pagehide',stop);window.removeEventListener('keydown',key);document.removeEventListener('visibilitychange',hide);};},[]);
 const navigate=next=>{disarm();setTab(next);};
 const switchEnv=next=>{disarm();robot.disconnect();setEnv(next);setDraft(profiles[next]);setResult('');setAck(false);};
 const save=()=>{disarm();robot.disconnect();const next={...profiles,[env]:draft};setProfiles(next);localStorage.setItem('swas.profiles',JSON.stringify(next));};
 async function trigger(name){if(!service(name))throw new Error('Service unavailable: '+name);const response=await robot.call(name,{});robot.event(name+': '+JSON.stringify(response));if(response.success!==true)throw new Error(response.message||'Service rejected request');return response;}
 async function operate(action){
  if(env!=='sim'){setResult('Hardware commands are disabled in this release.');return;}
  if(action!=='stop'&&busy)return;
  if(action==='start'&&!window.confirm('Start the connected demo? Autonomous movement and battery lifting may follow. Confirm the simulation area is clear.'))return;
  if(action==='confirm'&&!window.confirm((wf.confirmationStates[state]||'Continue the sequence')+'? This can initiate movement.'))return;
  setBusy(action);setResult('Waiting for service response…');disarm();
  try{
   if(action==='stop'){
    if(CONTROL.stop!==stopService&&service(CONTROL.stop))await trigger(CONTROL.stop);
    const response=await trigger(stopService);setResult(response.message+' This does not cancel an independently running lift operation.');
   }else if(action==='start'){
    if(service(CONTROL.auto))await trigger(CONTROL.auto);
    try{const response=await trigger(wf.start);setResult(response.message+' Waiting for reported sequence state.');}
    catch(error){if(service(CONTROL.stop))try{await trigger(CONTROL.stop);}catch{}throw error;}
   }else{const response=await trigger(wf.confirm);setResult(response.message+' Check the reported sequence state.');}
  }catch(error){setResult(error.message);robot.event('Operator command: '+error.message);}finally{setBusy('');}
 }
 async function requestManual(){
  if(env!=='sim'||busy)return;setBusy('manual');setResult('Requesting manual authority…');disarm();
  try{
   // MANUAL_PLATFORM_HMI_STAGE2
   if(service('/bxu/platform/manual')){
    const reply=await trigger('/bxu/platform/manual');
    robot.velocity(CONTROL.teleop,[0,0,0]);
    setResult(reply.message+' Arm jogging separately. This platform has no automatic demo loaded.');
    return;
   }

   if(!service(CONTROL.manual)||!service(stopService))throw new Error('Install/remap the optional simulation command gate first; see SIM_TELEOP.md.');
   await trigger(stopService);
   const until=Date.now()+3500;let ready=false;
   while(Date.now()<until){const d=robot.state.data;ready=d[topicOf('mission')]?.value?.data==='WAITING'&&d[topicOf('frontEnabled')]?.value?.data===false&&d[topicOf('dockEnabled')]?.value?.data===false;if(ready)break;await sleep(100);}
   if(!ready)throw new Error('No stopped-supervisor acknowledgement. Manual authority not requested.');
   const reply=await trigger(CONTROL.manual);setResult(reply.message+' Arm jogging separately.');
  }catch(error){setResult(error.message);robot.event(error.message);}finally{setBusy('');}
 }
 const canStart=env==='sim'&&healthy&&clockActive&&freshState&&wf.startStates?.includes(state)&&service(wf.start)&&!busy;
 const canConfirm=env==='sim'&&healthy&&clockActive&&freshState&&!!wf.confirmationStates?.[state]&&service(wf.confirm)&&!busy;
 const canStop=env==='sim'&&connected&&!!stopService&&service(stopService);
 const missionControls=()=>service('/bxu/platform/manual')
  ? h('section',{className:'card mission'},h('h2',null,'Manual-only simulation platform'),h('p',null,'No automatic demo is loaded. Use Manual to request control and arm jogging. Dynamic application loading is the next increment.'),h('button',{onClick:()=>navigate('Manual')},'Open manual controls'),h('small',null,'Lift feedback is simulated acknowledgement only; lift control is not enabled.'))
  : h('section',{className:'card mission'},h('div',{className:'section-head'},h('h2',null,'Operator actions'),h(Pill,{good:freshState},freshState?nice(state):'No recent sequence state')),
  h('label',null,'Connected workflow',h('select',{value:workflow,disabled:!!busy,onChange:e=>{disarm();setWorkflow(e.target.value);setResult('');}},...WORKFLOWS.map(w=>h('option',{key:w.id,value:w.id},w.name)))),
  h('p',null,wf.description),
  wf.start?h('div',{className:'button-row'},h('button',{className:'primary',disabled:!canStart,onClick:()=>operate('start')},busy==='start'?'Requesting…':'Start demo'),h('button',{disabled:!canConfirm,onClick:()=>operate('confirm')},wf.confirmationStates[state]||'Continue'),h('button',{className:'danger',disabled:!canStop,onClick:()=>operate('stop')},platformOnly?'Stop platform':'Stop sequence')):h('p',{className:'note'},'This workflow is monitoring-only.'),
  !clockActive&&env==='sim'&&h('p',{className:'note'},'Simulation clock is not advancing. Unpause Gazebo before Start / Continue.'),
  wf.start&&!service(wf.start)&&h('p',{className:'note'},'Supervisor services not discovered. Launch the demo with its supervisor enabled.'),
  h('p',{className:'command-result',role:'status'},result||'Actions require actual discovered services and recent supervisor reports.'),
  h('small',null,'Stop sequence requests controller disable and zero velocity. It is not an emergency stop or lift-cancel command.'));
 const autoCam=r.topics.includes('/bxu/front_vision/image_raw')?'markers':r.topics.includes('/bxu/front_yolo_vision/image_raw')?'yolo':'front';
 const camera=id=>{const c=cameraList.find(c=>c[0]===(id==='auto'?autoCam:id))||cameraList[0];return h(Camera,{key:env+c[0],camera:c,base:profiles[env].video,available:r.topics.includes(c[2]),connected,fps});};
 const cameraSelect=h('select',{value:cameraId,'aria-label':'Camera source',onChange:e=>setCameraId(e.target.value)},h('option',{value:'auto'},'Automatic · available front view'),...cameraList.map(c=>h('option',{key:c[0],value:c[0]},c[1])));
 const statusCard=(title,keys)=>h('section',{className:'card'},h('h2',null,title),...keys.map(key=>{const msg=row(key);return h('div',{className:'report',key},h('div',null,h('strong',null,TOPICS.find(t=>t[0]===key)[2]),h('small',null,msg?'Last message '+Math.max(0,(now-msg.at)/1000).toFixed(1)+'s ago':has(key)?'Topic discovered; no message received':'Not available in this graph')),h('span',null,nice(msg?.value?.data)));}));
 const alignment=(key,title)=>{const msg=row(key),a=msg?.value,valid=a?.markers_detected===true;return h('section',{className:'card'},h('div',{className:'section-head'},h('h3',null,title),h(Pill,{good:valid&&recent(key)},!msg?'No data':!recent(key)?'Historical':valid?'Target reported':'No target')),
  h('div',{className:'alignment'},...[['Lateral',valid?fmt(a.lateral_error_m*1000,1):'—','mm'],['Yaw',valid?fmt(a.yaw_error_rad*180/Math.PI,1):'—','°'],['Distance',valid&&a.distance_m>=0?fmt(a.distance_m):'—','m']].map(([label,v,unit])=>h('div',{key:label},h('small',null,label),h('strong',null,v,h('small',null,unit))))),h('small',null,msg?'Last received '+Math.max(0,(now-msg.at)/1000).toFixed(1)+'s ago':'No estimate received. Invalid detections are not displayed as zero alignment.'));};
 const frontKey=has('frontMarker')?'frontMarker':'frontYolo';
 const odom=value('odom'),twist=odom?.twist?.twist,pos=odom?.pose?.pose?.position,speed=twist?Math.hypot(twist.linear.x,twist.linear.y):null,cmd=value('cmd');
 let content;
 if(tab==='Overview')content=h(React.Fragment,null,
  h('section',{className:'station-banner'},h('div',null,h('span',{className:'eyebrow'},'SMART WORKSPACE · BATTERY SWAPPING'),h('h2',null,'One station. A shared view.'),h('p',null,'Exchange unit, receiving unit and operator workflow — together.'),h('div',{className:'button-row'},h(Pill,null,env==='sim'?'Simulation connection':'Hardware · monitor only'),h(Pill,null,r.topics.length+' discovered topics'))),h('figure',null,h('img',{src:'/station-reference.jpg',alt:'Static Gazebo station reference'}),h('figcaption',null,'GAZEBO STATION · STATIC REFERENCE'))),
  h('div',{className:'metrics'},h(Metric,{title:'Exchange sequence',value:service('/bxu/platform/manual')?'No demo loaded':freshState?nice(state):'Awaiting supervisor',note:'Reported supervisor state, not inferred from vision'}),h(Metric,{title:'BXU estimated speed',value:fmt(speed,3)+' m/s',note:recent('odom')?'Recent odometry':'No recent odometry; last value may be historical'}),h(Metric,{title:'Command authority',value:recent('gate')?nice(gateMode):'Original demo wiring',note:recent('gate')?'Optional simulation command gate':'Manual takeover not integrated'})),
  missionControls(),h('div',{className:'overview-grid'},h('div',null,h('div',{className:'section-head'},h('h2',null,'Station camera'),cameraSelect),camera(cameraId)),h('div',{className:'stack'},statusCard('BXU · exchange unit',['frontStatus','dockStatus','lift']),statusCard('BRU · receiving unit',['bruLock','bruSeated']))),
  h('div',{className:'two-col'},alignment(frontKey,frontKey==='frontMarker'?'Front marker alignment':'Front YOLO alignment'),alignment('dock','Docking alignment')));
 if(tab==='Operations')content=h(React.Fragment,null,missionControls(),
  h(MarkerApproachPanel,{
  operationRow:row('markerApproachOperation'),statusRow:row('markerApproachStatus'),connected,services:r.services,now,onTrigger:trigger,
  profile:env,onBeforeStart:disarm,getSnapshot:()=>{
   const current=robot.state;
   // SWAS_MARKER_APPROACH_SNAPSHOT_V2
   return {
    connected:current.link==='Connected',
    statusRow:current.data[topicOf('markerApproachStatus')],
    operationRow:current.data[topicOf('markerApproachOperation')]
   };
  }}),h('section',{className:'card'},h('h2',null,'Connected demo lifecycle'),h('ol',{className:'sequence'},...['WAITING','FRONT_APPROACH','DOCKING_ALIGN','LIFTING','WAIT_USER_AFTER_LIFT','LOWERING','WAIT_USER_AFTER_LOWER','DRIVE_AWAY','DONE'].map(s=>h('li',{key:s,className:state===s?'current':''},nice(s)))),h('p',null,'This sequence describes the supplied exchange workflow. Additional workflows belong in src/config.js with their real state and service contracts.')),h('section',{className:'card'},h('h2',null,'Launch management is separate'),h('p',null,'Start demo calls the running supervisor. It does not start a ROS launch file. Approved launch profiles need a small process manager on the machine running ROS; arbitrary browser shell commands are not provided.')));
 if(tab==='Cameras')content=h(React.Fragment,null,h('section',{className:'card'},h('div',{className:'section-head'},h('div',null,h('h2',null,'Camera workspace'),h('p',null,'Choose up to ten views. Inactive pages stop their preview requests.')),h('label',null,'Maximum preview rate',h('select',{value:fps,onChange:e=>setFps(Number(e.target.value))},...[2,4,8].map(n=>h('option',{key:n,value:n},n+' fps'))))),h('div',{className:'choices'},...cameraList.map(c=>h('label',{key:c[0]},h('input',{type:'checkbox',checked:cameras.includes(c[0]),disabled:!cameras.includes(c[0])&&cameras.length>=10,onChange:e=>setCameras(e.target.checked?[...cameras,c[0]]:cameras.filter(x=>x!==c[0]))}),c[1])))),h('div',{className:'two-col'},...cameras.map(camera)));
 if(tab==='Telemetry')content=h(React.Fragment,null,h('div',{className:'metrics'},h(Metric,{title:'Odometry X',value:fmt(pos?.x,3)+' m',note:'Published odometry frame'}),h(Metric,{title:'Odometry Y',value:fmt(pos?.y,3)+' m',note:'Not a global map position'}),h(Metric,{title:'Requested forward velocity',value:fmt(cmd?.linear?.x,3)+' m/s',note:'Observed /bxu/cmd_vel, not measured movement'})),h('div',{className:'two-col'},statusCard('BXU state reports',['frontStatus','dockStatus','frontEnabled','dockEnabled','lift']),statusCard('BRU state reports',['bruLock','bruSeated'])),h('div',{className:'two-col'},alignment('frontMarker','Front markers'),alignment('frontYolo','Front YOLO')),alignment('dock','Docking'),h('section',{className:'card'},h('h2',null,'BRU simulation model'),h('p',null,value('bruJoints')?'Joint-model messages received. This does not verify lock status, battery seating or physical BRU health.':'No BRU model messages received. Physical BRU signals remain unavailable unless published.')));
 if(tab==='Manual'){
  const canArm=env==='sim'&&healthy&&clockActive&&recent('gate')&&gateMode==='MANUAL'&&ack;
  const directions=[['↶','Rotate left',[0,0,1]],['↑','Forward',[1,0,0]],['↷','Rotate right',[0,0,-1]],['←','Strafe left',[0,1,0]],['■','Release',[0,0,0]],['→','Strafe right',[0,-1,0]],['','',null],['↓','Reverse',[-1,0,0]],['','',null]];
  content=h(React.Fragment,null,h('div',{className:'notice'},env==='hardware'?'Hardware movement is disabled. Validate command authority and physical safety before hardware activation.':'Manual jogging requires the optional simulation command gate and three publisher remaps. Without those, this page will not claim to control the robot. See SIM_TELEOP.md.'),h('div',{className:'two-col'},h('section',{className:'card'},h('div',{className:'section-head'},h('h2',null,'Deliberate manual control'),h(Pill,{good:armed},armed?'Armed':'Disarmed')),
   h('button',{className:'primary',disabled:env!=='sim'||!healthy||!service(CONTROL.manual)||!!busy,onClick:requestManual},platformOnly?'Request manual authority':'Stop sequence & request manual'),h('p',{role:'status'},result),
   h('label',null,'Jog speed '+scale+'%',h('input',{type:'range',min:10,max:100,step:10,value:scale,disabled:armed,onChange:e=>setScale(Number(e.target.value))})),
   h('label',{className:'check'},h('input',{type:'checkbox',checked:ack,disabled:armed,onChange:e=>setAck(e.target.checked)}),'I verified simulation-only gate wiring and a clear movement area.'),h('div',{className:'button-row'},h('button',{className:'primary',disabled:!canArm||armed,onClick:()=>{motion.current={armed:true,v:[0,0,0],began:Date.now()};setArmed(true);}},'Arm jogging'),h('button',{disabled:!armed,onClick:disarm},'Disarm')),
   h('div',{className:'jog-pad'},...directions.map(([icon,label,v],i)=>v?h('button',{key:i,disabled:!armed,onPointerDown:e=>{if(!motion.current.armed||motion.current.v.some(Boolean)||(e.pointerType==='mouse'&&e.button!==0))return;e.preventDefault();e.currentTarget.setPointerCapture(e.pointerId);const next=v.map((x,i)=>x*(i===2?.15:.20)*scale/100);motion.current.v=next;motion.current.began=Date.now();setVector(next);robot.velocity(CONTROL.teleop,next);},onPointerUp:release,onPointerCancel:disarm,onLostPointerCapture:release,onContextMenu:e=>e.preventDefault()},h('strong',null,icon),h('small',null,label)):h('span',{key:i}))),h('p',{className:'note'},'Requested velocities: '+vector.map(v=>fmt(v,3)).join(' / ')+' · x, y m/s; yaw rad/s. Release sends zero; no fixed hold-duration limit.')),h('div',null,camera(cameraId),h('section',{className:'card'},h('h3',null,'Manual is not automatic fallback'),h('p',null,'The optional gate stays in Manual after release. Lost input gives zero, not resumed autonomy. Restart the demo explicitly to select Auto again. Browser Escape/disarm stops this teleop input, not a lift mechanism.')))));
 }
 if(tab==='Help')content=h(OperatorHelp,{platformOnly:service('/bxu/platform/manual'),connection:r.link,profile:env,mission:state,missionFresh:freshState,authority:gateMode,authorityFresh:recent('gate'),clockAdvancing:clockActive,onNavigate:navigate});
 if(tab==='Applications')content=h(ApplicationsPanel,{statusRow:row('runtime'),dockingStatusRow:row('runtimeDocking'),connected,profile:env,services:r.services,now,onTrigger:trigger,onNavigate:navigate});
 if(tab==='System')content=h(React.Fragment,null,h('div',{className:'two-col'},h('section',{className:'card'},h('h2',null,'Connection profile'),h('label',null,'Rosbridge WebSocket',h('input',{value:draft.bridge,onChange:e=>setDraft({...draft,bridge:e.target.value})})),h('label',null,'Video server HTTP',h('input',{value:draft.video,onChange:e=>setDraft({...draft,video:e.target.value})})),h('button',{className:'primary',onClick:save},'Save & disconnect'),h('p',null,'Profiles change endpoints only. Both use the same BXU/BRU topic map. A selected profile does not prove endpoint identity.')),
  h('section',{className:'card'},h('h2',null,'Connection evidence'),...[
   ['WebSocket',r.link],['Wire messages received',r.wire.received],['Topic messages received',r.wire.publishes],['Interfaces with data',Object.keys(r.data).length],['Discovered services',r.services.length],['Simulation clock',clockActive?'Advancing':'Not advancing / unavailable']
  ].map(([label,v])=>h('div',{className:'report',key:label},h('span',null,label),h('strong',null,nice(v)))))),h('section',{className:'card'},h('div',{className:'section-head'},h('h2',null,'Telemetry subscription evidence'),h('button',{onClick:()=>{robot.state.subscriptions={};robot.discover();}},'Re-request subscriptions')),
   h('div',{className:'table'},...TOPICS.map(([key,topic,label])=>h('div',{className:'table-row',key},h('div',null,h('strong',null,label),h('code',null,topic)),h('span',null,r.subscriptions[topic]||'Not discovered'),h('small',null,row(key)?row(key).count+' messages':'0 messages'))))),
  h('section',{className:'card'},h('h2',null,'Actual ROS graph'),h('input',{placeholder:'Filter topics or services',value:filter,onChange:e=>setFilter(e.target.value)}),h('div',{className:'table'},...r.topics.filter(t=>t.includes(filter)).map(t=>h('div',{className:'table-row',key:t},h('code',null,t),h('small',null,r.types[r.topics.indexOf(t)]))),...r.services.filter(s=>s.includes(filter)).map(s=>h('div',{className:'table-row',key:'service'+s},h('code',null,s),h(Pill,null,'Service'))))),
  h('section',{className:'card'},h('h2',null,'Latest received messages'),...TOPICS.map(([key,topic,label])=>h('details',{key},h('summary',null,label),h('code',null,topic),h('pre',null,row(key)?JSON.stringify(row(key).value,null,2):'No message received. Discovery alone does not prove publication.')))));
 return h('div',{className:'shell'},h('aside',null,h('div',{className:'brand'},h('img',{src:'/ru_tech_logo.png',alt:'RU Tech logo'}),h('div',null,h('strong',null,'SWAS'),h('small',null,'STATION HMI'))),h('label',{className:'profile'},'CONNECTION PROFILE',h('select',{value:env,onChange:e=>switchEnv(e.target.value)},h('option',{value:'sim'},'Simulation'),h('option',{value:'hardware'},'Hardware · read-only'))),
  h('nav',null,...['Overview','Operations','Applications','Cameras','Telemetry','Manual','Help','System'].map(name=>h('button',{key:name,className:tab===name?'active':'',onClick:()=>navigate(name)},name))),h('div',{className:'aside-foot'},'RU TECH · SWAS',h('p',null,'BXU + BRU operator station'),h('small',null,'Local network · v0.3'))),
 h('main',null,h('header',null,h('div',null,h('span',{className:'eyebrow'},'SWAS / '+(env==='sim'?'SIMULATION':'HARDWARE')),h('h1',null,tab==='Manual'?'Manual control':tab)),h('div',{className:'button-row'},h(Pill,{good:healthy},r.link),h('button',{onClick:()=>{disarm();if(connected||r.link==='Connecting')robot.disconnect();else robot.connect(profiles[env].bridge);}},connected||r.link==='Connecting'?'Disconnect':'Connect'),h('button',{className:'danger',disabled:!canStop,onClick:()=>operate('stop')},platformOnly?'Stop platform':'Stop sequence'))),
  h('div',{className:'status-strip'},h('span',null,env==='sim'?'SIMULATION PROFILE':'HARDWARE MONITORING'),h('span',null,clockActive?'Simulation clock advancing':'No advancing simulation clock'),h('span',null,'Authority: '+(recent('gate')?nice(gateMode):'original demo wiring'))),content,
  h('section',{className:'card activity'},h('h3',null,'Recent HMI activity'),r.events.length?h('div',null,...r.events.slice(0,8).map((e,i)=>h('p',{key:i},h('time',null,new Date(e.at).toLocaleTimeString()),e.text))):h('p',null,'Connect to discover the running system.')),
  h('footer',null,'SWAS HMI · RU Tech',h('span',null,'Service acknowledgement is not physical completion.'))));
}
createRoot(document.getElementById('root')).render(h(App));
