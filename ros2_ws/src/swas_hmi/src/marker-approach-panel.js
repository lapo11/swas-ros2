// SWAS_MARKER_APPROACH_OPERATION_V4
// Preparation display and explicit isolated-dry-run/authorized-motion controls.
// No direct velocity publication, controller remapping, or process management.
import React,{useEffect,useRef,useState} from 'react';

const h=React.createElement;
const PREPARE='/bxu/operations/marker_approach/prepare';
const START='/bxu/operations/marker_approach/start';
const AUTHORIZE='/bxu/operations/marker_approach/authorize';
const STOP='/bxu/operations/marker_approach/stop';
const GATE_STOP='/bxu/control/stop';
const STATUS_AGE_MS=2500;
const nice=value=>value===undefined||value===null||value===''?'Unavailable':String(value).replaceAll('_',' ');

function Pill({children,good=false}){return h('span',{className:'pill '+(good?'good':'')},children);}

export function MarkerApproachPanel({statusRow,operationRow,connected,services=[],now=Date.now(),onTrigger,profile,getSnapshot,onBeforeStart}){
 const [busy,setBusy]=useState('');
 const [result,setResult]=useState('');
 const sequence=useRef(0);
 const pending=useRef('');
 const mounted=useRef(true);
 const live=useRef(null);
 live.current={getSnapshot,profile,onTrigger};

 useEffect(()=>{
  mounted.current=true;
  return ()=>{
   mounted.current=false;
   sequence.current+=1;
  };
 },[]);

 const pause=ms=>new Promise(resolve=>setTimeout(resolve,ms));
 const message=error=>error?.message||String(error);
 const show=text=>{if(mounted.current)setResult(text);};

 function snapshot(){
  const current=live.current;
  const report=current.getSnapshot();
  if(current.profile!=='sim')
   throw new Error('Operation commands are disabled outside the simulation profile.');
  if(!report.connected)
   throw new Error('Station disconnected during the start sequence.');
  return report;
 }

 function assertCurrent(token){
  if(!mounted.current||sequence.current!==token)
   throw new Error('Start sequence cancelled.');
  return snapshot();
 }

 function readOperation(row){
  const age=Date.now()-row?.at;
  if(!row||!Number.isFinite(age)||age<0||age>1000)return null;
  try{
   const value=JSON.parse(row.value?.data||'null');
   if(value?.operation!=='marker_approach')return null;
   return value;
  }catch{return null;}
 }

 function isolatedActive(value){
  return value?.state==='ACTIVE'
   &&value.motion_authorized===false
   &&value.motion_output_connected===false;
 }

 async function checkedTrigger(service){
  const response=await live.current.onTrigger(service);
  if(response?.success!==true)
   throw new Error(response?.message||'Service did not confirm success: '+service);
  return response;
 }

 async function stopBoth(){
  const reports=[];
  // Independent attempts: one failure must not skip the other request.
  for(const service of [GATE_STOP,STOP]){
   try{
    const response=await checkedTrigger(service);
    reports.push(response.message||service+': accepted');
   }catch(error){
    reports.push(service+': FAILED: '+message(error));
   }
  }
  return reports.join(' ');
 }


 let preparation=null,preparationError='';
 let operation=null,operationError='';
 try{
  preparation=JSON.parse(statusRow?.value?.data||'null');
  if(preparation&&preparation.operation!=='marker_approach')preparationError='Unsupported preparation report.';
 }catch{
  preparation=null;
  preparationError='Preparation report could not be read.';
 }
 try{
  operation=JSON.parse(operationRow?.value?.data||'null');
  if(operation&&operation.operation!=='marker_approach')operationError='Unsupported operation report.';
 }catch{
  operation=null;
  operationError='Operation report could not be read.';
 }

 const preparationAge=statusRow?now-statusRow.at:null;
 const operationAge=operationRow?now-operationRow.at:null;
 const preparationFresh=connected&&preparationAge!==null&&preparationAge>=0&&preparationAge<=STATUS_AGE_MS&&!!preparation&&!preparationError;
 const operationFresh=connected&&operationAge!==null&&operationAge>=0&&operationAge<=STATUS_AGE_MS&&!!operation&&!operationError;

 const state=operationFresh?operation.state:'UNKNOWN';
 const prepared=preparationFresh&&preparation.state==='PREPARED';
 const active=state==='ACTIVE';
 const canStart=profile==='sim'&&connected&&preparationFresh&&operationFresh&&state==='IDLE'&&services.includes(START)&&services.includes(AUTHORIZE)&&services.includes(STOP)&&services.includes(GATE_STOP)&&!busy;
 const canStop=profile==='sim'&&connected&&(services.includes(STOP)||services.includes(GATE_STOP))&&busy!=='stop';
 const canCheck=connected&&services.includes(PREPARE)&&!busy;

 async function check(){
  if(!canCheck)return;
  setBusy('check');setResult('Checking preparation…');
  try{
   const response=await onTrigger(PREPARE);
   setResult(response.message||'Preparation check completed.');
  }catch(err){setResult(err.message);}
  finally{setBusy('');}
 }

 async function start(){
  if(!canStart||pending.current)return;
  if(!window.confirm('Start the marker-approach operation and authorize gate motion? The simulated BXU may move. Confirm the simulation area is clear.'))return;

  const token=++sequence.current;
  pending.current='start';
  setBusy('start');
  show('Starting controller; waiting for a fresh ACTIVE report…');

  try{
   onBeforeStart?.();
   assertCurrent(token);

   // After Stop, preparation can legitimately be BLOCKED because the
   // controller was active. Recovery is a non-motion preparation request.
   const initialPreparation=(()=>{
    const report=assertCurrent(token);
    const row=report.statusRow;
    const age=Date.now()-(row?.at||0);
    if(!row||!Number.isFinite(age)||age<0||age>STATUS_AGE_MS)
     throw new Error('No fresh preparation report before start.');
    try{
     const value=JSON.parse(row.value?.data||'null');
     if(value?.operation!=='marker_approach')
      throw new Error('Unsupported preparation report before start.');
     return value;
    }catch(error){
     if(error instanceof Error&&error.message.startsWith('Unsupported'))
      throw error;
     throw new Error('Preparation report could not be read before start.');
    }
   })();

   if(initialPreparation.state!=='PREPARED'){
    show('Preparation is '+initialPreparation.state+'; requesting recovery preparation…');
    await checkedTrigger(PREPARE);

    const preparedDeadline=performance.now()+4000;
    let recovered=false;
    while(performance.now()<preparedDeadline){
     const report=assertCurrent(token);
     const row=report.statusRow;
     const age=Date.now()-(row?.at||0);
     if(row&&Number.isFinite(age)&&age>=0&&age<=STATUS_AGE_MS){
      try{
       const value=JSON.parse(row.value?.data||'null');
       if(value?.operation==='marker_approach'&&value.state==='PREPARED'){
        recovered=true;
        break;
       }
      }catch{}
     }
     await pause(100);
    }
    if(!recovered)
     throw new Error('Preparation did not return to PREPARED within 4 seconds.');
   }

   assertCurrent(token);
   const requestedAt=Date.now();
   const started=await checkedTrigger(START);
   assertCurrent(token);

   // Read current robot.state, not the props captured before START.
   const activeDeadline=performance.now()+4000;
   let observedActive=false;
   while(performance.now()<activeDeadline){
    const report=assertCurrent(token);
    const row=report.operationRow;
    const operation=readOperation(row);
    if(row?.at>=requestedAt&&isolatedActive(operation)){
     observedActive=true;
     break;
    }
    if(operation&&['FAULT','DONE'].includes(operation.state))
     throw new Error('Operation entered '+operation.state+' before authorization.');
    await pause(100);
   }
   if(!observedActive)
    throw new Error('No fresh ACTIVE operation report received within 4 seconds.');

   show('ACTIVE received; requesting gate authorization…');

   // The browser and gate receive state independently. Retry only the
   // exact pre-authorization state refusal, for a bounded interval.
   // Never retry a successful authorization or a transport/other error.
   const authorityDeadline=performance.now()+2500;
   let authorized=null;
   while(performance.now()<authorityDeadline){
    const report=assertCurrent(token);
    if(!isolatedActive(readOperation(report.operationRow)))
     throw new Error('Operation is no longer ACTIVE and fresh.');

    try{
     authorized=await checkedTrigger(AUTHORIZE);
     break;
    }catch(error){
     if(message(error)!=='Operation authority refused: marker approach is not ACTIVE and fresh')
      throw error;
     await pause(100);
    }
   }
   if(!authorized)
    throw new Error('Gate did not receive an acceptable ACTIVE report within the authorization window.');

   assertCurrent(token);
   show((started.message||'Controller started.')+' '+
        (authorized.message||'Gate authorization accepted.')+
        ' Check reported gate authority and simulation motion.');
  }catch(error){
   const stopped=await stopBoth();
   if(mounted.current&&sequence.current===token)
    show('Approach start failed: '+message(error)+' Stop requests: '+stopped);
  }finally{
   if(sequence.current===token){
    pending.current='';
    if(mounted.current)setBusy('');
   }
  }
 }

 async function stop(){
  if(!canStop||pending.current==='stop')return;
  // Stop does not require a confirmation or a fresh operation report.
  const token=++sequence.current;
  pending.current='stop';
  setBusy('stop');
  show('Requesting gate STOP and controller disable…');
  try{
   const report=await stopBoth();
   if(mounted.current&&sequence.current===token)show(report);
  }finally{
   if(sequence.current===token){
    pending.current='';
    if(mounted.current)setBusy('');
   }
  }
 }

 const components=preparation?.component_states||{};
 const controller=preparation?.controller_status;
 const good=operationFresh&&state==='ACTIVE';

 return h('section',{className:'card'},
  h('div',{className:'section-head'},
   h('h2',null,'Marker approach'),
   h(Pill,{good},operationFresh?nice(state):'Not current')
  ),
  h('p',null,operationFresh?operation.reason:(preparationFresh?preparation.reason:'No current marker-approach report.')),
  h('div',{className:'report'},
   h('div',null,h('strong',null,'Preparation'),h('small',null,preparationFresh?nice(preparation.state):'Not current')),
   h('div',null,h('strong',null,'Front-marker vision'),h('small',null,nice(components.front_markers))),
   h('div',null,h('strong',null,'Docking vision'),h('small',null,nice(components.docking))),
   h('div',null,h('strong',null,'Controller'),h('small',null,nice(controller))),
   h('div',null,h('strong',null,'Motion authority'),h('small',null,'See reported gate authority in the top status bar'))
  ),
  h('div',{className:'button-row'},
   h('button',{className:'primary',disabled:!canStart,onClick:start},
    busy==='start'?'Starting…':'Start approach'),
   h('button',{className:'danger',disabled:!canStop,onClick:stop},
    busy==='stop'?'Stopping…':'Stop approach'),
   h('button',{disabled:!canCheck,onClick:check},
    busy==='check'?'Checking…':'Check preparation')
  ),
  !connected&&h('p',{className:'note'},'Station disconnected. Displayed reports are historical.'),
  connected&&!services.includes(START)&&h('p',{className:'note'},'Start service is not discovered. Start the marker-approach operation coordinator.'),
  connected&&!services.includes(AUTHORIZE)&&h('p',{className:'note'},'Authorize service is not discovered. Restart the platform gate with operation authorization enabled.'),
  connected&&preparationFresh&&preparation.state==='BLOCKED'&&h('p',{className:'note'},'Preparation is BLOCKED: '+nice(preparation.reason)+'. Start approach will first request non-motion recovery preparation.'),
  connected&&!preparationFresh&&h('p',{className:'note'},'Preparation status is not current. Check the preparation monitor.'),
  h('p',{className:'command-result',role:'status'},result||'Start requires an IDLE operation. If preparation is BLOCKED after a stop, Start first requests non-motion recovery preparation, waits for PREPARED, then waits for ACTIVE before gate authorization. Stop requests gate STOP and controller disable.'),
  h('small',null,'Simulation validation only. The command gate remains the sole final-motion publisher.')
 );
}
