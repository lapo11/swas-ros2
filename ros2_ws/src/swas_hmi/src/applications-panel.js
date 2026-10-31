import React,{useState} from 'react';
import {COMPONENTS,visionView} from './runtime-vision-model';
const h=React.createElement;
const LABELS={UNLOADED:'Not loaded',STARTING:'Starting',WAITING_FOR_DATA:'Waiting for observations',READY:'Receiving observations',STOPPING:'Unloading',FAILED:'Failed',EXTERNAL:'External process'};

function VisionRow({component,statusRow,connected,profile,services,now,onTrigger,onNavigate}){
 const [busy,setBusy]=useState(null),[message,setMessage]=useState(''),[uncertain,setUncertain]=useState(false);
 // Main's periodic display clock can slightly precede the newest received row.
 const view=visionView({statusRow,connected,profile,services,now:Math.max(now||0,Date.now()),componentId:component.id});
 const data=view.data;
 async function request(action){
  if(busy||uncertain||!(action==='load'?view.canLoad:view.canUnload))return;
  setBusy(action);setMessage('Waiting for '+component.label.toLowerCase()+' manager response…');
  try{
   const response=await onTrigger(action==='load'?component.load:component.unload);
   if(response?.success!==true)throw new Error(response?.message||'Manager rejected the request.');
   setMessage(response.message+' The reported state below determines completion.');
  }catch(error){
   const text=error?.message||String(error);setMessage(text);
   if(/timeout|outcome.*unknown|connection closed|disconnected|transport unavailable/i.test(text))setUncertain(true);
  }finally{setBusy(null);}
 }
 const target=!view.fresh||data?.state!=='READY'?'No current observation':data.target_detected===true?'Target detected (reported)':data.target_detected===false?'No target detected (reported)':'Not reported';
 const images=data?.processed_image_receipts_current;
 const imageValues=images&&typeof images==='object'?Object.values(images):[];
 return h('section',{className:'card application-component'},
  h('div',{className:'section-head'},h('div',null,h('h2',null,component.label),h('p',{className:'application-description'},component.description)),
   h('span',{className:'pill '+(view.fresh&&data?.state==='READY'?'good':'')},data?(view.fresh?'':'Last report: ')+LABELS[data.state]:'No current report')),
  h('div',{className:'application-actions'},
   h('button',{className:'primary','aria-label':'Load '+component.label,disabled:!view.canLoad||!!busy||uncertain,onClick:()=>request('load')},busy==='load'?'Requesting load…':'Load vision'),
   h('button',{'aria-label':'Unload '+component.label,disabled:!view.canUnload||!!busy||uncertain,onClick:()=>request('unload')},busy==='unload'?'Requesting unload…':'Unload vision'),
   h('button',{onClick:()=>onNavigate('Cameras')},'Open cameras')),
  view.notice&&h('p',{className:'notice',role:'status'},view.notice),
  h('dl',{className:'application-report'},
   h('dt',null,'Manager report'),h('dd',null,data?.reason||'Waiting for this component report.'),
   h('dt',null,'Target observation'),h('dd',null,target),
   ...(component.id==='docking'?[h('dt',{key:'images-title'},'Processed overlay receipts'),h('dd',{key:'images-value'},!view.fresh?'No current report':imageValues.length===2?imageValues.filter(value=>value===true).length+' of 2 reported current':'Not reported')]:[]),
   h('dt',null,'Report receipt'),h('dd',null,view.age===null?'No message received':Math.max(0,view.age/1000).toFixed(1)+' seconds ago'+(view.fresh?'':' · not current'))),
  message&&h('p',{className:'command-result',role:'status'},message),
  uncertain&&h('div',{className:'notice'},h('p',null,'The request outcome was not confirmed. Do not repeatedly retry. Inspect the current report first.'),h('button',{disabled:!view.fresh||['STARTING','WAITING_FOR_DATA','STOPPING'].includes(data?.state),onClick:()=>{setUncertain(false);setMessage('Current report acknowledged. No command sent.');}},'Acknowledge current report')),
  h('p',{className:'note'},component.id==='docking'?'Receiving observations requires alignment and both processed camera outputs according to the manager. No target can be a valid result. This is not docking permission.':'Receiving observations means recent alignment output and expected publishers according to the manager. It is not permission to move.'),
  data&&h('details',null,h('summary',null,'Diagnostic details'),h('dl',{className:'application-report'},
   h('dt',null,'Process ID'),h('dd',null,data.pid??'No owned process'),
   h('dt',null,'Vision log'),h('dd',null,data.log_file||'No log yet'),
   h('dt',null,'Publisher identity'),h('dd',null,data.publisher_identity?.state||'Not reported'),
   h('dt',null,'Automatic restart'),h('dd',null,data.automatic_restart===false?'Disabled':'Not confirmed'))));
}

export function ApplicationsPanel({statusRow,dockingStatusRow,connected,profile,services,now,onTrigger,onNavigate}){
 return h('div',{className:'applications-page'},
  h('section',{className:'card'},h('h2',null,'Perception applications'),h('p',null,'Load or unload each perception pipeline independently. Both may run together on this simulation platform.'),h('p',{className:'note'},'Stop the platform before loading or unloading. These controls do not start controllers, enable automatic motion, operate the lift, or load a demo.')),
  ...COMPONENTS.map(component=>h(VisionRow,{key:component.id,component,statusRow:component.id==='front_markers'?statusRow:dockingStatusRow,connected,profile,services,now,onTrigger,onNavigate})),
  h('section',{className:'card'},h('h3',null,'Demo execution remains separate'),h('p',null,'Preparing and starting a complete automatic demo is not implemented by these perception controls. HTTP camera-preview reliability is a separate display issue; manager readiness is not capture-freshness or physical-safety confirmation.')));
}
