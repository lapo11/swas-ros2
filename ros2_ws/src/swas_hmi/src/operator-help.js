import React,{useState} from 'react';
const h=React.createElement;
const GUIDES=[
 {id:'connect',title:'Connect to the station',summary:'Use the correct environment and wait for actual reports.',steps:[
  'Choose the Simulation or Hardware connection profile that matches the intended system. A profile changes the connection address; it does not switch robot control mode.',
  'Select Connect. If needed, open System to check the saved bridge and camera addresses.',
  'Check the reported sequence state and command authority. A connected indicator alone does not confirm that the robot, gate or cameras are operating.'
 ]},
 {id:'start',title:'Start the marker demo',summary:'Start a running demo workflow, not a launch process.',steps:[
  'Open Operations and select demo_markers. The demo must already be running on the system.',
  'Check that the sequence reports Waiting, the simulation is unpaused, and the operating area is clear.',
  'Select Start demo and review the confirmation. This can initiate automatic movement and later battery lifting.',
  'Watch the reported sequence state. An accepted request is not evidence that the action has completed.'
 ]},
 {id:'continue',title:'Continue when the sequence requests permission',summary:'Lowering and departure require separate checks.',steps:[
  'After lifting, the confirmation permits the battery plate to lower. Check the area and the displayed operation before confirming.',
  'After lowering, the confirmation permits reverse departure. Check that the departure path is clear.',
  'Confirm only once and wait for the sequence report. If the response is missing or uncertain, do not repeatedly press the control.'
 ]},
 {id:'manual',title:'Move the BXU manually',summary:'Stop the sequence, obtain manual authority, then arm jogging.',steps:[
  'Open Manual and select Stop sequence & request manual. This resets the current sequence; it is not a pause/resume command.',
  'Wait for reported Manual authority. If authority is unavailable, do not attempt to work around the disabled controls.',
  'Begin at a low speed setting, check the area, acknowledge the checks and select Arm jogging.',
  'Hold the required direction. Release it to request zero velocity. Forward/reverse, sideways movement and rotation are separate controls.',
  'Disarm when finished. Leaving the page or losing focus also disarms the current HMI jogging session.',
  'Restart automatic operation explicitly through Operations. Loss of manual input must not automatically resume the demo.'
 ]},
 {id:'stop',title:'Understand Stop, Disarm and emergency stopping',summary:'These controls have different scopes.',steps:[
  'Stop sequence requests a supervisor stop and, when available, a command-gate stop. Check the reported response and actual movement.',
  'Disarm and Escape stop this HMI jogging input. They are not general stop commands for autonomous operation.',
  'Neither control cancels an independently running lift operation or guarantees an actuator has stopped.',
  'For hazardous or unexpected movement, use the site’s physical emergency-stop procedure. Do not depend on a browser or Wi-Fi connection.'
 ]},
 {id:'camera',title:'Choose and interpret camera views',summary:'Use current views; do not drive using an uncertain image.',steps:[
  'Open Cameras, select the views needed for the task and use Pause/Resume as appropriate. Up to ten available views can be selected.',
  'Use a lower preview rate if many views are open or the display becomes slow.',
  'A paused, unavailable or historical preview is not suitable for judging movement.',
  'The displayed HTTP receipt age is not the age of camera capture. If a view appears frozen or delayed, stop relying on it for movement and request assistance.'
 ]},
 {id:'disabled',title:'A control is unavailable or a request has no response',summary:'Read the state; do not repeatedly retry.',steps:[
  'Check the connection profile and whether the station is connected. Hardware control is intentionally unavailable in this development release.',
  'Start requires a recent Waiting report and an advancing simulation clock. Continue is available only at the two confirmation steps.',
  'Manual jogging requires reported Manual gate authority and a separate arm action.',
  'If a command times out, its outcome may be unknown. Observe the sequence and request assistance rather than assuming it failed and repeating it.',
  'If reports remain unavailable, stop the task and contact the system maintainer. Reconnecting the screen is not proof that motion has stopped.'
 ]},
 {id:'finish',title:'Finish a run or repeat the demo',summary:'Leave the station in a known state.',steps:[
  'After Done, check actual station condition and any remaining lift activity.',
  'To repeat the supplied marker workflow, select Stop sequence to return to Waiting, verify the area and then Start demo.',
  'Disarm manual controls before leaving the station. Disconnecting the HMI is not a stop command for an automatic sequence.'
 ]},
];
export function helpContext({connection,profile,mission,missionFresh,authority,authorityFresh,clockAdvancing}){
 if(connection!=='Connected')return {title:'Station connection is unavailable',message:'Select the intended profile and Connect. Do not issue movement requests until sequence and authority reports are available.',target:'System',action:'Open connection settings'};
 if(profile==='hardware')return {title:'Hardware profile: monitoring only',message:'This development release does not enable hardware control. Use approved site procedures for physical operation; the profile selector does not grant motion authority.',target:'Overview',action:'Open station overview'};
 if(!missionFresh)return {title:'No recent sequence report',message:'The screen is connected, but the current sequence is not confirmed. Check that the intended demo is running and request assistance if reports do not return.',target:'Overview',action:'Check station reports'};
 if(!authorityFresh)return {title:'No recent command-authority report',message:'Manual control is not confirmed. Missing authority information does not mean the robot is stopped. Do not arm jogging or work around disabled controls.',target:'Manual',action:'Check manual authority'};
 if(mission==='FAULT')return {title:'The supervisor reports a fault',message:'Stop the sequence using the approved controls. Do not restart until the cause has been reviewed and the operating area is safe.',target:'Operations',action:'Open operator actions'};
 if(mission==='WAIT_USER_AFTER_LIFT')return {title:'Permission to lower is required',message:'Check the plate area, then use the lowering confirmation in Operations. Confirmation permits movement; it is not a passive acknowledgement.',target:'Operations',action:'Review the requested action'};
 if(mission==='WAIT_USER_AFTER_LOWER')return {title:'Permission to depart is required',message:'Check the reverse departure path, then use the departure confirmation in Operations.',target:'Operations',action:'Review departure confirmation'};
 if(!clockAdvancing)return {title:'Simulation time is not advancing',message:'Automatic Start and Continue require an unpaused simulation. Do not interpret a stationary screen as a verified safe physical state.',target:'Overview',action:'Check station reports'};
 if(authority==='MANUAL')return {title:'Manual authority is reported',message:'Use Manual to check the area, select a low initial speed and arm jogging. Hold to request movement and release to request zero.',target:'Manual',action:'Open manual controls'};
 if(mission==='DONE')return {title:'The sequence reports completion',message:'Verify actual station condition. To repeat the marker workflow, return it to Waiting with Stop sequence before starting again.',target:'Operations',action:'Open operator actions'};
 if(mission==='WAITING')return {title:'The sequence is waiting',message:'If the intended demo is loaded and the area is clear, review Start demo in Operations. Waiting alone is not a complete safety check.',target:'Operations',action:'Open operator actions'};
 return {title:'An automatic sequence is in progress',message:'Monitor the reported operation. Stop the sequence before requesting manual authority; manual takeover is not a pause/resume operation.',target:'Operations',action:'View operator actions'};
}
export function OperatorHelp(props){
 const [query,setQuery]=useState('');
 const context=helpContext(props);
 const search=query.trim().toLowerCase();
 const guides=GUIDES.filter(g=>(g.title+' '+g.summary+' '+g.steps.join(' ')).toLowerCase().includes(search));
 return h('div',{className:'operator-help'},
  h('section',{className:'help-intro'},h('h2',null,'Operator assistance'),h('p',null,'Instructions for using this station interface. For faults or uncertain motion, follow the site procedure and contact the responsible maintainer.')),
  h('section',{className:'help-context',role:'status','aria-live':'polite'},h('span',{className:'help-caption'},'Based on the reports currently available to this screen'),h('h3',null,context.title),h('p',null,context.message),h('button',{onClick:()=>props.onNavigate(context.target)},context.action)),
  h('label',{className:'help-search'},'Find an operator task',h('input',{type:'search',value:query,onChange:e=>setQuery(e.target.value),placeholder:'Search: start, manual, stop, cameras…'})),
  h('div',{className:'help-guides'},...guides.map(g=>h('details',{key:g.id,className:'help-guide'},h('summary',null,h('span',null,g.title),h('small',null,g.summary)),h('ol',null,...g.steps.map((step,i)=>h('li',{key:i},step)))))),
  !guides.length&&h('p',null,'No matching task. Try “manual”, “stop”, “continue” or “camera”.'),
  h('section',{className:'help-scope'},h('h3',null,'Do not confuse these indications'),h('dl',null,
   h('dt',null,'Connected'),h('dd',null,'The browser has a connection to the bridge; it does not confirm the complete robot is operational.'),
   h('dt',null,'Request accepted'),h('dd',null,'A service accepted the request; check the resulting sequence state and actual operation.'),
   h('dt',null,'Waiting / zero velocity request'),h('dd',null,'A reported state or command, not confirmation of physical stopping.'),
   h('dt',null,'Image received'),h('dd',null,'The browser received an image response; source-capture freshness is not verified.'))));
}
