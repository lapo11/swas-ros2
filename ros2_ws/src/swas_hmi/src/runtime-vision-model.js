export const COMPONENTS=[
 {id:'front_markers',label:'Front-marker vision',description:'Front marker observations and the front-camera overlay.'},
 {id:'docking',label:'Docking vision',description:'Docking alignment and processed overlays from both upward-facing cameras.'},
].map(component=>({...component,load:'/bxu/runtime/components/'+component.id+'/load',unload:'/bxu/runtime/components/'+component.id+'/unload'}));
// Preserve existing front-only imports/tests/API.
export const LOAD=COMPONENTS[0].load;
export const UNLOAD=COMPONENTS[0].unload;
export const STATES=['UNLOADED','STARTING','WAITING_FOR_DATA','READY','STOPPING','FAILED','EXTERNAL'];
export function visionView({statusRow,connected,profile,services=[],now=Date.now(),componentId='front_markers'}){
 const component=COMPONENTS.find(item=>item.id===componentId);
 const age=statusRow?now-statusRow.at:null;
 let data=null,error='';
 if(!component)error='Unsupported component; commands unavailable.';
 try{
  const parsed=JSON.parse(statusRow?.value?.data||'null');
  if(parsed){
   if(parsed.runtime!=='vision_only_simulation_v1'||parsed.component_id!==componentId||!STATES.includes(parsed.state))error='Unsupported component report; commands unavailable.';
   else data=parsed;
  }
 }catch{error='Component report could not be read; commands unavailable.';}
 const fresh=connected&&age!==null&&age>=0&&age<=1500&&!!data&&!error;
 const loadAvailable=!!component&&services.includes(component.load),unloadAvailable=!!component&&services.includes(component.unload);
 const controllable=fresh&&profile==='sim'&&loadAvailable&&unloadAvailable;
 const canLoad=controllable&&data.load_allowed===true&&data.process_running===false&&['UNLOADED','FAILED'].includes(data.state);
 const canUnload=controllable&&data.unload_allowed===true&&data.process_running===true&&['STARTING','WAITING_FOR_DATA','READY'].includes(data.state);
 let notice=error;
 if(!notice&&!connected)notice='Station disconnected. Any displayed reports are historical.';
 else if(!notice&&profile!=='sim')notice='These application controls are simulation-only. Hardware commands remain unavailable.';
 else if(!notice&&!data)notice='No supported report received for this component. Check that the registry manager is running.';
 else if(!notice&&!fresh)notice='This component report is not current. Commands are disabled.';
 else if(!notice&&(!loadAvailable||!unloadAvailable))notice='This component’s load and unload services are not both discovered.';
 else if(!notice&&data.load_blocker)notice=data.load_blocker;
 else if(!notice&&data.state==='EXTERNAL')notice='An external publisher is present. This manager will not adopt or stop it.';
 return {component,data,fresh,age,canLoad,canUnload,notice};
}
