// HTTP preview scheduling only. Never used by ROS commands or teleop.
export function createSnapshotLimiter(maximum=3){
 let active=0,pumping=false;
 const pending=[];
 function pump(){
  if(pumping)return;pumping=true;
  try{
   while(active<maximum&&pending.length){
    const entry=pending.shift();if(entry.state!=='queued')continue;
    entry.state='active';active++;
    const done=()=>{if(entry.state!=='active')return;entry.state='done';active--;pump();};
    entry.done=done;
    try{entry.cleanup=entry.start(done);}catch(error){done();entry.onError?.(error);}
   }
  }finally{pumping=false;}
 }
 return {
  enqueue(start,onError){
   const entry={state:'queued',start,onError,cleanup:null,done:null};pending.push(entry);pump();
   return ()=>{
    if(entry.state==='queued'){entry.state='cancelled';return;}
    if(entry.state==='active'){try{entry.cleanup?.();}finally{entry.done();}}
   };
  },
  counts:()=>({active,queued:pending.filter(e=>e.state==='queued').length}),
 };
}
export const sharedSnapshotLimiter=createSnapshotLimiter(3);

export function startSnapshotLoop({url,onFrame,onState=()=>{},fps=4,timeoutMs=8000,
 limiter=sharedSnapshotLimiter,makeImage=()=>new Image(),
 setTimer=setTimeout,clearTimer=clearTimeout,
 now=()=>performance.now(),random=Math.random}){
 let stopped=false,nextTimer=null,cancelRequest=null;
 let failures=0,attempts=0,successes=0;
 const rate=Math.max(1,Math.min(10,Number(fps)||4));
 const notify=value=>{if(stopped)return;try{onState({...value,attempts,successes,failures});}catch{/* An observer must not strand a request slot. */}};
 function schedule(delay){if(!stopped)nextTimer=setTimer(request,delay);}
 function request(){
  if(stopped)return;
  notify({phase:'queued'});
  cancelRequest=limiter.enqueue(release=>{
   if(stopped){release();return ()=>{};}
   let image=null,deadline=null,settled=false;
   const began=now();attempts++;
   function detach(abort){
    if(deadline!==null)clearTimer(deadline);
    if(image){image.onload=null;image.onerror=null;if(abort){try{image.src='';}catch{}}}
   }
   function finish(ok,error=''){
    if(settled)return;settled=true;
    let worked=false,detail=error;
    try{
     if(stopped)return;
     if(ok){
      if(!image||image.naturalWidth<=0||image.naturalHeight<=0)throw new Error('Decoded image has no valid dimensions');
      onFrame(image);
      successes++;failures=0;worked=true;
      notify({phase:'received',lastSuccessAt:now(),latencyMs:Math.max(0,now()-began),error:''});
     }
    }catch(problem){detail='Preview render failed: '+(problem?.message||String(problem));}
    finally{
     detach(!worked);release();
     if(!stopped){
      if(!worked){failures++;notify({phase:'retrying',error:detail||'Image request failed; browser did not expose HTTP status'});}
      const delay=worked?1000/rate:Math.min(8000,750*Math.pow(2,Math.min(failures-1,4)))+Math.floor(random()*250);
      schedule(delay);
     }
    }
   }
   try{
    image=makeImage();image.onload=()=>finish(true);image.onerror=()=>finish(false,'Image request or decoding failed; inspect the URL/server response');
    const address=url();notify({phase:'requesting',url:address});
    deadline=setTimer(()=>finish(false,'Preview request timed out after '+timeoutMs+' ms'),timeoutMs);
    image.src=address;
   }catch(error){finish(false,'Preview request could not start: '+(error?.message||String(error)));}
   return ()=>{settled=true;detach(true);};
  },error=>{
   if(stopped)return;
   failures++;notify({phase:'retrying',error:'Preview scheduler error: '+(error?.message||String(error))});schedule(1500);
  });
 }
 request();
 return ()=>{if(stopped)return;stopped=true;if(nextTimer!==null)clearTimer(nextTimer);cancelRequest?.();};
}
