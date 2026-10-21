import React,{useEffect,useRef,useState} from 'react';
import {snapshotUrl} from './protocol';
const h=React.createElement;
export function Camera({camera,base,available,connected,fps=4}){
 const canvas=useRef(null);const [status,setStatus]=useState('Waiting for preview'),[last,setLast]=useState(0),[now,setNow]=useState(Date.now()),[paused,setPaused]=useState(false),[visible,setVisible]=useState(!document.hidden);
 useEffect(()=>{const tick=setInterval(()=>setNow(Date.now()),500),change=()=>setVisible(!document.hidden);document.addEventListener('visibilitychange',change);return()=>{clearInterval(tick);document.removeEventListener('visibilitychange',change);};},[]);
 useEffect(()=>{setLast(0);
  if(paused||!visible){setStatus('Preview paused');return;}if(!connected){setStatus('Connect ROS bridge');return;}
  if(!available){setStatus('Camera topic not discovered');return;}if(!base){setStatus('Set video server address in System');return;}
  let dead=false,timer,deadline,img;
  const load=()=>{if(dead)return;let done=false;img=new Image();
   const finish=ok=>{if(done||dead)return;done=true;clearTimeout(deadline);
    if(ok&&canvas.current){const c=canvas.current;c.width=img.naturalWidth;c.height=img.naturalHeight;c.getContext('2d').drawImage(img,0,0);setLast(Date.now());setStatus('Preview received');}
    else setStatus('No image response — check direct preview and server log');
    timer=setTimeout(load,ok?1000/fps:1800);
   };img.onload=()=>finish(true);img.onerror=()=>finish(false);
   try{img.src=snapshotUrl(base,camera[2]);}catch(e){setStatus(e.message);return;}
   deadline=setTimeout(()=>{finish(false);img.src='';},4500);
  };load();return()=>{dead=true;clearTimeout(timer);clearTimeout(deadline);if(img){img.onload=null;img.onerror=null;img.src='';}};
 },[camera[2],base,available,connected,fps,paused,visible]);
 const fresh=last&&now-last<3500&&!paused&&visible&&connected;
 return h('section',{className:'card camera'},h('div',{className:'camera-header'},h('h3',null,camera[1]),h('button',{className:'quiet',onClick:()=>setPaused(!paused)},paused?'Resume':'Pause')),
  h('div',{className:'camera-screen'},h('canvas',{ref:canvas,style:{opacity:fresh?1:.12},'aria-label':camera[1]}),!fresh&&h('div',{className:'camera-message'},h('span',null,'▣'),h('strong',null,status),h('small',null,'Video server :8080 · separate from rosbridge'))),
  h('div',{className:'camera-footer'},h('span',null,fresh?'HTTP image received '+((now-last)/1000).toFixed(1)+'s ago':'No current preview'),h('span',null,'Up to '+fps+' fps · capture age unverified')));
}
