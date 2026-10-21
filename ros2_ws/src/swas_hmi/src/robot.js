import {useEffect,useRef,useState} from 'react';
import {Bridge} from './protocol';
import {TOPICS} from './config';
export function useRobot(){
 const client=useRef(null);if(!client.current)client.current=new Bridge();
 const [,paint]=useState(0);
 useEffect(()=>{const c=client.current;c.onTopics=names=>TOPICS.forEach(([,topic])=>{if(names.includes(topic))c.subscribe(topic);});
  const refresh=setInterval(()=>paint(n=>n+1),200),discovery=setInterval(()=>c.discover(),4000);
  return()=>{clearInterval(refresh);clearInterval(discovery);c.disconnect();};},[]);
 return client.current;
}
