// Native JSON rosbridge client: no hidden subscription or offline motion queue.
export class Bridge {
 constructor({WebSocketImpl=globalThis.WebSocket,onChange=()=>{}}={}){
  this.WS=WebSocketImpl;this.changed=onChange;this.socket=null;this.serial=0;this.pending=new Map();
  this.state={link:'Disconnected',topics:[],types:[],services:[],data:{},events:[],wire:{sent:0,received:0,publishes:0},discoveryAt:0,subscriptions:{}};
  this.advertised=false;this.generation=0;
 }
 event(text){this.state.events=[{at:Date.now(),text},...this.state.events].slice(0,60);this.changed();}
 send(message){if(this.socket?.readyState!==1||this.socket.bufferedAmount>65536)return false;
  this.socket.send(JSON.stringify(message));this.state.wire.sent++;return true;}
 connect(url){this.disconnect();if(!/^wss?:\/\//.test(url)){this.event('Invalid WebSocket address.');return;}
  const g=this.generation;this.state.link='Connecting';this.socket=new this.WS(url);
  this.socket.onopen=()=>{if(g!==this.generation)return;this.state.link='Connected';this.event('WebSocket open. Discovering ROS topics/services.');this.discover();};
  this.socket.onerror=()=>{if(g===this.generation){this.state.link='Connection error';this.event('WebSocket error: inspect address, network and bridge.');}};
  this.socket.onclose=()=>{if(g===this.generation){this.state.link='Disconnected';this.state.discoveryAt=0;this.failPending('Connection closed; outcome may be unknown.');this.event('Disconnected. Last telemetry is historical.');}};
  this.socket.onmessage=e=>{if(g!==this.generation)return;try{this.receive(JSON.parse(e.data));}catch(error){this.event('Message handling error: '+error.message);}};
  this.changed();
 }
 disconnect(){this.generation++;this.discovering=false;this.failPending('Disconnected; outcome may be unknown.');
  if(this.socket){this.socket.onclose=null;this.socket.close();}this.socket=null;this.advertised=false;
  this.state={...this.state,link:'Disconnected',topics:[],types:[],services:[],data:{},discoveryAt:0,subscriptions:{},wire:{sent:0,received:0,publishes:0}};this.changed();}
 failPending(message){for(const p of this.pending.values()){clearTimeout(p.timer);p.reject(new Error(message));}this.pending.clear();}
 call(service,args={},timeout=7000){return new Promise((resolve,reject)=>{
  if(this.socket?.readyState!==1){reject(new Error('Not connected. No command queued.'));return;}
  const id='swas-call-'+(++this.serial),timer=setTimeout(()=>{this.pending.delete(id);reject(new Error('Response timeout. Command outcome unknown; do not automatically retry.'));},timeout);
  this.pending.set(id,{resolve,reject,timer});
  if(!this.send({op:'call_service',id,service,args})){clearTimeout(timer);this.pending.delete(id);reject(new Error('Transport unavailable.'));}
 });}
 receive(message){this.state.wire.received++;
  if(message.op==='publish'){
   this.state.wire.publishes++;const previous=this.state.data[message.topic],now=Date.now();
   const same=previous&&JSON.stringify(previous.value)===JSON.stringify(message.msg);
   this.state.data[message.topic]={value:message.msg,at:now,count:(previous?.count||0)+1,changedAt:same?previous.changedAt:now};
   this.state.subscriptions[message.topic]='receiving';
  }else if(message.op==='service_response'){
   const p=this.pending.get(message.id);if(p){clearTimeout(p.timer);this.pending.delete(message.id);if(message.result===false)p.reject(new Error(typeof message.values==='string'?message.values:JSON.stringify(message.values)));else p.resolve(message.values||{});}
  }else if(message.op==='status'&&(message.level==='error'||message.level==='warning')){
   this.event('rosbridge '+message.level+': '+(message.msg||JSON.stringify(message)));
   if(message.id?.startsWith('sub:'))this.state.subscriptions[message.id.slice(4)]='bridge error';
   const p=this.pending.get(message.id);if(p&&message.level==='error'){clearTimeout(p.timer);this.pending.delete(message.id);p.reject(new Error(message.msg||'Bridge rejected request'));}
  }
  this.changed();
 }
 subscribe(topic){if(this.state.subscriptions[topic])return;
  // Let the running ROS publisher supply its type; avoid guessing custom schemas.
  if(this.send({op:'subscribe',id:'sub:'+topic,topic,throttle_rate:200,queue_length:1,compression:'none'}))this.state.subscriptions[topic]='requested';
 }
 async discover(){if(this.state.link!=='Connected'||this.discovering)return;this.discovering=true;const generation=this.generation;
  try{const topics=await this.call('/rosapi/topics');if(generation!==this.generation)return;
   this.state.topics=topics.topics||[];this.state.types=topics.types||[];this.state.discoveryAt=Date.now();
   if(this.onTopics)this.onTopics(this.state.topics);
   const services=await this.call('/rosapi/services');if(generation!==this.generation)return;this.state.services=services.services||[];
  }catch(e){if(generation===this.generation)this.event('Discovery: '+e.message);}
  finally{if(generation===this.generation)this.discovering=false;this.changed();}
 }
 velocity(topic,vector){if(this.state.link!=='Connected')return false;
  if(!this.advertised){if(!this.send({op:'advertise',topic,type:'geometry_msgs/msg/Twist',latch:false,queue_size:1}))return false;this.advertised=true;}
  return this.send({op:'publish',topic,msg:{linear:{x:vector[0],y:vector[1],z:0},angular:{x:0,y:0,z:vector[2]}}});
 }
}
export function snapshotUrl(base,topic,{width=800,quality=70,stamp=Date.now()}={}){
 // This server does not decode %2F in topic query values. ROS graph names are
 // safe here after validation; encode other query fields normally.
 if(!/^\/[A-Za-z0-9_/]+$/.test(topic))throw new Error('Unsupported image topic name');
 return base.replace(/\/$/,'')+'/snapshot?topic='+topic+'&type=jpeg&width='+width+'&quality='+quality+'&qos_profile=sensor_data&_hmi='+stamp;
}
