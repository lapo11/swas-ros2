import {test} from 'node:test';
import assert from 'node:assert/strict';
import {Bridge,snapshotUrl} from '../src/protocol.js';
class FakeSocket {
 constructor(){this.readyState=0;this.bufferedAmount=0;this.sent=[];FakeSocket.last=this;}
 send(s){this.sent.push(JSON.parse(s));}
 close(){this.readyState=3;this.onclose?.();}
 open(){this.readyState=1;this.onopen?.();}
 message(m){this.onmessage({data:JSON.stringify(m)});}
}
const immediate=()=>new Promise(resolve=>setImmediate(resolve));
test('camera topic slashes stay literal, never %2F',()=>{
 const url=snapshotUrl('http://localhost:8080/','/bxu/front_vision/image_raw',{stamp:1});
 assert.ok(url.includes('topic=/bxu/front_vision/image_raw'));assert.equal(url.includes('%2F'),false);
 assert.throws(()=>snapshotUrl('http://localhost:8080','/bad&injection=x'));
});
test('discovery sends real subscriptions and routes incoming topic data',async()=>{
 const b=new Bridge({WebSocketImpl:FakeSocket});
 b.onTopics=names=>names.forEach(t=>b.subscribe(t));
 b.connect('ws://localhost:9090');const s=FakeSocket.last;s.open();
 const topicsCall=s.sent.find(m=>m.service==='/rosapi/topics');
 s.message({op:'service_response',id:topicsCall.id,result:true,values:{topics:['/bxu/cmd_vel'],types:['geometry_msgs/msg/Twist']}});
 await immediate();
 assert.ok(s.sent.some(m=>m.op==='subscribe'&&m.topic==='/bxu/cmd_vel'));
 const servicesCall=s.sent.find(m=>m.service==='/rosapi/services');
 s.message({op:'service_response',id:servicesCall.id,result:true,values:{services:['/bxu/supervisor/start']}});
 await immediate();
 s.message({op:'publish',topic:'/bxu/cmd_vel',msg:{linear:{x:0.1}}});
 assert.equal(b.state.data['/bxu/cmd_vel'].value.linear.x,0.1);
 assert.equal(b.state.wire.publishes,1);assert.equal(b.state.data['/bxu/cmd_vel'].count,1);
 b.subscribe('/bxu/cmd_vel');assert.equal(s.sent.filter(m=>m.op==='subscribe').length,1);
 b.disconnect();
});
test('service rejection payload is not confused with transport success',async()=>{
 const b=new Bridge({WebSocketImpl:FakeSocket});b.socket=new FakeSocket();b.socket.readyState=1;b.state.link='Connected';
 const promise=b.call('/bxu/supervisor/start');const request=b.socket.sent.at(-1);
 b.receive({op:'service_response',id:request.id,result:true,values:{success:false,message:'Not WAITING'}});
 const response=await promise;assert.equal(response.success,false);b.disconnect();
});
test('offline velocity is not queued; bridge errors are visible',()=>{
 const b=new Bridge({WebSocketImpl:FakeSocket});assert.equal(b.velocity('/bxu/teleop/cmd_vel',[1,0,0]),false);
 b.receive({op:'status',level:'error',id:'sub:/bxu/odom',msg:'Type support missing'});
 assert.equal(b.state.subscriptions['/bxu/odom'],'bridge error');assert.match(b.state.events[0].text,/Type support/);
});
