#!/usr/bin/env python3
"""Opt-in replacement manager for TWO perception components; never run with old manager.
No controller, gate, scene, supervisor, lift actuation or velocity publication.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix,get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from swas_msgs.msg import BxuAlignmentOutput
from swas_system.bxu.publisher_identity_policy import PublisherIdentityWindow
from swas_system.bxu.vision_component_registry import SPECS,build_command


def recent(at,now,limit=1.0):return at is not None and 0 <= now-at <= limit

class ManagedVision:
    def __init__(self,node,spec):
        self.node,self.spec=node,spec
        self.process=None;self.log_handle=None;self.log_path=None
        self.state='UNLOADED';self.reason='No owned application process'
        self.started_at=None;self.last_alignment_at=None;self.target_detected=None
        self.image_at={topic:None for topic in spec.required_images}
        self.identity=None;self.last_conflict=None;self.window=None
        self.stop_at=None;self.stop_level=0;self.stop_outcome='UNLOADED'
        self.pub=node.create_publisher(String,spec.status_topic,1)
        node.create_subscription(BxuAlignmentOutput,spec.alignment,self.alignment_cb,qos_profile_sensor_data)
        for topic in spec.required_images:
            node.create_subscription(Image,topic,lambda msg,t=topic:self.image_cb(t,msg),qos_profile_sensor_data)
        prefix='/bxu/runtime/components/'+spec.id
        node.create_service(Trigger,prefix+'/load',self.load_cb)
        node.create_service(Trigger,prefix+'/unload',self.unload_cb)

    def alive(self):return self.process is not None and self.process.poll() is None

    def alignment_cb(self,msg):
        if self.alive():
            self.last_alignment_at=time.monotonic();self.target_detected=bool(msg.markers_detected)

    def image_cb(self,topic,msg):
        if not self.alive():return
        # These exact output dimensions/encoding come from the tested simulation config.
        valid=(msg.width==640 and msg.height==480 and msg.encoding=='bgr8' and
               msg.step>=msg.width*3 and len(msg.data)>=msg.step*msg.height)
        self.image_at[topic]=time.monotonic() if valid else None

    def publishers(self):return {topic:self.node.get_publishers_info_by_topic(topic) for topic in self.spec.output_topics}

    def foreign_problem(self):
        if any(self.publishers().values()):return 'Existing output publisher discovered; external processes are not adopted or killed'
        return None

    def load_problem(self):
        problem=self.node.platform_problem(True) or self.node.ownership_problem() or self.foreign_problem()
        if problem:return problem
        if any(not self.node.get_publishers_info_by_topic(topic) for topic in self.spec.input_topics):
            return 'Required image/camera-info input publishers are not all discovered'
        return None

    def load_cb(self,request,response):
        if self.process is not None:
            response.success=self.alive() and self.state!='STOPPING'
            response.message='Already owned; no duplicate created. Inspect readiness.' if response.success else 'Previous process is exiting/stopping; wait for completion'
            return response
        try:
            problem=self.load_problem()
            if problem:raise RuntimeError(problem)
            command=build_command(self.spec,get_package_prefix('swas_system'),get_package_share_directory('swas_system'))
            logs=Path(os.environ.get('ROS_LOG_DIR',str(Path.home()/'.ros/log')))/'bxu_vision_registry_manager'
            logs.mkdir(parents=True,exist_ok=True)
            self.log_path=str(logs/(self.spec.id+'-'+str(time.time_ns())+'.log'))
            self.log_handle=open(self.log_path,'ab',buffering=0)
            self.last_alignment_at=None;self.target_detected=None
            self.image_at={topic:None for topic in self.spec.required_images}
            self.window=PublisherIdentityWindow(timeout_s=5.0);self.identity=None;self.last_conflict=None
            self.process=subprocess.Popen(command,stdout=self.log_handle,stderr=subprocess.STDOUT,start_new_session=True)
            self.started_at=time.monotonic();self.state='STARTING';self.reason='Process started; no motion permission granted'
            response.success=True;response.message='Load accepted; wait for this component to report READY'
        except Exception as error:
            if self.process is None and self.log_handle is not None:self.log_handle.close();self.log_handle=None
            self.state='FAILED';self.reason=str(error);response.success=False;response.message=self.reason
        return response

    def signal_owned(self,sig):
        if not self.alive():return
        try:os.killpg(self.process.pid,sig)
        except ProcessLookupError:pass

    def begin_stop(self,reason,outcome='UNLOADED'):
        self.state='STOPPING';self.reason=reason;self.stop_outcome=outcome
        self.stop_at=time.monotonic();self.stop_level=1;self.signal_owned(signal.SIGINT)

    def unload_cb(self,request,response):
        try:
            problem=self.node.platform_problem(False)
            if problem:raise RuntimeError(problem)
            if self.process is None:
                problem=self.foreign_problem()
                if problem:raise RuntimeError(problem)
                self.state='UNLOADED';self.reason='No owned application process'
                response.success=True;response.message=self.reason
            elif self.state=='STOPPING':response.success=True;response.message='Unload already in progress'
            else:
                self.begin_stop('Explicit unload of owned '+self.spec.label)
                response.success=True;response.message='Unload accepted; wait for UNLOADED'
        except Exception as error:response.success=False;response.message=str(error)
        return response

    def finish(self,code):
        expected=self.state=='STOPPING';self.process=None
        if self.log_handle is not None:self.log_handle.close();self.log_handle=None
        self.last_alignment_at=None;self.target_detected=None
        self.image_at={topic:None for topic in self.spec.required_images}
        self.identity=None
        if expected:
            self.state=self.stop_outcome
            if self.state=='UNLOADED':self.reason='Owned application exited; platform and other components left running'
        else:self.state='FAILED';self.reason=f'Process exited (code {code}); inspect log. No automatic restart.'

    def tick(self):
        now=time.monotonic()
        if self.process is not None:
            code=self.process.poll()
            if code is not None:self.finish(code)
            elif self.state=='STOPPING':
                if now-self.stop_at>=6 and self.stop_level<3:self.signal_owned(signal.SIGKILL);self.stop_level=3
                elif now-self.stop_at>=3 and self.stop_level<2:self.signal_owned(signal.SIGTERM);self.stop_level=2
            else:
                observed=self.publishers()
                identity=self.window.evaluate(observed,self.spec.node_name,'/',time.monotonic())
                # Presentation only; the shared identity policy itself is unchanged.
                identity=dict(identity);identity['reason']=identity['reason'].replace('both marker output publishers','all configured vision output publishers').replace('per marker output topic','per vision output topic')
                self.identity=identity;state=identity['state'];now=time.monotonic()
                if state in ('CONFLICT','IDENTITY_TIMEOUT'):
                    snapshot={topic:[{'node_name':p.node_name,'node_namespace':p.node_namespace,'endpoint_gid':str(getattr(p,'endpoint_gid','unavailable'))} for p in endpoints] for topic,endpoints in observed.items()}
                    self.last_conflict={'state':state,'reason':identity['reason'],'owned_pid':self.process.pid,'publishers_at_conflict':snapshot}
                    self.begin_stop('Publisher guard: '+identity['reason'],'FAILED')
                else:
                    alignment_ok=recent(self.last_alignment_at,now)
                    images_ok=all(recent(at,now) for at in self.image_at.values())
                    if state=='MATCHED' and alignment_ok and images_ok:
                        self.state='READY';self.reason='Recent required perception outputs received; target presence is separate. No motion permission.'
                    else:
                        receipts=[self.last_alignment_at]+list(self.image_at.values())
                        stale=any(now-(at if at is not None else self.started_at)>20 for at in receipts)
                        if now-self.started_at>20 and (state!='MATCHED' or stale):
                            self.begin_stop('Required publishers/data did not become or remain available within 20 seconds','FAILED')
                        else:
                            self.state='WAITING_FOR_DATA'
                            self.reason=identity['reason'] if state!='MATCHED' else 'Waiting for recent alignment'+(' and both processed camera overlays' if self.image_at else '')
        elif self.foreign_problem():self.state='EXTERNAL';self.reason='Output publishers are external/unmanaged; load refused'
        elif self.state=='EXTERNAL':self.state='UNLOADED';self.reason='External output publishers no longer discovered'

    def report(self):
        now=time.monotonic();alive=self.alive()
        try:problem=self.load_problem() if self.process is None else None
        except Exception as error:problem='Graph check failed: '+str(error)
        current={topic:recent(at,now) for topic,at in self.image_at.items()}
        return {
            'runtime':'vision_only_simulation_v1','component_id':self.spec.id,'state':self.state,'reason':self.reason,
            'process_running':alive,'pid':self.process.pid if alive else None,'target_detected':self.target_detected,
            'readiness':'alignment receipt'+(' and both processed overlay receipts' if self.image_at else '')+', not source-capture freshness or motion permission',
            'load_allowed':self.process is None and problem is None,'load_blocker':problem,
            'unload_allowed':self.process is not None and self.node.platform_problem(False) is None,
            'log_file':self.log_path,'controllers_started':False,'automatic_restart':False,
            'publisher_identity':self.identity,'last_publisher_conflict':self.last_conflict,
            'processed_image_receipts_current':current,
        }


class VisionRegistryManager(Node):
    def __init__(self):
        super().__init__('bxu_vision_registry_manager')
        self.platform=None;self.platform_at=None
        self.create_subscription(String,'/bxu/platform/status',self.platform_cb,1)
        self.workers={spec.id:ManagedVision(self,spec) for spec in SPECS}
        self.aggregate=self.create_publisher(String,'/bxu/runtime/components/status',1)
        self.timer=self.create_timer(0.25,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().warning('OPT-IN vision registry. Do NOT run the old bxu_component_manager simultaneously. No component starts automatically; simulation only.')

    def platform_cb(self,msg):
        try:
            data=json.loads(msg.data)
            if not isinstance(data,dict):raise ValueError('Platform report must be an object')
            self.platform,self.platform_at=data,time.monotonic()
        except (ValueError,TypeError):self.platform,self.platform_at=None,None

    def platform_problem(self,ready):
        if not recent(self.platform_at,time.monotonic(),1.5):return 'Recent manual-platform status required'
        if self.platform.get('platform')!='manual_only_simulation':return 'Manual-only simulation platform required'
        if self.platform.get('mode')!='STOP':return 'Stop platform before loading or unloading'
        if ready and self.platform.get('blockers')!=[]:return 'Resolve platform blockers before loading'
        return None

    def ownership_problem(self):
        endpoints=self.get_publishers_info_by_topic('/bxu/runtime/status')
        if len(endpoints)!=1:return 'Exactly one registry/front-status publisher required; stop the old manager'
        if endpoints[0].node_name!=self.get_name() or endpoints[0].node_namespace!=self.get_namespace():
            return 'Waiting for this registry status publisher identity; do not run old manager concurrently'
        return None

    def tick(self):
        reports={}
        for key,worker in self.workers.items():
            try:worker.tick()
            except Exception as error:
                if worker.alive():worker.begin_stop('Manager observation error: '+str(error),'FAILED')
                else:worker.state='FAILED';worker.reason='Manager observation error: '+str(error)
            report=worker.report();reports[key]=report;worker.pub.publish(String(data=json.dumps(report)))
        self.aggregate.publish(String(data=json.dumps({'runtime':'vision_registry_simulation_v1','components':reports,'controllers_started':False})))

    def destroy_node(self):
        # Signal all owned children in parallel; never search/kill by node name.
        for sig,wait in ((signal.SIGINT,2.0),(signal.SIGTERM,1.0),(signal.SIGKILL,1.0)):
            alive=[w for w in self.workers.values() if w.alive()]
            if not alive:break
            for worker in alive:worker.signal_owned(sig)
            deadline=time.monotonic()+wait
            while any(w.alive() for w in alive) and time.monotonic()<deadline:time.sleep(0.03)
        for worker in self.workers.values():
            if worker.process is not None and worker.process.poll() is not None:
                try:worker.process.wait(timeout=0)
                except subprocess.TimeoutExpired:pass
            if worker.log_handle is not None:worker.log_handle.close();worker.log_handle=None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args);node=VisionRegistryManager()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
