# SWAS_BOUNDED_PUBLISHER_IDENTITY_V1
# SWAS_PUBLISHER_CONFLICT_DIAGNOSTICS_V1
#!/usr/bin/env python3
"""Increment 1: allowlisted marker-vision process ONLY, in manual-only simulation.
No controllers, motion, gate, scene or supervisor are started or commanded.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger
from swas_msgs.msg import BxuAlignmentOutput
from swas_system.bxu.publisher_identity_policy import PublisherIdentityWindow

PREFIX = '/bxu/runtime/components/front_markers'
ALIGNMENT = '/bxu/front_vision/alignment'
IMAGE = '/bxu/front_vision/image_raw'
CAMERA = '/bxu/front_camera/image_raw'
INFO = '/bxu/front_camera/camera_info'
EXPECTED_NODE = 'bxu_front_vision_node'


def vision_command(prefix, share):
    executable = Path(prefix) / 'lib/swas_system/bxu_front_vision_node'
    config = Path(share) / 'config/bxu_front_vision.yaml'
    if not executable.is_file() or not os.access(executable, os.X_OK):
        raise RuntimeError('Installed vision executable missing/not executable: '+str(executable))
    if not config.is_file():
        raise RuntimeError('Vision parameter file missing: '+str(config))
    return [str(executable), '--ros-args', '--params-file', str(config),
            '-r', '__node:='+EXPECTED_NODE,
            '-p', 'use_sim_time:=true', '-p', 'input_source:=ros',
            '-p', 'image_topic:='+CAMERA, '-p', 'camera_info_topic:='+INFO,
            '-p', 'publish_overlay_image:=true']


class ComponentManager(Node):
    def __init__(self):
        super().__init__('bxu_component_manager')
        self.process = None
        self.log_handle = None
        self.log_path = None
        self.started_at = None
        self.last_alignment_at = None
        self.target_detected = None
        self.platform = None
        self.platform_at = None
        self.state = 'UNLOADED'
        self.reason = 'No managed application loaded'
        self.stop_started_at = None
        self.stop_level = 0
        self.stop_outcome = 'UNLOADED'
        self.publisher = self.create_publisher(String, '/bxu/runtime/status', 1)
        self.create_subscription(String, '/bxu/platform/status', self.platform_cb, 1)
        self.create_subscription(BxuAlignmentOutput, ALIGNMENT, self.alignment_cb, qos_profile_sensor_data)
        self.create_service(Trigger, PREFIX+'/load', self.load_cb)
        self.create_service(Trigger, PREFIX+'/unload', self.unload_cb)
        self.timer = self.create_timer(0.25, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('Marker vision manager ready. No child is started until explicit load; simulation only.')

    def platform_cb(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                raise ValueError('Expected platform object')
            self.platform, self.platform_at = value, time.monotonic()
        except (ValueError, TypeError):
            self.platform, self.platform_at = None, None

    def alignment_cb(self, msg):
        if self.process is not None and self.process.poll() is None:
            self.last_alignment_at = time.monotonic()
            self.target_detected = bool(msg.markers_detected)

    def platform_problem(self, now, require_ready):
        if self.platform_at is None or not 0 <= now-self.platform_at <= 1.5:
            return 'Recent manual-platform status required'
        if self.platform.get('platform') != 'manual_only_simulation':
            return 'Only the opt-in manual-only simulation platform is supported'
        if self.platform.get('mode') != 'STOP':
            return 'Stop the platform before loading or unloading an application'
        if require_ready and self.platform.get('blockers') != []:
            return 'Resolve platform blockers before loading vision'
        return None

    def marker_publishers(self):
        return {topic:self.get_publishers_info_by_topic(topic) for topic in (ALIGNMENT, IMAGE)}

    def external_problem(self):
        if any(self.marker_publishers().values()):
            return 'Marker-vision outputs already have a publisher. Existing/external processes are not adopted or killed.'
        return None

    def load_problem(self, now):
        problem = self.platform_problem(now, True)
        if problem:
            return problem
        problem = self.external_problem()
        if problem:
            return problem
        if not self.get_publishers_info_by_topic(CAMERA) or not self.get_publishers_info_by_topic(INFO):
            return 'Front camera and camera-info publishers must be present'
        return None

    def load_cb(self, request, response):
        if self.process is not None:
            if self.state == 'STOPPING' or self.process.poll() is not None:
                response.success, response.message = False, 'Previous owned process is stopping/exiting; wait for its status'
            else:
                response.success, response.message = True, 'Already loaded; no duplicate created. Inspect readiness status.'
            return response
        try:
            now = time.monotonic()
            problem = self.load_problem(now)
            if problem:
                raise RuntimeError(problem)
            command = vision_command(get_package_prefix('swas_system'), get_package_share_directory('swas_system'))
            logs = Path(os.environ.get('ROS_LOG_DIR', str(Path.home()/'.ros/log'))) / 'bxu_component_manager'
            logs.mkdir(parents=True, exist_ok=True)
            self.log_path = str(logs / ('front_markers-'+str(time.time_ns())+'.log'))
            self.log_handle = open(self.log_path, 'ab', buffering=0)
            self.last_alignment_at, self.target_detected = None, None
            # Installed executable and fixed args only. Never use shell=True or a browser-supplied command.
            self.process = subprocess.Popen(command, stdout=self.log_handle, stderr=subprocess.STDOUT, start_new_session=True)
            self.started_at = now
            self.state, self.reason = 'STARTING', 'Process started; awaiting alignment output. This is not motion readiness.'
            response.success, response.message = True, 'Load accepted; check /bxu/runtime/status for READY or failure'
        except Exception as error:
            if self.process is None and self.log_handle is not None:
                self.log_handle.close(); self.log_handle = None
            self.state, self.reason = 'FAILED', str(error)
            response.success, response.message = False, self.reason
        return response

    def signal_owned(self, sig):
        if self.process is None or self.process.poll() is not None:
            return
        try:
            os.killpg(self.process.pid, sig)
        except ProcessLookupError:
            pass

    def begin_stop(self, reason, outcome='UNLOADED'):
        self.stop_started_at, self.stop_level = time.monotonic(), 1
        self.stop_outcome = outcome
        self.state, self.reason = 'STOPPING', reason
        self.signal_owned(signal.SIGINT)

    def unload_cb(self, request, response):
        try:
            problem = self.platform_problem(time.monotonic(), False)
            if problem:
                raise RuntimeError(problem)
            if self.process is None:
                foreign = self.external_problem()
                if foreign:
                    raise RuntimeError(foreign)
                self.state, self.reason = 'UNLOADED', 'No owned application process'
                response.success, response.message = True, self.reason
            elif self.state == 'STOPPING':
                response.success, response.message = True, 'Unload already in progress; no duplicate operation'
            else:
                self.begin_stop('Explicit unload of owned marker vision only')
                response.success, response.message = True, 'Unload accepted; wait for UNLOADED'
        except Exception as error:
            response.success, response.message = False, str(error)
        return response

    def finish_process(self, returncode):
        expected = self.state == 'STOPPING'
        self.process = None
        if self.log_handle is not None:
            self.log_handle.close(); self.log_handle = None
        self.last_alignment_at, self.target_detected = None, None
        if expected:
            self.state = self.stop_outcome
            if self.state == 'UNLOADED':
                self.reason = 'Owned marker vision exited; scene, gate, HMI and lift left running'
        else:
            self.state, self.reason = 'FAILED', f'Vision exited unexpectedly (code {returncode}); inspect log. No automatic restart.'

    def tick(self):
        now = time.monotonic()
        if self.process is not None:
            code = self.process.poll()
            if code is not None:
                self.finish_process(code)
            elif self.state == 'STOPPING':
                elapsed = now-self.stop_started_at
                if elapsed >= 6 and self.stop_level < 3:
                    self.signal_owned(signal.SIGKILL); self.stop_level = 3
                elif elapsed >= 3 and self.stop_level < 2:
                    self.signal_owned(signal.SIGTERM); self.stop_level = 2
            else:
                publishers = self.marker_publishers()
                attempt = (self.process.pid, self.started_at)
                if getattr(self, '_publisher_identity_attempt', None) != attempt:
                    self._publisher_identity_attempt = attempt
                    self._publisher_identity_window = PublisherIdentityWindow(timeout_s=5.0)
                    self.last_publisher_conflict = None
                identity_report = self._publisher_identity_window.evaluate(
                    publishers, EXPECTED_NODE, '/', time.monotonic())
                self.publisher_identity_report = identity_report
                conflict = identity_report['state'] in ('CONFLICT', 'IDENTITY_TIMEOUT')
                if conflict:
                    observed = {}
                    for topic, endpoints in publishers.items():
                        records = []
                        for endpoint in endpoints:
                            try:
                                gid = bytes(getattr(endpoint, 'endpoint_gid', [])).hex()
                            except Exception:
                                gid = 'unavailable'
                            name = getattr(endpoint, 'node_name', None)
                            namespace = getattr(endpoint, 'node_namespace', None)
                            records.append({
                                'node_name': name,
                                'node_namespace': namespace,
                                'endpoint_gid': gid,
                                'matches_expected_identity': name == EXPECTED_NODE and namespace == '/',
                            })
                        observed[topic] = records
                    self.last_publisher_conflict = {
                        'recorded_wall_time_ns': time.time_ns(),
                        'owned_pid': self.process.pid if self.process is not None else None,
                        'expected_node': EXPECTED_NODE,
                        'expected_namespace': '/',
                        'publishers_at_conflict': observed,
                    }
                    print('BXU_MARKER_PUBLISHER_CONFLICT ' + json.dumps(self.last_publisher_conflict), flush=True)
                    self.begin_stop(('Publisher identity unresolved: '+identity_report['reason'] if identity_report['state']=='IDENTITY_TIMEOUT' else 'Conflicting marker output publisher; stopping only the owned process'), 'FAILED')
                elif identity_report['state'] != 'MATCHED':
                    if time.monotonic()-self.started_at > 20:
                        self.begin_stop('Marker output publishers did not become fully available within 20 seconds', 'FAILED')
                    else:
                        self.state, self.reason = 'WAITING_FOR_DATA', identity_report['reason']
                elif self.last_alignment_at is not None and 0 <= now-self.last_alignment_at <= 1.0 and len(publishers[ALIGNMENT]) == 1:
                    self.state, self.reason = 'READY', 'Recent alignment output received. Target presence is separate; no controller is enabled.'
                elif now-(self.last_alignment_at if self.last_alignment_at is not None else self.started_at) > 20:
                    self.begin_stop('No recent alignment output for 20 seconds; inspect log/camera pipeline', 'FAILED')
                else:
                    self.state, self.reason = 'WAITING_FOR_DATA', 'Process exists; waiting for recent alignment output'
        elif self.external_problem():
            self.state, self.reason = 'EXTERNAL', 'Marker outputs belong to an external/unmanaged publisher; load refused'
        elif self.state == 'EXTERNAL':
            self.state, self.reason = 'UNLOADED', 'External publisher no longer discovered'
        alive = self.process is not None and self.process.poll() is None
        load_problem = None if self.process is not None else self.load_problem(time.monotonic())
        status = {
                    'publisher_identity': getattr(self, 'publisher_identity_report', None),
            'last_publisher_conflict': getattr(self, 'last_publisher_conflict', None),
            'runtime':'vision_only_simulation_v1', 'component_id':'front_markers', 'state':self.state,
            'reason':self.reason, 'process_running':alive, 'pid':self.process.pid if alive else None,
            'target_detected':self.target_detected, 'readiness':'alignment receipt, not source-capture freshness or motion permission',
            'load_allowed':self.process is None and load_problem is None,
            'load_blocker':load_problem,
            'unload_allowed':self.process is not None and self.platform_problem(time.monotonic(),False) is None,
            'log_file':self.log_path, 'controllers_started':False, 'automatic_restart':False,
        }
        self.publisher.publish(String(data=json.dumps(status)))

    def destroy_node(self):
        if self.process is not None:
            for sig, wait in ((signal.SIGINT, 2.0),(signal.SIGTERM,1.0),(signal.SIGKILL,1.0)):
                if self.process.poll() is not None:
                    break
                self.signal_owned(sig)
                try:
                    self.process.wait(timeout=wait)
                except subprocess.TimeoutExpired:
                    continue
            if self.log_handle is not None:
                self.log_handle.close(); self.log_handle = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ComponentManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
