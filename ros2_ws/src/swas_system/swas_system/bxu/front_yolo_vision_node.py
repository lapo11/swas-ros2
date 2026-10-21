"""ROS hardware/ROS-image adapter for image-space front YOLO estimation.

Metric BxuAlignmentOutput fields are intentionally INVALID placeholders.
Never connect this stage directly to a metric approach controller.
"""
import ast
import copy
from dataclasses import fields
import json
import math
from pathlib import Path
import threading
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header, String
from swas_msgs.msg import BxuAlignmentOutput, BxuFrontYoloObservation

from swas_system.bxu.front_yolo_estimator import (
    Detection, EstimatorConfig, FrontYoloEstimator, center,
)


DEFAULTS = {
    'input_source': 'device',
    'model_path': 'models/first_yolov8_24092026.onnx',
    'camera_device': '/dev/v4l/by-id/usb-openaicam_openaicam_5423534724-video-index0',
    'calib_file': 'calib/front_cam_1280x720_calib.npz',
    'frame_width': 1280, 'frame_height': 720, 'capture_fps': 30,
    'publish_rate_hz': 15.0, 'confidence_threshold': 0.22,
    'inference_iou': 0.70, 'imgsz': 640, 'max_detections': 100,
    'image_topic': '/bxu/front_camera/image_raw',
    'camera_info_topic': '/bxu/front_camera/camera_info',
    'frame_id': 'bxu_front_camera',
    'output_image_topic': '/bxu/front_yolo_vision/image_raw',
    'diagnostics_topic': '/bxu/front_yolo_vision/diagnostics',
    'alignment_topic': '/bxu/front_yolo_vision/alignment',
    'publish_overlay_image': True, 'max_frame_age_s': 1.0,
    'stream_timeout_s': 1.0,
    'observation_topic': '/bxu/front_yolo_vision/observation',
}


def image_to_bgr(message):
    channels = {'bgr8': 3, 'rgb8': 3, 'mono8': 1, 'bgra8': 4, 'rgba8': 4}
    if message.encoding not in channels:
        raise ValueError(f'Unsupported Image encoding: {message.encoding}')
    count = channels[message.encoding]
    if message.width <= 0 or message.height <= 0 or message.step < message.width * count:
        raise ValueError('Invalid Image dimensions/step.')
    raw = np.frombuffer(message.data, dtype=np.uint8)
    if raw.size < message.height * message.step:
        raise ValueError('Image data is shorter than height * step.')
    rows = raw[:message.height * message.step].reshape(message.height, message.step)
    data = rows[:, :message.width * count].reshape(message.height, message.width, count)
    if message.encoding == 'bgr8':
        return np.ascontiguousarray(data)
    conversion = {'rgb8': cv2.COLOR_RGB2BGR, 'mono8': cv2.COLOR_GRAY2BGR,
                  'bgra8': cv2.COLOR_BGRA2BGR, 'rgba8': cv2.COLOR_RGBA2BGR}
    if count == 1:
        data = data[:, :, 0]
    return cv2.cvtColor(data, conversion[message.encoding])


def detection_dict(detection):
    return {'class_id': detection.class_id, 'confidence': detection.confidence,
            'box_xyxy_px': list(detection.box)}


class BxuFrontYoloVisionNode(Node):
    def __init__(self):
        super().__init__('bxu_front_yolo_vision_node')
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)
        for field in fields(EstimatorConfig):
            self.declare_parameter(field.name, field.default)
        self.p = {name: self.get_parameter(name).value for name in DEFAULTS}
        self.config = EstimatorConfig(**{
            f.name: self.get_parameter(f.name).value for f in fields(EstimatorConfig)})
        if self.p['input_source'] not in ('device', 'ros'):
            raise ValueError('input_source must be device or ros.')
        for name in ('publish_rate_hz', 'max_frame_age_s', 'stream_timeout_s'):
            if not math.isfinite(float(self.p[name])) or self.p[name] <= 0:
                raise ValueError(f'{name} must be finite and positive.')
        if self.p['imgsz'] != 640:
            raise ValueError('This verified static ONNX model requires imgsz=640.')
        for name in ('confidence_threshold', 'inference_iou'):
            if not 0 < self.p[name] <= 1:
                raise ValueError(f'{name} must be in (0, 1].')
        if min(self.p['frame_width'], self.p['frame_height'], self.p['capture_fps'],
               self.p['max_detections']) <= 0:
            raise ValueError('Capture dimensions/rate and max_detections must be positive.')
        if self.p['alignment_topic'] == '/bxu/front_vision/alignment':
            raise ValueError('Stage-one placeholder output must not use the controller alignment topic.')
        self.stop = threading.Event()
        self.lock = threading.RLock()
        self.latest = None
        self.sequence = 0
        self.epoch = 0
        self.info = None
        self.info_signature = None
        self.last_stamp_ns = None
        self.last_arrival = None
        self.last_header = None
        self.stale = False
        self.cam = None
        self.worker = None
        self.capture_worker = None
        self.undistort_maps = None
        self.estimator = FrontYoloEstimator(self.config)
        self.image_pub = self.create_publisher(Image, self.p['output_image_topic'], 1)
        self.diag_pub = self.create_publisher(String, self.p['diagnostics_topic'], 10)
        self.alignment_pub = self.create_publisher(BxuAlignmentOutput, self.p['alignment_topic'], 10)
        self.observation_pub = self.create_publisher(BxuFrontYoloObservation, self.p['observation_topic'], 10)
        
        self.model = self._load_model()
        if self.p['input_source'] == 'ros':
            qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
            self.image_sub = self.create_subscription(Image, self.p['image_topic'], self._image_cb, qos)
            self.info_sub = self.create_subscription(CameraInfo, self.p['camera_info_topic'], self._info_cb, qos)
        else:
            self._open_device()
        self.watchdog = self.create_timer(0.25, self._watchdog,
                                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.worker = threading.Thread(target=self._process_loop, daemon=True, name='front-yolo-inference')
        self.worker.start()
        self.get_logger().info(
            f'Front YOLO started: input={self.p["input_source"]}; CPU, fixed 640x640; '
            'IMAGE-SPACE ONLY. Metric alignment placeholders are invalid; no controller integration.')

    def _path(self, value):
        path = Path(value).expanduser()
        if not path.is_absolute():
            path = Path(get_package_share_directory('swas_system')) / path
        if not path.is_file():
            raise FileNotFoundError(path)
        return path.resolve()

    def _load_model(self):
        import os
        # Do not let a running robot node modify its Python environment.
        os.environ.setdefault("YOLO_AUTOINSTALL", "false")
        import onnxruntime as ort
        from ultralytics import YOLO

        path = self._path(self.p['model_path'])
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        session = ort.InferenceSession(str(path), sess_options=options,
                                      providers=['CPUExecutionProvider'])
        inp = session.get_inputs()[0]
        if inp.shape != [1, 3, 640, 640] or inp.type != 'tensor(float)':
            raise RuntimeError(f'Unexpected ONNX input: {inp.shape}, {inp.type}')
        names = ast.literal_eval(session.get_modelmeta().custom_metadata_map.get('names', '{}'))
        if names != {0: 'car_side', 1: 'wheel'}:
            raise RuntimeError(f'Unexpected model class metadata: {names}')
        del session
        model = YOLO(str(path), task='detect')
        self.get_logger().info(f'Loading/warming local model: {path}')
        self._predict(model, np.zeros((640, 640, 3), dtype=np.uint8))
        return model

    def _predict(self, model, frame):
        return model.predict(
            source=frame,
            conf=float(self.p['confidence_threshold']),
            iou=float(self.p['inference_iou']),
            imgsz=640,
            rect=False,
            device='cpu',
            quantize=32,
            max_det=int(self.p['max_detections']),
            verbose=False,
            save=False,
            show=False,
        )[0]
    
    def _info_cb(self, message):
        k = np.asarray(message.k, dtype=float).reshape(3, 3)
        d = np.asarray(message.d, dtype=float)
        valid = (message.width > 0 and message.height > 0 and message.header.frame_id
                 and np.all(np.isfinite(k)) and k[0, 0] > 0 and k[1, 1] > 0
                 and np.all(np.isfinite(d)) and np.all(np.abs(d) <= 1e-10))
        with self.lock:
            if not valid:
                self.info = None
                self.latest = None
                self.epoch += 1
            else:
                signature = (message.width, message.height, message.header.frame_id, tuple(message.k))
                if signature != self.info_signature:
                    self.epoch += 1
                    self.latest = None
                    self.info_signature = signature
                self.info = message
        if not valid:
            self.get_logger().error('ROS CameraInfo must have finite intrinsics, a frame ID, and D=0. '
                                    'Use ideal/rectified ROS images; hardware mode handles USB rectification.',
                                    throttle_duration_sec=5.0)

    def _image_cb(self, message):
        stamp = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
        with self.lock:
            if self.last_stamp_ns is not None and stamp < self.last_stamp_ns:
                self.epoch += 1
            self.last_stamp_ns = stamp
            self._store(message, message.header)

    def _store(self, payload, header):
        with self.lock:
            self.sequence += 1
            self.last_arrival = time.monotonic()
            self.last_header = copy.deepcopy(header)
            self.latest = (self.sequence, payload, copy.deepcopy(header), self.last_arrival)
            self.stale = False

    def _open_device(self):
        device = str(Path(self.p['camera_device']).expanduser().resolve())
        cam = cv2.VideoCapture(device, cv2.CAP_V4L2)
        self.cam = cam
        cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.p['frame_width'])
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.p['frame_height'])
        cam.set(cv2.CAP_PROP_FPS, self.p['capture_fps'])
        cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        ok, frame = cam.read() if cam.isOpened() else (False, None)
        if not ok or frame is None:
            cam.release()
            raise RuntimeError(f'Could not read the front USB camera: {device}')
        height, width = frame.shape[:2]
        with np.load(self._path(self.p['calib_file']), allow_pickle=False) as calibration:
            k = np.asarray(calibration['camera_matrix'], dtype=np.float64).copy()
            d = np.asarray(calibration['dist_coeffs'], dtype=np.float64)
            cw, ch = map(int, calibration['image_size'])
        if (k.shape != (3, 3) or cw <= 0 or ch <= 0 or
                not np.all(np.isfinite(k)) or not np.all(np.isfinite(d)) or
                k[0, 0] <= 0 or k[1, 1] <= 0):
            cam.release()
            raise ValueError('Invalid hardware calibration.')
        k[0, :] *= width / cw
        k[1, :] *= height / ch
        rectified_k, _ = cv2.getOptimalNewCameraMatrix(k, d, (width, height), 0, (width, height))
        self.undistort_maps = cv2.initUndistortRectifyMap(k, d, None, rectified_k,
                                                       (width, height), cv2.CV_16SC2)
        self.device_size = (height, width)
        self.device_cx = float(rectified_k[0, 2])
        self.get_logger().info(f'USB camera: {device}; actual image {width}x{height}')
        self.capture_worker = threading.Thread(target=self._capture_loop, daemon=True,
                                               name='front-yolo-capture')
        self.capture_worker.start()

    def _capture_loop(self):
        while not self.stop.is_set():
            ok, frame = self.cam.read()
            if not ok or frame is None:
                self.get_logger().warning('No frame from front USB camera.', throttle_duration_sec=5.0)
                self.stop.wait(0.05)
                continue
            if frame.shape[:2] != self.device_size:
                self.get_logger().error('USB capture resolution changed; restart node.', throttle_duration_sec=5.0)
                continue
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.p['frame_id']
            self._store(frame, header)

    def _watchdog(self):
        with self.lock:
            if self.last_arrival is None:
                self._invalid('WAITING_FOR_IMAGE', Header())
            elif time.monotonic() - self.last_arrival > self.p['stream_timeout_s']:
                if not self.stale:
                    self.epoch += 1
                    self.latest = None
                    self.stale = True
                self._invalid('STREAM_STALE', self.last_header)

    def _invalid(self, state, header, reason=None):
        if self.stop.is_set():
            return
        self._alignment(header, False)
        payload = {'schema_version': 1, 'valid': False, 'state': state,
                   'metric_valid': False, 'source_stamp': {'sec': header.stamp.sec,
                   'nanosec': header.stamp.nanosec}, 'frame_id': header.frame_id}
        if reason:
            payload['reason'] = str(reason)
        msg = String()
        msg.data = json.dumps(payload, allow_nan=False)
        self.diag_pub.publish(msg)

    def _alignment(self, header, observed):
        # TODO: replace only after metric geometry and controller validity are validated.
        msg = BxuAlignmentOutput()
        msg.header = header
        msg.lateral_error_m = 0.0
        msg.yaw_error_rad = 0.0
        msg.distance_m = -1.0
        msg.forward_error_m = 0.0
        msg.markers_detected = False
        msg.num_markers_detected = 0
        msg.visibility_class = 'ML_IMAGE_ONLY' if observed else 'NONE'
        msg.confidence = 0.0
        self.alignment_pub.publish(msg)

    def _process_loop(self):
        consumed = -1
        epoch_seen = -1
        last_signature = None
        period = 1.0 / self.p['publish_rate_hz']
        while not self.stop.is_set():
            cycle = time.monotonic()
            with self.lock:
                item, epoch, info = self.latest, self.epoch, self.info
            if epoch != epoch_seen:
                self.estimator.reset()
                epoch_seen = epoch
                last_signature = None
            if item is None or item[0] == consumed:
                self.stop.wait(0.01)
                continue
            sequence, payload, header, received = item
            consumed = sequence
            try:
                if cycle - received > self.p['max_frame_age_s']:
                    raise ValueError('Frame expired before inference.')
                if self.p['input_source'] == 'ros':
                    if info is None:
                        raise ValueError('Waiting for valid CameraInfo.')
                    if (payload.width, payload.height) != (info.width, info.height):
                        raise ValueError('Image and CameraInfo dimensions differ.')
                    if not header.frame_id or header.frame_id != info.header.frame_id:
                        raise ValueError('Image and CameraInfo frame IDs differ or are empty.')
                    stamp = header.stamp.sec * 1000000000 + header.stamp.nanosec
                    ros_age = (self.get_clock().now().nanoseconds - stamp) / 1e9
                    if ros_age > self.p['max_frame_age_s'] or ros_age < -0.25:
                        raise ValueError(f'ROS source stamp is stale/in the future: age={ros_age:.3f}s')
                    frame = image_to_bgr(payload)
                    cx = float(info.k[2])
                else:
                    frame = cv2.remap(payload, *self.undistort_maps, interpolation=cv2.INTER_LINEAR)
                    cx = self.device_cx
                signature = (frame.shape, header.frame_id)
                if signature != last_signature:
                    self.estimator.reset()
                    last_signature = signature
                inference_start = time.monotonic()
                prediction = self._predict(self.model, frame)
                inference_ms = (time.monotonic() - inference_start) * 1000
                detections = []
                if prediction.boxes is not None:
                    boxes = prediction.boxes.xyxy.cpu().numpy()
                    classes = prediction.boxes.cls.cpu().numpy()
                    scores = prediction.boxes.conf.cpu().numpy()
                    for box, class_id, score in zip(boxes, classes, scores):
                        if int(class_id) in (0, 1):
                            detections.append(Detection(tuple(map(float, box)), float(score), int(class_id)))
                if time.monotonic() - received > self.p['max_frame_age_s']:
                    raise ValueError('Inference result expired; increase max_frame_age_s only after checking performance.')
                result = self.estimator.update(detections, received)
                diagnostics = self._diagnostics(result, header, frame, cx, inference_ms, (time.monotonic() - cycle) * 1000)
                observation = self._observation(header, frame, result, cx)

                overlay = self._overlay(frame, result, inference_ms) if self.p['publish_overlay_image'] else frame
                with self.lock:
                    if self.stop.is_set() or epoch != self.epoch or self.stale:
                        continue
                    self._alignment(header, result['valid'])
                    message = String()
                    message.data = json.dumps(diagnostics, allow_nan=False)
                    self.diag_pub.publish(message)
                    self.observation_pub.publish(observation)
                    image = Image()
                    image.header = header
                    image.height, image.width = overlay.shape[:2]
                    image.encoding = 'bgr8'
                    image.is_bigendian = 0
                    image.step = image.width * 3
                    image.data = np.ascontiguousarray(overlay).tobytes()
                    self.image_pub.publish(image)
            except Exception as exc:
                self.estimator.reset()
                self.get_logger().error(f'Front YOLO frame rejected: {exc}', throttle_duration_sec=5.0)
                with self.lock:
                    if epoch == self.epoch:
                        self._invalid('PROCESSING_ERROR', header, exc)
            self.stop.wait(max(0.0, period - (time.monotonic() - cycle)))

    def _diagnostics(self, result, header, frame, cx, inference_ms, processing_ms):
        estimated = result['ema_center']
        return {
            'schema_version': 1, 'valid': result['valid'], 'metric_valid': False,
            'state': result['state'], 'frame_id': header.frame_id,
            'source_stamp': {'sec': header.stamp.sec, 'nanosec': header.stamp.nanosec},
            'image_width': frame.shape[1], 'image_height': frame.shape[0],
            'principal_point_x_px': cx,
            'center_error_x_px': estimated[0] - cx if estimated is not None else None,
            'center_error_x_normalized': (estimated[0] - cx) / (frame.shape[1] / 2) if estimated is not None else None,
            'car_center_px': result['car_center'], 'wheel_center_px': result['wheel_center'],
            'fused_center_px': result['fused_center'], 'ema_center_px': estimated,
            'wheel_line_angle_deg': result['wheel_line_angle_deg'],
            'wheel_angle_uses_held': result['held_wheels'] > 0,
            'real_wheels': result['real_wheels'], 'held_wheels': result['held_wheels'],
            'primary_switched': result['primary_switched'],
            'primary': detection_dict(result['primary']) if result['primary'] else None,
            'detector_confidence': result['detector_confidence'],
            'car_detections': [detection_dict(d) for d in result['cars']],
            'wheel_detections': [detection_dict(d) for d in result['wheels']],
            'orphan_wheels': [detection_dict(d) for d in result['orphans']],
            'secondary_wheels': [detection_dict(d) for c in result['candidates'] for d in c['secondary']],
            'primary_wheels': [dict(detection_dict(e['detection']), held=e['held'],
                                    side=e['side'], age_frames=e['age_frames']) for e in result['wheel_entries']],
            'inference_ms': inference_ms, 'processing_ms': processing_ms,
            'wheel_only_mode': result['wheel_only_mode'],
            'confirmed_pair': result['confirmed_pair'],
        }


    def _observation(self, header, frame, result, cx):
        msg = BxuFrontYoloObservation()
        msg.header = header

        msg.valid = bool(result['valid'])
        msg.state = str(result['state'])

        msg.image_width = int(frame.shape[1])
        msg.image_height = int(frame.shape[0])
        msg.principal_point_x_px = float(cx)

        estimated = result['ema_center']
        if estimated is not None:
            msg.center_x_px = float(estimated[0])
            msg.center_y_px = float(estimated[1])
            msg.center_error_x_px = float(estimated[0] - cx)
            msg.center_error_x_normalized = float(
                (estimated[0] - cx) / (frame.shape[1] / 2.0)
            )

        angle_deg = result['wheel_line_angle_deg']
        if angle_deg is not None:
            msg.wheel_line_valid = True
            msg.wheel_line_angle_rad = math.radians(float(angle_deg))

        msg.real_wheels = int(result['real_wheels'])
        msg.held_wheels = int(result['held_wheels'])
        msg.confirmed_pair = bool(result['confirmed_pair'])
        msg.wheel_only_mode = bool(result['wheel_only_mode'])
        msg.detector_confidence = float(result['detector_confidence'])

        for entry in result['wheel_entries']:
            msg.wheel_box_xyxy_px.extend(
                float(value) for value in entry['detection'].box
            )
            msg.wheel_held.append(bool(entry['held']))
            msg.wheel_side.append(str(entry['side']))

        return msg
    

    def _overlay(self, frame, result, inference_ms):
        frame = frame.copy()
        def box(d, color, label):
            x1, y1, x2, y2 = map(int, d.box)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f'{label} {d.confidence:.2f}', (max(0, x1), max(15, y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
        def cross(point, color, size=8):
            if point is not None:
                cv2.drawMarker(frame, tuple(map(int, point)), color, cv2.MARKER_CROSS, size, 2)
        index = result['primary_index']
        for i, car in enumerate(result['cars']):
            box(car, (0, 255, 0) if i == index else (100, 100, 100), 'primary' if i == index else 'other car')
        for i, candidate in enumerate(result['candidates']):
            if i != index:
                for wheel in candidate['pair'].values():
                    box(wheel, (120, 60, 60), 'other wheel')
            for wheel in candidate['secondary']:
                box(wheel, (160, 32, 160), 'secondary')
        for wheel in result['orphans']:
            box(wheel, (160, 32, 160), 'orphan')
        for entry in result['wheel_entries']:
            box(entry['detection'], (0, 165, 255) if entry['held'] else (0, 0, 255),
                'held wheel' if entry['held'] else 'wheel')
        for point in result['history'][:-1]:
            cross(point, (0, 255, 255), 6)
        cross(result['car_center'], (255, 255, 0))
        cross(result['wheel_center'], (255, 0, 0))
        cross(result['ema_center'], (0, 0, 255), 16)
        if len(result['wheel_entries']) == 2:
            a, b = [tuple(map(int, center(e['detection'].box))) for e in result['wheel_entries']]
            cv2.line(frame, a, b, (0, 255, 0), 2)
        angle = result['wheel_line_angle_deg']
        lines = [f'YOLO IMAGE-SPACE ONLY | {result["state"]} | inference {inference_ms:.0f} ms',
                 f'Cars {len(result["cars"])} | wheels real {result["real_wheels"]}, held {result["held_wheels"]}',
                 f'Wheel image angle: {angle:.1f} deg (NOT vehicle yaw)' if angle is not None else 'Wheel image angle: unavailable']
        for i, text in enumerate(lines):
            cv2.putText(frame, text, (10, 25 + 23 * i), cv2.FONT_HERSHEY_SIMPLEX,
                        0.52, (0, 255, 255), 1, cv2.LINE_AA)
        return frame

    def destroy_node(self):
        self.stop.set()
        if self.capture_worker is not None:
            self.capture_worker.join(timeout=2.0)
        if self.cam is not None:
            self.cam.release()
        if self.worker is not None:
            self.worker.join(timeout=5.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BxuFrontYoloVisionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()