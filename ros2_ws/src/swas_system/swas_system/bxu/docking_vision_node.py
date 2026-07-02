import os
import socketserver
import threading
import time
from http import server

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Header

from swas_msgs.msg import BxuAlignmentOutput


class BxuDockingVisionNode(Node):
    def __init__(self):
        super().__init__('bxu_docking_vision_node')

        self.declare_parameter('alignment_topic', '/bxu/docking_vision/alignment')
        self.declare_parameter('frame_id', 'bxu_base_link')

        self.declare_parameter('cam_width', 640)
        self.declare_parameter('cam_height', 480)
        self.declare_parameter('cam_fps', 30.0)

        self.declare_parameter(
            'cam0_device',
            'libcamerasrc camera-name="/base/axi/pcie@120000/rp1/i2c@88000/ov5647@36" ! '
            'video/x-raw,width=640,height=480,format=RGB,framerate=30/1 ! '
            'videoconvert ! video/x-raw,format=BGR ! '
            'appsink drop=true max-buffers=1 sync=false'
        )
        self.declare_parameter(
            'cam1_device',
            'libcamerasrc camera-name="/base/axi/pcie@120000/rp1/i2c@80000/ov5647@36" ! '
            'video/x-raw,width=640,height=480,format=RGB,framerate=30/1 ! '
            'videoconvert ! video/x-raw,format=BGR ! '
            'appsink drop=true max-buffers=1 sync=false'
        )
        self.declare_parameter('cam_backend', 'GSTREAMER')
        
        self.declare_parameter('cal_file_cam0', 'calib/camera_0_640x480_calibration.npz')
        self.declare_parameter('cal_file_cam1', 'calib/camera_1_640x480_calibration.npz')
        self.declare_parameter('zero_file', 'calib/alignment_zero.npz')
        self.declare_parameter('zero_avg_frames', 30)

        self.declare_parameter('marker_size_mm', 9.0)
        self.declare_parameter('cam0_target_id', 0)
        self.declare_parameter('cam1_target_id', 1)
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')

        self.declare_parameter('cam0_pos_mm', [0.0, 0.0])
        self.declare_parameter('cam1_pos_mm', [600.0, 0.0])
        self.declare_parameter('cam_to_robot_r', [1.0, 0.0, 0.0, 1.0])

        self.declare_parameter('adaptive_thresh_win_size_min', 3)
        self.declare_parameter('adaptive_thresh_win_size_max', 53)
        self.declare_parameter('adaptive_thresh_win_size_step', 4)
        self.declare_parameter('max_marker_perimeter_rate', 10.0)
        self.declare_parameter('perspective_remove_pixel_per_cell', 8)

        self.declare_parameter('clahe_clip_limit', 2.0)
        self.declare_parameter('clahe_tile_grid', [8, 8])

        self.declare_parameter('publish_rate_hz', 10.0)

        self.declare_parameter('enable_mjpeg_stream', True)
        self.declare_parameter('stream_port', 5000)

        self.width = int(self.get_parameter('cam_width').value)
        self.height = int(self.get_parameter('cam_height').value)
        self.cam_fps = float(self.get_parameter('cam_fps').value)

        self.cam0_device = str(self.get_parameter('cam0_device').value)
        self.cam1_device = str(self.get_parameter('cam1_device').value)
        self.cam_backend = str(self.get_parameter('cam_backend').value).upper()

        self.pkg_share = get_package_share_directory('swas_system')

        self.cal_files = {
            0: self._resolve_path(self.get_parameter('cal_file_cam0').value),
            1: self._resolve_path(self.get_parameter('cal_file_cam1').value),
        }
        self.zero_file = self._resolve_path(self.get_parameter('zero_file').value)
        self.zero_avg_frames = int(self.get_parameter('zero_avg_frames').value)

        self.marker_size_mm = float(self.get_parameter('marker_size_mm').value)
        self.cam_targets = {
            0: int(self.get_parameter('cam0_target_id').value),
            1: int(self.get_parameter('cam1_target_id').value),
        }

        aruco_dict_name = str(self.get_parameter('aruco_dict').value)
        if not hasattr(cv2.aruco, aruco_dict_name):
            raise ValueError(f'Invalid aruco_dict: {aruco_dict_name}')
        self.aruco_dict_const = getattr(cv2.aruco, aruco_dict_name)

        self.cam0_pos = np.array(self.get_parameter('cam0_pos_mm').value, dtype=np.float64)
        self.cam1_pos = np.array(self.get_parameter('cam1_pos_mm').value, dtype=np.float64)

        r_flat = self.get_parameter('cam_to_robot_r').value
        self.cam_to_robot_r = np.array(r_flat, dtype=np.float64).reshape(2, 2)

        alignment_topic = str(self.get_parameter('alignment_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.enable_mjpeg_stream = bool(self.get_parameter('enable_mjpeg_stream').value)
        self.stream_port = int(self.get_parameter('stream_port').value)

        self.alignment_pub = self.create_publisher(BxuAlignmentOutput, alignment_topic, 10)

        self._latest_errors = {0: None, 1: None}
        self._errors_lock = threading.Lock()
        self._frame = None
        self._frame_lock = threading.Lock()

        self.get_logger().info(f'Package share: {self.pkg_share}')
        self.get_logger().info(f'Cam0 device: {self.cam0_device}')
        self.get_logger().info(f'Cam1 device: {self.cam1_device}')
        self.get_logger().info(f'Cam0 calib: {self.cal_files[0]}')
        self.get_logger().info(f'Cam1 calib: {self.cal_files[1]}')
        self.get_logger().info(f'Zero file: {self.zero_file}')

        self._load_calibrations()
        self._setup_detector()
        self._setup_zero_offsets()
        self._setup_cameras()

        if self.enable_mjpeg_stream:
            self._setup_mjpeg_stream()

        self._capture_running = True
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()

        period_s = 1.0 / publish_rate_hz
        self.timer = self.create_timer(period_s, self._on_timer)

        self.get_logger().info('BXU docking vision node started.')

    def _resolve_path(self, path_value):
        path_str = str(path_value).strip()
        if not path_str:
            return ''
        if os.path.isabs(path_str):
            return path_str
        return os.path.join(self.pkg_share, path_str)

    def _backend_flag(self):
        if self.cam_backend == 'V4L2':
            return cv2.CAP_V4L2
        if self.cam_backend == 'GSTREAMER':
            return cv2.CAP_GSTREAMER
        return cv2.CAP_ANY

    def _load_cal(self, path):
        if path and os.path.exists(path):
            cal = np.load(path)
            mtx = cal['camera_matrix'].astype(np.float32).copy()
            dist = cal['dist_coeffs'].astype(np.float32).copy()
            self.get_logger().info(
                f'Loaded calibration: {path} '
                f'(fx={mtx[0,0]:.1f}, fy={mtx[1,1]:.1f}, cx={mtx[0,2]:.1f}, cy={mtx[1,2]:.1f})'
            )
            return mtx, dist

        self.get_logger().warn(f'Calibration file missing: {path} — using defaults.')
        f = float(self.width)
        mat = np.array([
            [f, 0.0, self.width / 2.0],
            [0.0, f, self.height / 2.0],
            [0.0, 0.0, 1.0],
        ], dtype=np.float32)
        return mat, np.zeros((5, 1), dtype=np.float32)

    def _load_calibrations(self):
        self.calibrations = {i: self._load_cal(self.cal_files[i]) for i in (0, 1)}
        self.undist_maps = {}

        for i in (0, 1):
            mtx, dist = self.calibrations[i]
            new_mtx, _ = cv2.getOptimalNewCameraMatrix(
                mtx, dist, (self.width, self.height), alpha=1, newImgSize=(self.width, self.height)
            )
            mapx, mapy = cv2.initUndistortRectifyMap(
                mtx, dist, None, new_mtx, (self.width, self.height), cv2.CV_32FC1
            )
            self.undist_maps[i] = (mapx, mapy)
            self.calibrations[i] = (new_mtx.astype(np.float32), dist.astype(np.float32))

    def _setup_detector(self):
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.aruco_params.adaptiveThreshWinSizeMin = int(
            self.get_parameter('adaptive_thresh_win_size_min').value
        )
        self.aruco_params.adaptiveThreshWinSizeMax = int(
            self.get_parameter('adaptive_thresh_win_size_max').value
        )
        self.aruco_params.adaptiveThreshWinSizeStep = int(
            self.get_parameter('adaptive_thresh_win_size_step').value
        )
        self.aruco_params.maxMarkerPerimeterRate = float(
            self.get_parameter('max_marker_perimeter_rate').value
        )
        self.aruco_params.perspectiveRemovePixelPerCell = int(
            self.get_parameter('perspective_remove_pixel_per_cell').value
        )
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        self.aruco_dict = cv2.aruco.Dictionary_get(self.aruco_dict_const)

        clip_limit = float(self.get_parameter('clahe_clip_limit').value)
        tile_grid_vals = self.get_parameter('clahe_tile_grid').value
        tile_grid = (int(tile_grid_vals[0]), int(tile_grid_vals[1]))
        self.clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=tile_grid)

    def _setup_zero_offsets(self):
        self.zero_offsets = {0: (0.0, 0.0), 1: (0.0, 0.0)}
        if self.zero_file and os.path.exists(self.zero_file):
            z = np.load(self.zero_file)
            self.zero_offsets[0] = (float(z['cam0_zero_px_x']), float(z['cam0_zero_px_y']))
            self.zero_offsets[1] = (float(z['cam1_zero_px_x']), float(z['cam1_zero_px_y']))
            self.get_logger().info(f'Loaded zero offsets: {self.zero_offsets}')
        else:
            self.get_logger().info(f'No zero calibration file found at {self.zero_file} — using optical centre.')

    def _open_one_camera(self, device, name):
        backend = self._backend_flag()
        cap = cv2.VideoCapture(device, backend)

        if not cap.isOpened():
            cap.release()
            cap = cv2.VideoCapture(device)
        if not cap.isOpened():
            raise RuntimeError(f'Failed to open {name} camera at {device}')

        if self.cam_backend != 'GSTREAMER':
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FPS, self.cam_fps)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        actual_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = cap.get(cv2.CAP_PROP_FPS)

        self.get_logger().info(
            f'{name} opened: {device} '
            f'({actual_w:.0f}x{actual_h:.0f} @ {actual_fps:.1f} fps)'
        )
        return cap

    def _setup_cameras(self):
        self.cameras = {
            0: self._open_one_camera(self.cam0_device, 'Cam0'),
            1: self._open_one_camera(self.cam1_device, 'Cam1'),
        }

        for idx, cap in self.cameras.items():
            ok, frame = cap.read()
            if not ok or frame is None:
                raise RuntimeError(f'Initial frame grab failed for camera {idx} ({[self.cam0_device, self.cam1_device][idx]})')

        time.sleep(0.2)

    def _setup_mjpeg_stream(self):
        node_self = self

        class _Handler(server.BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_GET(self):
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
                self.end_headers()
                try:
                    while True:
                        with node_self._frame_lock:
                            jpg = node_self._frame
                        if jpg is None:
                            time.sleep(0.05)
                            continue
                        self.wfile.write(b'--FRAME\r\nContent-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n')
                        time.sleep(0.04)
                except Exception:
                    pass

        class _StreamServer(socketserver.ThreadingMixIn, server.HTTPServer):
            allow_reuse_address = True
            daemon_threads = True

        threading.Thread(
            target=_StreamServer(('0.0.0.0', self.stream_port), _Handler).serve_forever,
            daemon=True
        ).start()

    def _process_and_track(self, frame_bgr, cam_idx):
        mtx, dist = self.calibrations[cam_idx]
        target_id = self.cam_targets[cam_idx]
        fx, fy = float(mtx[0, 0]), float(mtx[1, 1])
        cx, cy = float(mtx[0, 2]), float(mtx[1, 2])
        cxi, cyi = int(round(cx)), int(round(cy))

        mapx, mapy = self.undist_maps[cam_idx]
        gray = cv2.remap(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY), mapx, mapy, cv2.INTER_LINEAR)
        bgr = cv2.remap(frame_bgr, mapx, mapy, cv2.INTER_LINEAR)
        gray = self.clahe.apply(gray)

        error = None

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_params
        )

        if ids is not None and len(ids) > 0:
            ids_flat = ids.flatten().tolist()
            for i, m_id in enumerate(ids_flat):
                if int(m_id) != target_id:
                    continue

                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    [corners[i]], self.marker_size_mm, mtx, dist
                )
                if rvecs is None or tvecs is None:
                    break

                tvec = tvecs[0][0]
                z = float(tvec[2])
                if not np.isfinite(z) or z <= 0.0:
                    break

                c = corners[i][0]
                marker_px = (int(np.mean(c[:, 0])), int(np.mean(c[:, 1])))

                err_px_x = marker_px[0] - cxi
                err_px_y = -(marker_px[1] - cyi)

                zpx, zpy = self.zero_offsets[cam_idx]
                corrected_px_x = err_px_x - zpx
                corrected_px_y = err_px_y - zpy

                err_mm_x = (corrected_px_x / fx) * z
                err_mm_y = (corrected_px_y / fy) * z

                if cam_idx == 1:
                    error = (-float(err_mm_x), float(err_mm_y))
                else:
                    error = (float(err_mm_x), -float(err_mm_y))
                break

        return bgr, error

    def _capture_loop(self):
        while self._capture_running:
            try:
                ok0, f0 = self.cameras[0].read()
                ok1, f1 = self.cameras[1].read()

                if not ok0 or f0 is None:
                    raise RuntimeError(f'Failed reading frame from {self.cam0_device}')
                if not ok1 or f1 is None:
                    raise RuntimeError(f'Failed reading frame from {self.cam1_device}')

                r0, e0 = self._process_and_track(f0, 0)
                r1, e1 = self._process_and_track(f1, 1)

                with self._errors_lock:
                    self._latest_errors[0] = e0
                    self._latest_errors[1] = e1

                if self.enable_mjpeg_stream:
                    combined = np.hstack((r0, r1))
                    ok, jpg = cv2.imencode('.jpg', combined)
                    if ok:
                        with self._frame_lock:
                            self._frame = jpg.tobytes()

            except Exception as e:
                self.get_logger().error(f'Capture loop exception: {e}')
                time.sleep(0.1)

    def _get_errors(self):
        with self._errors_lock:
            return self._latest_errors[0], self._latest_errors[1]

    def _compute_fusion(self, err0, err1):
        if err0 is None and err1 is None:
            return None

        valid = [e for e in (err0, err1) if e is not None]
        ex_cam = float(np.mean([e[0] for e in valid]))
        ey_cam = float(np.mean([e[1] for e in valid]))

        if err0 is not None and err1 is not None:
            baseline = self.cam1_pos - self.cam0_pos
            sep = float(np.linalg.norm(baseline))
            theta_err = 0.0 if sep < 1e-6 else float((err1[0] - err0[0]) / sep)
        else:
            theta_err = 0.0

        err_robot = self.cam_to_robot_r @ np.array([ex_cam, ey_cam], dtype=np.float64)
        ex_r = float(err_robot[0])
        ey_r = float(err_robot[1])
        dist_mm = float(np.sqrt(ex_r ** 2 + ey_r ** 2))

        return ex_r, ey_r, theta_err, dist_mm, len(valid)

    def _on_timer(self):
        err0, err1 = self._get_errors()
        result = self._compute_fusion(err0, err1)

        msg = BxuAlignmentOutput()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        if result is None:
            msg.forward_error_m = 0.0
            msg.lateral_error_m = 0.0
            msg.yaw_error_rad = 0.0
            msg.distance_m = -1.0
            msg.markers_detected = False
            msg.num_markers_detected = 0
            msg.visibility_class = 'NONE'
            msg.confidence = 0.0
        else:
            ex_r, ey_r, theta_err, dist_mm, num_seen = result
            msg.forward_error_m = ex_r / 1000.0
            msg.lateral_error_m = ey_r / 1000.0
            msg.yaw_error_rad = theta_err
            msg.distance_m = dist_mm / 1000.0
            msg.markers_detected = True
            msg.num_markers_detected = int(num_seen)
            if num_seen == 2:
                msg.visibility_class = 'FULL'
                msg.confidence = 1.0
            else:
                msg.visibility_class = 'PARTIAL'
                msg.confidence = 0.5

        self.alignment_pub.publish(msg)

    def destroy_node(self):
        self.get_logger().info('BXU docking vision node shutting down — stopping cameras.')
        self._capture_running = False

        if hasattr(self, '_capture_thread') and self._capture_thread.is_alive():
            self._capture_thread.join(timeout=1.0)

        for cap in getattr(self, 'cameras', {}).values():
            try:
                cap.release()
            except Exception:
                pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BxuDockingVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()