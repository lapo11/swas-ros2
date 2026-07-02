# import os
# from collections import deque
# from typing import Dict, Any, List

# import cv2
# import numpy as np
# import rclpy
# from ament_index_python.packages import get_package_share_directory
# from geometry_msgs.msg import Pose
# from rclpy.node import Node
# from std_msgs.msg import Header

# from swas_msgs.msg import BxuAlignmentOutput, MarkerDetection, MarkerDetectionArray


# def load_calibration(cal_file, stream_size):
#     data = np.load(cal_file)
#     cam_mtx = data["camera_matrix"].astype(np.float32)
#     dist = data["dist_coeffs"].astype(np.float32)
#     cal_w, cal_h = map(int, data["image_size"])
#     out_w, out_h = stream_size
#     if (cal_w, cal_h) != (out_w, out_h):
#         sx = out_w / cal_w
#         sy = out_h / cal_h
#         cam_mtx = cam_mtx.copy()
#         cam_mtx[0, 0] *= sx
#         cam_mtx[1, 1] *= sy
#         cam_mtx[0, 2] *= sx
#         cam_mtx[1, 2] *= sy
#     return cam_mtx, dist, float(data["reprojection_error"])


# def classify_visibility(used_ids, distance_cm, blind_start_distance_cm):
#     ids = sorted(used_ids)
#     if not ids:
#         return 'NONE', False
#     if ids == [2, 3, 4, 5]:
#         visibility = 'FULL'
#     elif ids == [4, 5]:
#         visibility = 'CENTER_PAIR'
#     elif ids == [2, 3]:
#         visibility = 'OUTER_PAIR'
#     elif all(mid in [2, 4] for mid in ids):
#         visibility = 'LEFT_SIDE_ONLY'
#     elif all(mid in [5, 3] for mid in ids):
#         visibility = 'RIGHT_SIDE_ONLY'
#     elif len(ids) == 1:
#         visibility = 'SINGLE_MARKER'
#     else:
#         visibility = 'MIXED_PARTIAL'

#     too_close = (
#         distance_cm is not None
#         and distance_cm < blind_start_distance_cm + 5.0
#         and visibility not in ('FULL', 'CENTER_PAIR')
#     )
#     return visibility, too_close


# def confidence_from_ids(used_ids, distance_cm, solve_ok, blind_start_distance_cm):
#     if not solve_ok or not used_ids:
#         return 0.0, 'low', False

#     n = len(used_ids)
#     score = min(1.0, 0.22 * n + 0.12)

#     if distance_cm is not None and distance_cm < blind_start_distance_cm:
#         score -= 0.20

#     if n >= 3:
#         label = 'high'
#     elif n == 2:
#         label = 'medium'
#     else:
#         label = 'low'

#     score = max(0.0, min(1.0, score))
#     forward_allowed = (
#         (label == 'high') or
#         (label == 'medium' and distance_cm is not None and distance_cm > 40.0)
#     )
#     return score, label, forward_allowed


# def rvec_tvec_to_pose(rvec, tvec):
#     pose = Pose()
#     if rvec is None or tvec is None:
#         return pose

#     pose.position.x = float(tvec[0][0]) / 1000.0
#     pose.position.y = float(tvec[1][0]) / 1000.0
#     pose.position.z = float(tvec[2][0]) / 1000.0

#     R, _ = cv2.Rodrigues(rvec)
#     T = np.eye(4, dtype=np.float64)
#     T[:3, :3] = R

#     qw = np.sqrt(max(0.0, 1.0 + T[0, 0] + T[1, 1] + T[2, 2])) / 2.0
#     qx = np.sqrt(max(0.0, 1.0 + T[0, 0] - T[1, 1] - T[2, 2])) / 2.0
#     qy = np.sqrt(max(0.0, 1.0 - T[0, 0] + T[1, 1] - T[2, 2])) / 2.0
#     qz = np.sqrt(max(0.0, 1.0 - T[0, 0] - T[1, 1] + T[2, 2])) / 2.0

#     qx = np.copysign(qx, T[2, 1] - T[1, 2])
#     qy = np.copysign(qy, T[0, 2] - T[2, 0])
#     qz = np.copysign(qz, T[1, 0] - T[0, 1])

#     pose.orientation.w = float(qw)
#     pose.orientation.x = float(qx)
#     pose.orientation.y = float(qy)
#     pose.orientation.z = float(qz)
#     return pose


# class BxuFrontVisionNode(Node):
#     def __init__(self):
#         super().__init__('bxu_front_vision_node')

#         self._declare_parameters()
#         self._load_parameters()

#         self.cam_mtx, self.dist_coeffs, self.reproj_err = load_calibration(
#             self.calib_file, (self.frame_width, self.frame_height)
#         )

#         self.clahe = cv2.createCLAHE(
#             clipLimit=self.clahe_clip_limit,
#             tileGridSize=(self.clahe_tile_grid_width, self.clahe_tile_grid_height)
#         )

#         self.last_seen: Dict[int, Dict[str, Any]] = {}
#         self.pose_history = deque(maxlen=self.pose_smoothing_window)

#         self.alignment_pub = self.create_publisher(
#             BxuAlignmentOutput,
#             '/bxu/front_vision/alignment',
#             10
#         )
#         self.raw_markers_pub = self.create_publisher(
#             MarkerDetectionArray,
#             '/bxu/front_vision/raw_markers',
#             10
#         )

#         camera_device_real = os.path.realpath(self.camera_device)
#         self.cam = cv2.VideoCapture(camera_device_real, cv2.CAP_V4L2)
#         self.cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
#         self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
#         self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
#         self.cam.set(cv2.CAP_PROP_FPS, self.capture_fps)
#         self.cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)

#         if not self.cam.isOpened():
#             raise RuntimeError(f"Could not open USB camera: {camera_device_real}")

#         ok, test = self.cam.read()
#         if not ok or test is None:
#             raise RuntimeError(f"USB camera opened but no frames received from {camera_device_real}")

#         h, w = test.shape[:2]
#         self.rect_cam_mtx, _ = cv2.getOptimalNewCameraMatrix(
#             self.cam_mtx, self.dist_coeffs, (w, h), 0, (w, h)
#         )
#         self.undistort_map1, self.undistort_map2 = cv2.initUndistortRectifyMap(
#             self.cam_mtx, self.dist_coeffs, None, self.rect_cam_mtx, (w, h), cv2.CV_16SC2
#         )

#         self.aruco_dict = cv2.aruco.Dictionary_get(self.aruco_dictionary_id)
#         self.aruco_params = cv2.aruco.DetectorParameters_create()

#         self.frame_idx = 0
#         self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.process_frame)

#         self.get_logger().info(f'Front vision node started on {camera_device_real}')
#         self.get_logger().info(f'Calibration file: {self.calib_file}')
#         self.get_logger().info(f'Reprojection error: {self.reproj_err:.4f} px')

#     def _declare_parameters(self):
#         self.declare_parameter('camera_device', '/dev/v4l/by-id/usb-openaicam_openaicam_5423534724-video-index0')
#         self.declare_parameter('calib_file', 'calib/front_cam_1280x720_calib.npz')

#         self.declare_parameter('frame_width', 1280)
#         self.declare_parameter('frame_height', 720)
#         self.declare_parameter('capture_fps', 30)
#         self.declare_parameter('publish_rate_hz', 15.0)

#         self.declare_parameter('aruco_dictionary_id', 0)

#         self.declare_parameter('clahe_enabled', True)
#         self.declare_parameter('clahe_clip_limit', 2.0)
#         self.declare_parameter('clahe_tile_grid_width', 8)
#         self.declare_parameter('clahe_tile_grid_height', 8)

#         self.declare_parameter('detection_persist_frames', 4)
#         self.declare_parameter('pose_smoothing_window', 5)

#         self.declare_parameter('target_center_x_mm', 0.0)
#         self.declare_parameter('target_center_y_mm', 0.0)

#         self.declare_parameter('center_pair_only_below_cm', 45.0)
#         self.declare_parameter('center_pair_only_max_lateral_cm', 30.0)
#         self.declare_parameter('blind_start_distance_cm', 30.0)
#         self.declare_parameter('near_visible_distance_cm', 100.0)

#         self.declare_parameter('marker_ids', [2, 3, 4, 5])

#         self.declare_parameter('marker_2.size_mm', 64.0)
#         self.declare_parameter('marker_2.center_x_mm', -433.0)
#         self.declare_parameter('marker_2.center_y_mm', 0.0)

#         self.declare_parameter('marker_3.size_mm', 64.0)
#         self.declare_parameter('marker_3.center_x_mm', 442.0)
#         self.declare_parameter('marker_3.center_y_mm', 0.0)

#         self.declare_parameter('marker_4.size_mm', 66.0)
#         self.declare_parameter('marker_4.center_x_mm', -105.0)
#         self.declare_parameter('marker_4.center_y_mm', 0.0)

#         self.declare_parameter('marker_5.size_mm', 64.0)
#         self.declare_parameter('marker_5.center_x_mm', 110.0)
#         self.declare_parameter('marker_5.center_y_mm', 0.0)

#     def _load_parameters(self):
#         self.camera_device = str(self.get_parameter('camera_device').value)
#         calib_file_param = str(self.get_parameter('calib_file').value)

#         if os.path.isabs(calib_file_param):
#             self.calib_file = calib_file_param
#         else:
#             pkg_share = get_package_share_directory('swas_system')
#             self.calib_file = os.path.join(pkg_share, calib_file_param)

#         self.frame_width = int(self.get_parameter('frame_width').value)
#         self.frame_height = int(self.get_parameter('frame_height').value)
#         self.capture_fps = int(self.get_parameter('capture_fps').value)
#         self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

#         self.aruco_dictionary_id = int(self.get_parameter('aruco_dictionary_id').value)

#         self.clahe_enabled = bool(self.get_parameter('clahe_enabled').value)
#         self.clahe_clip_limit = float(self.get_parameter('clahe_clip_limit').value)
#         self.clahe_tile_grid_width = int(self.get_parameter('clahe_tile_grid_width').value)
#         self.clahe_tile_grid_height = int(self.get_parameter('clahe_tile_grid_height').value)

#         self.detection_persist_frames = int(self.get_parameter('detection_persist_frames').value)
#         self.pose_smoothing_window = int(self.get_parameter('pose_smoothing_window').value)

#         self.target_center_x_mm = float(self.get_parameter('target_center_x_mm').value)
#         self.target_center_y_mm = float(self.get_parameter('target_center_y_mm').value)

#         self.center_pair_only_below_cm = float(self.get_parameter('center_pair_only_below_cm').value)
#         self.center_pair_only_max_lateral_cm = float(self.get_parameter('center_pair_only_max_lateral_cm').value)
#         self.blind_start_distance_cm = float(self.get_parameter('blind_start_distance_cm').value)
#         self.near_visible_distance_cm = float(self.get_parameter('near_visible_distance_cm').value)

#         self.marker_ids = [int(x) for x in self.get_parameter('marker_ids').value]
#         self.markers = {}
#         for marker_id in self.marker_ids:
#             self.markers[marker_id] = {
#                 "size_mm": float(self.get_parameter(f'marker_{marker_id}.size_mm').value),
#                 "center_xy_mm": (
#                     float(self.get_parameter(f'marker_{marker_id}.center_x_mm').value),
#                     float(self.get_parameter(f'marker_{marker_id}.center_y_mm').value),
#                 ),
#             }

#     def _preprocess_for_detection(self, bgr):
#         gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
#         if self.clahe_enabled:
#             gray = self.clahe.apply(gray)
#         if gray.dtype != np.uint8:
#             gray = gray.astype(np.uint8)
#         if not gray.flags['C_CONTIGUOUS']:
#             gray = np.ascontiguousarray(gray)
#         return gray

#     def _apply_detection_persistence(self, found_now):
#         for mid, corners in found_now.items():
#             self.last_seen[mid] = {"corners": corners.copy(), "frame_idx": self.frame_idx}

#         persisted = {}
#         for mid, entry in list(self.last_seen.items()):
#             age = self.frame_idx - entry["frame_idx"]
#             if age <= self.detection_persist_frames:
#                 persisted[mid] = entry["corners"].copy()
#             else:
#                 del self.last_seen[mid]
#         return persisted

#     def _smooth_pose(self, distance_cm, yaw_rad, lateral_cm):
#         values = np.array([distance_cm, yaw_rad, lateral_cm], dtype=np.float32)
#         self.pose_history.append(values)
#         stacked = np.stack(self.pose_history, axis=0)
#         smoothed = np.mean(stacked, axis=0)
#         return [float(v) for v in smoothed]

#     def _object_corners_for_marker(self, marker_id):
#         spec = self.markers[marker_id]
#         cx, cy = spec["center_xy_mm"]
#         half = spec["size_mm"] / 2.0
#         return np.array([
#             [cx - half, cy - half, 0.0],
#             [cx + half, cy - half, 0.0],
#             [cx + half, cy + half, 0.0],
#             [cx - half, cy + half, 0.0],
#         ], dtype=np.float32)

#     def _solve_pose_from_ids(self, found, used_ids):
#         obj_pts_list, img_pts_list = [], []

#         for mid in used_ids:
#             obj_pts_list.append(self._object_corners_for_marker(mid))
#             img_pts_list.append(found[mid])

#         obj_pts = np.vstack(obj_pts_list).astype(np.float32)
#         img_pts = np.vstack(img_pts_list).astype(np.float32)

#         solve_flag = cv2.SOLVEPNP_IPPE if len(used_ids) == 1 else cv2.SOLVEPNP_ITERATIVE
#         solve_ok, rvec, tvec = cv2.solvePnP(
#             obj_pts,
#             img_pts,
#             self.rect_cam_mtx,
#             np.zeros((5, 1), dtype=np.float32),
#             flags=solve_flag
#         )

#         result = {
#             "used_ids": used_ids[:],
#             "solve_ok": False,
#             "distance_cm": None,
#             "yaw_rad": None,
#             "lateral_cm": None,
#             "rvec": None,
#             "tvec": None,
#         }

#         if not solve_ok:
#             return result
#         if not (np.all(np.isfinite(rvec)) and np.all(np.isfinite(tvec))):
#             return result
#         if float(tvec[2][0]) <= 0.0:
#             return result

#         distance_cm = float(tvec[2][0]) / 10.0
#         target_center_wall = np.array(
#             [[self.target_center_x_mm], [self.target_center_y_mm], [0.0]],
#             dtype=np.float32
#         )
#         R, _ = cv2.Rodrigues(rvec)
#         target_center_cam = (R @ target_center_wall) + tvec
#         lateral_cm = float(target_center_cam[0][0]) / 10.0
#         normal_cam = R @ np.array([0.0, 0.0, 1.0])
#         yaw_rad = float(np.arctan2(normal_cam[0], normal_cam[2]))

#         result.update({
#             "solve_ok": True,
#             "distance_cm": distance_cm,
#             "yaw_rad": yaw_rad,
#             "lateral_cm": lateral_cm,
#             "rvec": rvec,
#             "tvec": tvec,
#         })
#         return result

#     def _publish_raw_markers(self, header, found, rvec_map, tvec_map):
#         msg = MarkerDetectionArray()
#         msg.header = header
#         detections: List[MarkerDetection] = []

#         for mid in sorted(found.keys()):
#             det = MarkerDetection()
#             det.marker_id = int(mid)
#             corners = found[mid]
#             det.corners_u = [float(corners[i][0]) for i in range(4)]
#             det.corners_v = [float(corners[i][1]) for i in range(4)]
#             det.pose = rvec_tvec_to_pose(rvec_map.get(mid), tvec_map.get(mid))
#             detections.append(det)

#         msg.detections = detections
#         self.raw_markers_pub.publish(msg)

#     def _detect_markers(self, gray):
#         return cv2.aruco.detectMarkers(
#             gray,
#             self.aruco_dict,
#             parameters=self.aruco_params
#         )

#     def process_frame(self):
#         ok, frame = self.cam.read()
#         if not ok or frame is None:
#             self.get_logger().warning('No frame received from front camera')
#             return

#         self.frame_idx += 1

#         if frame.ndim == 2:
#             bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
#         else:
#             bgr = frame.copy()

#         rectified = cv2.remap(
#             bgr,
#             self.undistort_map1,
#             self.undistort_map2,
#             interpolation=cv2.INTER_LINEAR
#         )

#         gray = self._preprocess_for_detection(rectified)
#         if gray is None or gray.size == 0:
#             self.get_logger().warning('Empty grayscale frame')
#             return

#         corners, ids, _ = self._detect_markers(gray)

#         found_now = {}
#         rvec_map = {}
#         tvec_map = {}

#         if ids is not None and len(ids) > 0:
#             ids_flat = ids.flatten().tolist()
#             for i, mid in enumerate(ids_flat):
#                 if mid not in self.markers:
#                     continue

#                 c = corners[i][0].astype(np.float32)
#                 found_now[mid] = c

#                 marker_obj = self._object_corners_for_marker(mid)
#                 marker_img = c.astype(np.float32)
#                 ok_pose, rvec, tvec = cv2.solvePnP(
#                     marker_obj,
#                     marker_img,
#                     self.rect_cam_mtx,
#                     np.zeros((5, 1), dtype=np.float32),
#                     flags=cv2.SOLVEPNP_IPPE
#                 )
#                 if ok_pose:
#                     rvec_map[mid] = rvec
#                     tvec_map[mid] = tvec

#         found = self._apply_detection_persistence(found_now)
#         usable_ids = sorted(found.keys())

#         solve_ok = False
#         distance_cm = None
#         yaw_rad = None
#         lateral_cm = None
#         used_ids = []

#         if usable_ids:
#             selected_ids = usable_ids[:]

#             if 4 in found and 5 in found:
#                 pair_result = self._solve_pose_from_ids(found, [4, 5])
#                 if (
#                     pair_result["solve_ok"]
#                     and pair_result["distance_cm"] is not None
#                     and pair_result["distance_cm"] < self.center_pair_only_below_cm
#                     and abs(pair_result["lateral_cm"]) < self.center_pair_only_max_lateral_cm
#                 ):
#                     selected_ids = [4, 5]

#             pose_result = self._solve_pose_from_ids(found, selected_ids)
#             used_ids = pose_result["used_ids"][:]
#             solve_ok = pose_result["solve_ok"]
#             distance_cm = pose_result["distance_cm"]
#             yaw_rad = pose_result["yaw_rad"]
#             lateral_cm = pose_result["lateral_cm"]

#             if solve_ok:
#                 distance_cm, yaw_rad, lateral_cm = self._smooth_pose(
#                     distance_cm, yaw_rad, lateral_cm
#                 )
#             else:
#                 self.pose_history.clear()
#         else:
#             self.pose_history.clear()

#         visibility_class, _ = classify_visibility(
#             used_ids,
#             distance_cm,
#             self.blind_start_distance_cm
#         )
#         confidence_score, _, _ = confidence_from_ids(
#             used_ids,
#             distance_cm,
#             solve_ok,
#             self.blind_start_distance_cm
#         )

#         header = Header()
#         header.stamp = self.get_clock().now().to_msg()
#         header.frame_id = 'bxu_front_camera'

#         align_msg = BxuAlignmentOutput()
#         align_msg.header = header
#         align_msg.lateral_error_m = float(lateral_cm / 100.0) if solve_ok and lateral_cm is not None else 0.0
#         align_msg.yaw_error_rad = float(yaw_rad) if solve_ok and yaw_rad is not None else 0.0
#         align_msg.distance_m = float(distance_cm / 100.0) if solve_ok and distance_cm is not None else -1.0
#         align_msg.markers_detected = bool(len(usable_ids) > 0)
#         align_msg.num_markers_detected = int(len(used_ids))
#         align_msg.visibility_class = visibility_class
#         align_msg.confidence = float(confidence_score)

#         self.alignment_pub.publish(align_msg)
#         self._publish_raw_markers(header, found, rvec_map, tvec_map)

#     def destroy_node(self):
#         if hasattr(self, 'cam') and self.cam is not None:
#             self.cam.release()
#         super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)
#     node = BxuFrontVisionNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


import os
from collections import deque
from typing import Dict, Any, List


import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header


from swas_msgs.msg import BxuAlignmentOutput, MarkerDetection, MarkerDetectionArray



def load_calibration(cal_file, stream_size):
    data = np.load(cal_file)
    cam_mtx = data["camera_matrix"].astype(np.float32)
    dist = data["dist_coeffs"].astype(np.float32)
    cal_w, cal_h = map(int, data["image_size"])
    out_w, out_h = stream_size
    if (cal_w, cal_h) != (out_w, out_h):
        sx = out_w / cal_w
        sy = out_h / cal_h
        cam_mtx = cam_mtx.copy()
        cam_mtx[0, 0] *= sx
        cam_mtx[1, 1] *= sy
        cam_mtx[0, 2] *= sx
        cam_mtx[1, 2] *= sy
    return cam_mtx, dist, float(data["reprojection_error"])



def classify_visibility(used_ids, distance_cm, blind_start_distance_cm):
    ids = sorted(used_ids)
    if not ids:
        return 'NONE', False
    if ids == [2, 3, 4, 5]:
        visibility = 'FULL'
    elif ids == [4, 5]:
        visibility = 'CENTER_PAIR'
    elif ids == [2, 3]:
        visibility = 'OUTER_PAIR'
    elif all(mid in [2, 4] for mid in ids):
        visibility = 'LEFT_SIDE_ONLY'
    elif all(mid in [5, 3] for mid in ids):
        visibility = 'RIGHT_SIDE_ONLY'
    elif len(ids) == 1:
        visibility = 'SINGLE_MARKER'
    else:
        visibility = 'MIXED_PARTIAL'


    too_close = (
        distance_cm is not None
        and distance_cm < blind_start_distance_cm + 5.0
        and visibility not in ('FULL', 'CENTER_PAIR')
    )
    return visibility, too_close



def confidence_from_ids(used_ids, distance_cm, solve_ok, blind_start_distance_cm):
    if not solve_ok or not used_ids:
        return 0.0, 'low', False


    n = len(used_ids)
    score = min(1.0, 0.22 * n + 0.12)


    if distance_cm is not None and distance_cm < blind_start_distance_cm:
        score -= 0.20


    if n >= 3:
        label = 'high'
    elif n == 2:
        label = 'medium'
    else:
        label = 'low'


    score = max(0.0, min(1.0, score))
    forward_allowed = (
        (label == 'high') or
        (label == 'medium' and distance_cm is not None and distance_cm > 40.0)
    )
    return score, label, forward_allowed



def rvec_tvec_to_pose(rvec, tvec):
    pose = Pose()
    if rvec is None or tvec is None:
        return pose


    pose.position.x = float(tvec[0][0]) / 1000.0
    pose.position.y = float(tvec[1][0]) / 1000.0
    pose.position.z = float(tvec[2][0]) / 1000.0


    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R


    qw = np.sqrt(max(0.0, 1.0 + T[0, 0] + T[1, 1] + T[2, 2])) / 2.0
    qx = np.sqrt(max(0.0, 1.0 + T[0, 0] - T[1, 1] - T[2, 2])) / 2.0
    qy = np.sqrt(max(0.0, 1.0 - T[0, 0] + T[1, 1] - T[2, 2])) / 2.0
    qz = np.sqrt(max(0.0, 1.0 - T[0, 0] - T[1, 1] + T[2, 2])) / 2.0


    qx = np.copysign(qx, T[2, 1] - T[1, 2])
    qy = np.copysign(qy, T[0, 2] - T[2, 0])
    qz = np.copysign(qz, T[1, 0] - T[0, 1])


    pose.orientation.w = float(qw)
    pose.orientation.x = float(qx)
    pose.orientation.y = float(qy)
    pose.orientation.z = float(qz)
    return pose



class BxuFrontVisionNode(Node):
    def __init__(self):
        super().__init__('bxu_front_vision_node')


        self._declare_parameters()
        self._load_parameters()


        self.cam_mtx, self.dist_coeffs, self.reproj_err = load_calibration(
            self.calib_file, (self.frame_width, self.frame_height)
        )


        self.clahe = cv2.createCLAHE(
            clipLimit=self.clahe_clip_limit,
            tileGridSize=(self.clahe_tile_grid_width, self.clahe_tile_grid_height)
        )


        self.last_seen: Dict[int, Dict[str, Any]] = {}
        self.pose_history = deque(maxlen=self.pose_smoothing_window)


        self.alignment_pub = self.create_publisher(
            BxuAlignmentOutput,
            '/bxu/front_vision/alignment',
            10
        )
        self.raw_markers_pub = self.create_publisher(
            MarkerDetectionArray,
            '/bxu/front_vision/raw_markers',
            10
        )

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(
            Image,
            '/bxu/front_vision/image_raw',
            5
        )


        camera_device_real = os.path.realpath(self.camera_device)
        self.cam = cv2.VideoCapture(camera_device_real, cv2.CAP_V4L2)
        self.cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.cam.set(cv2.CAP_PROP_FPS, self.capture_fps)
        self.cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)


        if not self.cam.isOpened():
            raise RuntimeError(f"Could not open USB camera: {camera_device_real}")


        ok, test = self.cam.read()
        if not ok or test is None:
            raise RuntimeError(f"USB camera opened but no frames received from {camera_device_real}")


        h, w = test.shape[:2]
        self.rect_cam_mtx, _ = cv2.getOptimalNewCameraMatrix(
            self.cam_mtx, self.dist_coeffs, (w, h), 0, (w, h)
        )
        self.undistort_map1, self.undistort_map2 = cv2.initUndistortRectifyMap(
            self.cam_mtx, self.dist_coeffs, None, self.rect_cam_mtx, (w, h), cv2.CV_16SC2
        )


        self.aruco_dict = cv2.aruco.Dictionary_get(self.aruco_dictionary_id)
        self.aruco_params = cv2.aruco.DetectorParameters_create()


        self.frame_idx = 0
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.process_frame)


        self.get_logger().info(f'Front vision node started on {camera_device_real}')
        self.get_logger().info(f'Calibration file: {self.calib_file}')
        self.get_logger().info(f'Reprojection error: {self.reproj_err:.4f} px')
        self.get_logger().info(f'Image stream overlays enabled: {self.publish_overlay_image}')


    def _declare_parameters(self):
        self.declare_parameter('camera_device', '/dev/v4l/by-id/usb-openaicam_openaicam_5423534724-video-index0')
        self.declare_parameter('calib_file', 'calib/front_cam_1280x720_calib.npz')


        self.declare_parameter('frame_width', 1280)
        self.declare_parameter('frame_height', 720)
        self.declare_parameter('capture_fps', 30)
        self.declare_parameter('publish_rate_hz', 15.0)


        self.declare_parameter('aruco_dictionary_id', 0)


        self.declare_parameter('clahe_enabled', True)
        self.declare_parameter('clahe_clip_limit', 2.0)
        self.declare_parameter('clahe_tile_grid_width', 8)
        self.declare_parameter('clahe_tile_grid_height', 8)


        self.declare_parameter('detection_persist_frames', 4)
        self.declare_parameter('pose_smoothing_window', 5)


        self.declare_parameter('target_center_x_mm', 0.0)
        self.declare_parameter('target_center_y_mm', 0.0)


        self.declare_parameter('center_pair_only_below_cm', 45.0)
        self.declare_parameter('center_pair_only_max_lateral_cm', 30.0)
        self.declare_parameter('blind_start_distance_cm', 30.0)
        self.declare_parameter('near_visible_distance_cm', 100.0)


        self.declare_parameter('marker_ids', [2, 3, 4, 5])


        self.declare_parameter('marker_2.size_mm', 64.0)
        self.declare_parameter('marker_2.center_x_mm', -433.0)
        self.declare_parameter('marker_2.center_y_mm', 0.0)


        self.declare_parameter('marker_3.size_mm', 64.0)
        self.declare_parameter('marker_3.center_x_mm', 442.0)
        self.declare_parameter('marker_3.center_y_mm', 0.0)


        self.declare_parameter('marker_4.size_mm', 66.0)
        self.declare_parameter('marker_4.center_x_mm', -105.0)
        self.declare_parameter('marker_4.center_y_mm', 0.0)


        self.declare_parameter('marker_5.size_mm', 64.0)
        self.declare_parameter('marker_5.center_x_mm', 110.0)
        self.declare_parameter('marker_5.center_y_mm', 0.0)

        self.declare_parameter('publish_overlay_image', False)


    def _load_parameters(self):
        self.camera_device = str(self.get_parameter('camera_device').value)
        calib_file_param = str(self.get_parameter('calib_file').value)


        if os.path.isabs(calib_file_param):
            self.calib_file = calib_file_param
        else:
            pkg_share = get_package_share_directory('swas_system')
            self.calib_file = os.path.join(pkg_share, calib_file_param)


        self.frame_width = int(self.get_parameter('frame_width').value)
        self.frame_height = int(self.get_parameter('frame_height').value)
        self.capture_fps = int(self.get_parameter('capture_fps').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)


        self.aruco_dictionary_id = int(self.get_parameter('aruco_dictionary_id').value)


        self.clahe_enabled = bool(self.get_parameter('clahe_enabled').value)
        self.clahe_clip_limit = float(self.get_parameter('clahe_clip_limit').value)
        self.clahe_tile_grid_width = int(self.get_parameter('clahe_tile_grid_width').value)
        self.clahe_tile_grid_height = int(self.get_parameter('clahe_tile_grid_height').value)


        self.detection_persist_frames = int(self.get_parameter('detection_persist_frames').value)
        self.pose_smoothing_window = int(self.get_parameter('pose_smoothing_window').value)


        self.target_center_x_mm = float(self.get_parameter('target_center_x_mm').value)
        self.target_center_y_mm = float(self.get_parameter('target_center_y_mm').value)


        self.center_pair_only_below_cm = float(self.get_parameter('center_pair_only_below_cm').value)
        self.center_pair_only_max_lateral_cm = float(self.get_parameter('center_pair_only_max_lateral_cm').value)
        self.blind_start_distance_cm = float(self.get_parameter('blind_start_distance_cm').value)
        self.near_visible_distance_cm = float(self.get_parameter('near_visible_distance_cm').value)


        self.marker_ids = [int(x) for x in self.get_parameter('marker_ids').value]
        self.markers = {}
        for marker_id in self.marker_ids:
            self.markers[marker_id] = {
                "size_mm": float(self.get_parameter(f'marker_{marker_id}.size_mm').value),
                "center_xy_mm": (
                    float(self.get_parameter(f'marker_{marker_id}.center_x_mm').value),
                    float(self.get_parameter(f'marker_{marker_id}.center_y_mm').value),
                ),
            }

        self.publish_overlay_image = bool(self.get_parameter('publish_overlay_image').value)


    def _preprocess_for_detection(self, bgr):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        if self.clahe_enabled:
            gray = self.clahe.apply(gray)
        if gray.dtype != np.uint8:
            gray = gray.astype(np.uint8)
        if not gray.flags['C_CONTIGUOUS']:
            gray = np.ascontiguousarray(gray)
        return gray


    def _apply_detection_persistence(self, found_now):
        for mid, corners in found_now.items():
            self.last_seen[mid] = {"corners": corners.copy(), "frame_idx": self.frame_idx}


        persisted = {}
        for mid, entry in list(self.last_seen.items()):
            age = self.frame_idx - entry["frame_idx"]
            if age <= self.detection_persist_frames:
                persisted[mid] = entry["corners"].copy()
            else:
                del self.last_seen[mid]
        return persisted


    def _smooth_pose(self, distance_cm, yaw_rad, lateral_cm):
        values = np.array([distance_cm, yaw_rad, lateral_cm], dtype=np.float32)
        self.pose_history.append(values)
        stacked = np.stack(self.pose_history, axis=0)
        smoothed = np.mean(stacked, axis=0)
        return [float(v) for v in smoothed]


    def _object_corners_for_marker(self, marker_id):
        spec = self.markers[marker_id]
        cx, cy = spec["center_xy_mm"]
        half = spec["size_mm"] / 2.0
        return np.array([
            [cx - half, cy - half, 0.0],
            [cx + half, cy - half, 0.0],
            [cx + half, cy + half, 0.0],
            [cx - half, cy + half, 0.0],
        ], dtype=np.float32)


    def _solve_pose_from_ids(self, found, used_ids):
        obj_pts_list, img_pts_list = [], []


        for mid in used_ids:
            obj_pts_list.append(self._object_corners_for_marker(mid))
            img_pts_list.append(found[mid])


        obj_pts = np.vstack(obj_pts_list).astype(np.float32)
        img_pts = np.vstack(img_pts_list).astype(np.float32)


        solve_flag = cv2.SOLVEPNP_IPPE if len(used_ids) == 1 else cv2.SOLVEPNP_ITERATIVE
        solve_ok, rvec, tvec = cv2.solvePnP(
            obj_pts,
            img_pts,
            self.rect_cam_mtx,
            np.zeros((5, 1), dtype=np.float32),
            flags=solve_flag
        )


        result = {
            "used_ids": used_ids[:],
            "solve_ok": False,
            "distance_cm": None,
            "yaw_rad": None,
            "lateral_cm": None,
            "rvec": None,
            "tvec": None,
        }


        if not solve_ok:
            return result
        if not (np.all(np.isfinite(rvec)) and np.all(np.isfinite(tvec))):
            return result
        if float(tvec[2][0]) <= 0.0:
            return result


        distance_cm = float(tvec[2][0]) / 10.0
        target_center_wall = np.array(
            [[self.target_center_x_mm], [self.target_center_y_mm], [0.0]],
            dtype=np.float32
        )
        R, _ = cv2.Rodrigues(rvec)
        target_center_cam = (R @ target_center_wall) + tvec
        lateral_cm = float(target_center_cam[0][0]) / 10.0
        normal_cam = R @ np.array([0.0, 0.0, 1.0])
        yaw_rad = float(np.arctan2(normal_cam[0], normal_cam[2]))


        result.update({
            "solve_ok": True,
            "distance_cm": distance_cm,
            "yaw_rad": yaw_rad,
            "lateral_cm": lateral_cm,
            "rvec": rvec,
            "tvec": tvec,
        })
        return result


    def _publish_raw_markers(self, header, found, rvec_map, tvec_map):
        msg = MarkerDetectionArray()
        msg.header = header
        detections: List[MarkerDetection] = []


        for mid in sorted(found.keys()):
            det = MarkerDetection()
            det.marker_id = int(mid)
            corners = found[mid]
            det.corners_u = [float(corners[i][0]) for i in range(4)]
            det.corners_v = [float(corners[i][1]) for i in range(4)]
            det.pose = rvec_tvec_to_pose(rvec_map.get(mid), tvec_map.get(mid))
            detections.append(det)


        msg.detections = detections
        self.raw_markers_pub.publish(msg)


    def _detect_markers(self, gray):
        return cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_params
        )


    def _draw_overlay(self, rectified, found, used_ids, distance_cm, yaw_rad, lateral_cm, visibility_class, confidence_score):
        overlay = rectified.copy()

        for mid, corners in found.items():
            pts = corners.astype(np.int32)
            color = (0, 255, 0) if mid in used_ids else (0, 165, 255)
            cv2.polylines(overlay, [pts], isClosed=True, color=color, thickness=2)
            center = pts.mean(axis=0).astype(int)
            cv2.putText(
                overlay, f'ID {mid}', (center[0] - 15, center[1]),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA
            )

        lines = [
            f'visibility: {visibility_class}',
            f'confidence: {confidence_score:.2f}',
            f'distance_cm: {distance_cm:.1f}' if distance_cm is not None else 'distance_cm: n/a',
            f'yaw_deg: {np.degrees(yaw_rad):.1f}' if yaw_rad is not None else 'yaw_deg: n/a',
            f'lateral_cm: {lateral_cm:.1f}' if lateral_cm is not None else 'lateral_cm: n/a',
        ]

        y0 = 25
        for i, line in enumerate(lines):
            y = y0 + i * 22
            cv2.putText(
                overlay, line, (10, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA
            )

        return overlay


    def _publish_image(self, rectified, found, used_ids, distance_cm, yaw_rad, lateral_cm, visibility_class, confidence_score, header):
        if self.publish_overlay_image:
            output_frame = self._draw_overlay(
                rectified, found, used_ids, distance_cm, yaw_rad, lateral_cm,
                visibility_class, confidence_score
            )
        else:
            output_frame = rectified

        img_msg = self.bridge.cv2_to_imgmsg(output_frame, encoding='bgr8')
        img_msg.header = header
        self.image_pub.publish(img_msg)


    def process_frame(self):
        ok, frame = self.cam.read()
        if not ok or frame is None:
            self.get_logger().warning('No frame received from front camera')
            return


        self.frame_idx += 1


        if frame.ndim == 2:
            bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        else:
            bgr = frame.copy()


        rectified = cv2.remap(
            bgr,
            self.undistort_map1,
            self.undistort_map2,
            interpolation=cv2.INTER_LINEAR
        )


        gray = self._preprocess_for_detection(rectified)
        if gray is None or gray.size == 0:
            self.get_logger().warning('Empty grayscale frame')
            return


        corners, ids, _ = self._detect_markers(gray)


        found_now = {}
        rvec_map = {}
        tvec_map = {}


        if ids is not None and len(ids) > 0:
            ids_flat = ids.flatten().tolist()
            for i, mid in enumerate(ids_flat):
                if mid not in self.markers:
                    continue


                c = corners[i][0].astype(np.float32)
                found_now[mid] = c


                marker_obj = self._object_corners_for_marker(mid)
                marker_img = c.astype(np.float32)
                ok_pose, rvec, tvec = cv2.solvePnP(
                    marker_obj,
                    marker_img,
                    self.rect_cam_mtx,
                    np.zeros((5, 1), dtype=np.float32),
                    flags=cv2.SOLVEPNP_IPPE
                )
                if ok_pose:
                    rvec_map[mid] = rvec
                    tvec_map[mid] = tvec


        found = self._apply_detection_persistence(found_now)
        usable_ids = sorted(found.keys())


        solve_ok = False
        distance_cm = None
        yaw_rad = None
        lateral_cm = None
        used_ids = []


        if usable_ids:
            selected_ids = usable_ids[:]


            if 4 in found and 5 in found:
                pair_result = self._solve_pose_from_ids(found, [4, 5])
                if (
                    pair_result["solve_ok"]
                    and pair_result["distance_cm"] is not None
                    and pair_result["distance_cm"] < self.center_pair_only_below_cm
                    and abs(pair_result["lateral_cm"]) < self.center_pair_only_max_lateral_cm
                ):
                    selected_ids = [4, 5]


            pose_result = self._solve_pose_from_ids(found, selected_ids)
            used_ids = pose_result["used_ids"][:]
            solve_ok = pose_result["solve_ok"]
            distance_cm = pose_result["distance_cm"]
            yaw_rad = pose_result["yaw_rad"]
            lateral_cm = pose_result["lateral_cm"]


            if solve_ok:
                distance_cm, yaw_rad, lateral_cm = self._smooth_pose(
                    distance_cm, yaw_rad, lateral_cm
                )
            else:
                self.pose_history.clear()
        else:
            self.pose_history.clear()


        visibility_class, _ = classify_visibility(
            used_ids,
            distance_cm,
            self.blind_start_distance_cm
        )
        confidence_score, _, _ = confidence_from_ids(
            used_ids,
            distance_cm,
            solve_ok,
            self.blind_start_distance_cm
        )


        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'bxu_front_camera'


        align_msg = BxuAlignmentOutput()
        align_msg.header = header
        align_msg.lateral_error_m = float(lateral_cm / 100.0) if solve_ok and lateral_cm is not None else 0.0
        align_msg.yaw_error_rad = float(yaw_rad) if solve_ok and yaw_rad is not None else 0.0
        align_msg.distance_m = float(distance_cm / 100.0) if solve_ok and distance_cm is not None else -1.0
        align_msg.markers_detected = bool(len(usable_ids) > 0)
        align_msg.num_markers_detected = int(len(used_ids))
        align_msg.visibility_class = visibility_class
        align_msg.confidence = float(confidence_score)


        self.alignment_pub.publish(align_msg)
        self._publish_raw_markers(header, found, rvec_map, tvec_map)
        self._publish_image(
            rectified, found, used_ids, distance_cm, yaw_rad, lateral_cm,
            visibility_class, confidence_score, header
        )


    def destroy_node(self):
        if hasattr(self, 'cam') and self.cam is not None:
            self.cam.release()
        super().destroy_node()



def main(args=None):
    rclpy.init(args=args)
    node = BxuFrontVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()