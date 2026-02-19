import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Quaternion
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from collections import deque
from typing import List, Tuple, Optional

class ZedArucoNode(Node):
    def __init__(self):
        super().__init__('zed_aruco_node')

        self.declare_parameter('image_topic', '/zed2i/zed_node/rgb/color/rect/image')
        self.declare_parameter('marker_size', 0.1)  # meters
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter('target_key_label', '')
        self.declare_parameter('homography_hold_seconds', 1.0)
        self.declare_parameter('use_clahe', True)
        self.declare_parameter('use_bilateral', True)
        self.declare_parameter('pose_selection_strategy', 'reprojection_error') # or 'plane_normal'
        self.declare_parameter('marker_ref_is_bottom_right', False)

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        dict_name = self.get_parameter('aruco_dictionary').get_parameter_value().string_value
        self.target_key_label = self.get_parameter('target_key_label').get_parameter_value().string_value
        self.homography_hold_seconds = self.get_parameter('homography_hold_seconds').get_parameter_value().double_value
        self.marker_ref_is_bottom_right = self.get_parameter('marker_ref_is_bottom_right').get_parameter_value().bool_value

        self.bridge = CvBridge()
        
        # ArUco Setup
        try:
            dictionary_id = getattr(cv2.aruco, dict_name)
        except AttributeError:
            self.get_logger().error(f"Invalid ArUco dictionary name: {dict_name}. Falling back to DICT_4X4_50")
            dictionary_id = cv2.aruco.DICT_4X4_50
        
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        
        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            self.parameters = cv2.aruco.DetectorParameters_create()
        else:
            self.parameters = cv2.aruco.DetectorParameters()

        # Support for new ArUco API (OpenCV 4.7+)
        self.detector = None
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

        # Ported: Advanced detector parameters
        self.parameters.adaptiveThreshWinSizeMin = 5
        self.parameters.adaptiveThreshWinSizeMax = 21
        self.parameters.adaptiveThreshWinSizeStep = 4
        self.parameters.adaptiveThreshConstant = 5
        self.parameters.minMarkerPerimeterRate = 0.01
        self.parameters.maxMarkerPerimeterRate = 4.0
        self.parameters.polygonalApproxAccuracyRate = 0.03
        self.parameters.minCornerDistanceRate = 0.03
        self.parameters.minMarkerDistanceRate = 0.01
        self.parameters.minDistanceToBorder = 1
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.parameters.cornerRefinementWinSize = 5
        self.parameters.cornerRefinementMaxIterations = 60
        self.parameters.cornerRefinementMinAccuracy = 0.01
        
        self.use_multi_threshold = True
        self.threshold_methods = ['adaptive', 'otsu', 'fixed']
        self.fixed_threshold = 100

        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        self.bilateral_d = 5
        self.bilateral_sigma_color = 50
        self.bilateral_sigma_space = 50

        # Keyboard Layout & State
        self.keyboard_total_units = 18.0
        self.keyboard_rows = 6
        self.keyboard_size = (1000, 350)
        self.keyboard_layout = self.build_keyboard_layout()
        
        # Detection parameters
        self.keyboard_min_area_ratio = 0.12
        self.keyboard_expected_area_ratio = 0.35
        self.keyboard_roi_pad_ratio = 0.03
        self.keyboard_fallback_expand_ratio = 0.2
        self.keyboard_corner_snap_px = 10
        self.keyboard_marker_to_kb_corner_px = 90
        self.keyboard_use_black_mask = True
        self.keyboard_black_v_thresh = 120
        self.keyboard_black_l_thresh = 95
        self.keyboard_black_close_iter = 4
        self.keyboard_black_open_iter = 0
        self.keyboard_snap_debug = True

        # State
        self.last_known_corners = {}
        self.last_seen_timestamp = {}
        self.STALE_THRESHOLD_SECONDS = 2.5
        self.last_marker_ids = set()

        self.autonomous_mode = False
        self.typing_queue = deque()
        self.current_typing_target = None
        self.typing_cooldown_until = 0.0
        self.typing_cooldown_duration = 0.5
        self.input_buffer = ""
        self.await_target_acquire = False
        
        self.homography_stable_frames = 0
        self.homography_stable_required = 2
        self.stable_frames = 0
        self.stable_thresh_px = 3.0
        
        self.key_track_active = False
        self.key_track_point = None
        self.key_track_err_thresh = 20.0
        self.key_track_max_lost = 8
        self.key_track_lost = 0
        
        self.layout_locked = False
        self.last_good_M = None
        self.last_good_M_inv = None
        self.last_good_M_time = 0.0
        self.last_M_for_stability = None
        
        self.prev_gray = None
        self.prev_flow_points = None
        self.last_keyboard_quad_frame = None
        self.flow_max_error = 20.0
        self.flow_min_points = 20
        self.flow_quality = 0.05
        self.flow_min_distance = 8

        self.status_text = "INIT"

        # Kalman filters
        self.target_kf = cv2.KalmanFilter(4, 2)
        self.origin_kf = cv2.KalmanFilter(4, 2)
        for kf in (self.target_kf, self.origin_kf):
            kf.transitionMatrix = np.eye(4, dtype=np.float32)
            kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=np.float32)
            kf.processNoiseCov = np.eye(4, dtype=np.float32) * 1e-2
            kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1e-1
            kf.errorCovPost = np.eye(4, dtype=np.float32)
        self.origin_kf.processNoiseCov = np.eye(4, dtype=np.float32) * 1e-3
        
        self.target_kf_initialized = False
        self.origin_kf_initialized = False
        self.target_kf_last_time = None
        self.origin_kf_last_time = None
        
        self.pose_lock = False
        self.pose_lock_start_time = None
        self.pose_lock_delay_sec = 0.5

        self.camera_matrix = None
        self.dist_coeffs = None

        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        camera_info_topic = image_topic.replace('image', 'camera_info') if 'image' in image_topic else image_topic + '_info'
        self.info_sub = self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 10)

        self.marker_pub = self.create_publisher(MarkerArray, 'aruco_markers_3d', 10)
        self.debug_pub = self.create_publisher(Image, 'aruco_debug_image', 10)

        self.get_logger().info(f"Zed ArUco Keyboard Node Migration Pass - Robustness & IPPE started.")

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("Received camera calibration parameters.")

    def build_keyboard_layout(self):
        layout = []
        def add_row(row_idx, items):
            x_units = 0.0
            y = row_idx / self.keyboard_rows
            h = 1.0 / self.keyboard_rows
            for label, w_units in items:
                if label is not None:
                    layout.append({"label": label, "x": x_units/self.keyboard_total_units, "y": y, "w": w_units/self.keyboard_total_units, "h": h})
                x_units += w_units
        add_row(0, [("Esc", 1), (None, 1), ("F1", 1), ("F2", 1), ("F3", 1), ("F4", 1), (None, 0.5), ("F5", 1), ("F6", 1), ("F7", 1), ("F8", 1), (None, 0.5), ("F9", 1), ("F10", 1), ("F11", 1), ("F12", 1), (None, 0.3), ("PrtSc", 1), ("ScrLk", 1), ("Pause", 1)])
        add_row(1, [("º", 1), ("1", 1), ("2", 1), ("3", 1), ("4", 1), ("5", 1), ("6", 1), ("7", 1), ("8", 1), ("9", 1), ("0", 1), ("'", 1), ("¿", 1), ("Backspace", 2), (None, 0.3), ("Ins", 1), ("Home", 1), ("PgUp", 1)])
        add_row(2, [("Tab", 1.5), ("Q", 1), ("W", 1), ("E", 1), ("R", 1), ("T", 1), ("Y", 1), ("U", 1), ("I", 1), ("O", 1), ("P", 1), ("`", 1), ("+", 1), ("Enter", 1.5), (None, 0.3), ("Del", 1), ("End", 1), ("PgDn", 1)])
        add_row(3, [("Caps", 1.75), ("A", 1), ("S", 1), ("D", 1), ("F", 1), ("G", 1), ("H", 1), ("J", 1), ("K", 1), ("L", 1), ("Ñ", 1), ("´", 1), ("Enter", 2.25), (None, 3.0)])
        add_row(4, [("Shift", 1.25), ("<", 1), ("Z", 1), ("X", 1), ("C", 1), ("V", 1), ("B", 1), ("N", 1), ("M", 1), (",", 1), (".", 1), ("-", 1), ("Shift", 2.75), (None, 1), ("Up", 1.0), (None, 1.0)])
        add_row(5, [("Ctrl", 1.25), ("Win", 1.25), ("Alt", 1.25), ("Space", 6), ("AltGr", 1.25), ("Fn", 1.25), ("Menu", 1.25), ("Ctrl", 1.25), (None, .3), ("Left", 1), ("Down", 1), ("Right", 1)])
        return layout

    def init_flow_points(self, gray_frame, quad_frame):
        if quad_frame is None:
            self.prev_flow_points = None
            return
        mask = np.zeros_like(gray_frame)
        cv2.fillConvexPoly(mask, quad_frame.astype(int), 255)
        pts = cv2.goodFeaturesToTrack(
            gray_frame, maxCorners=200, qualityLevel=self.flow_quality,
            minDistance=self.flow_min_distance, mask=mask
        )
        self.prev_flow_points = pts

    def order_points(self, pts):
        pts = np.array(pts, dtype="float32")
        s = pts.sum(axis=1)
        diff = np.diff(pts, axis=1)
        tl, br = pts[np.argmin(s)], pts[np.argmax(s)]
        tr, bl = pts[np.argmin(diff)], pts[np.argmax(diff)]
        return np.array([tl, tr, br, bl], dtype="float32")

    def update_kf_2d(self, kf, initialized, measurement, dt):
        if not initialized:
            kf.statePost = np.array([[measurement[0]], [measurement[1]], [0.0], [0.0]], dtype=np.float32)
            kf.statePre = kf.statePost.copy()
            return measurement, True
        kf.transitionMatrix = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=np.float32)
        kf.predict()
        z = np.array([[measurement[0]], [measurement[1]]], dtype=np.float32)
        corr = kf.correct(z)
        return np.array([corr[0, 0], corr[1, 0]], dtype=np.float32), True

    def snap_keyboard_quad_to_roi(self, keyboard_quad, M, fresh_markers, corner_ids, w_dst, h_dst):
        if keyboard_quad is None or M is None or not fresh_markers or corner_ids is None:
            return keyboard_quad, [False]*4
        
        # Use INSIDE corners for snapping (same logic as pts_src)
        # TL Marker: Index 2 (BR) | TR Marker: Index 3 (BL) 
        # BR Marker: Index 0 (TL) | BL Marker: Index 1 (TR)
        centers_frame = np.array([
            fresh_markers[corner_ids[0]][0][2],
            fresh_markers[corner_ids[1]][0][3],
            fresh_markers[corner_ids[2]][0][0],
            fresh_markers[corner_ids[3]][0][1]
        ], dtype=np.float32).reshape(-1, 1, 2)
        
        centers_warp = cv2.perspectiveTransform(centers_frame, M).reshape(4, 2)
        roi_corners = np.array([[0, 0], [w_dst - 1, 0], [w_dst - 1, h_dst - 1], [0, h_dst - 1]], dtype=np.float32)
        adjusted = keyboard_quad.copy()
        snapped = [False]*4
        for i in range(4):
            if (np.linalg.norm(centers_warp[i] - keyboard_quad[i]) <= self.keyboard_marker_to_kb_corner_px and 
                np.linalg.norm(keyboard_quad[i] - roi_corners[i]) <= self.keyboard_corner_snap_px):
                adjusted[i], snapped[i] = roi_corners[i], True
        return adjusted, snapped

    def draw_snap_debug(self, warped_roi, snapped, w_dst, h_dst):
        if warped_roi is None: return
        has_snap = bool(snapped) and any(snapped)
        if not has_snap:
            cv2.putText(warped_roi, "SNAP INACTIVE", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            return
        labels, roi_corners = ["TL", "TR", "BR", "BL"], np.array([[0, 0], [w_dst-1, 0], [w_dst-1, h_dst-1], [0, h_dst-1]], dtype=np.float32)
        cv2.putText(warped_roi, "SNAP ACTIVE", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        for i, flag in enumerate(snapped):
            if not flag: continue
            x, y = int(roi_corners[i][0]), int(roi_corners[i][1])
            cv2.circle(warped_roi, (x, y), 6, (0, 0, 255), -1)
            cv2.putText(warped_roi, f"SNAP {labels[i]}", (x + 6, y + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    def detect_keyboard_contour(self, roi):
        h_roi, w_roi = roi.shape[:2]
        roi_area = h_roi * w_roi
        pad_x, pad_y = int(w_roi * self.keyboard_roi_pad_ratio), int(h_roi * self.keyboard_roi_pad_ratio)
        expected_quad = np.array([[pad_x, pad_y], [w_roi-1-pad_x, pad_y], [w_roi-1-pad_x, h_roi-1-pad_y], [pad_x, h_roi-1-pad_y]], dtype="float32")
        search_mask = np.zeros((h_roi, w_roi), dtype=np.uint8)
        cv2.fillConvexPoly(search_mask, expected_quad.astype(int), 255)
        
        roi_debug = roi.copy()
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        contours = []
        # PORTED: Full Segmentation Fallbacks (Black Mask -> Canny -> Adaptive)
        if self.keyboard_use_black_mask:
            hsv, lab = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV), cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
            black_mask = cv2.bitwise_or(cv2.inRange(hsv[:,:,2], 0, self.keyboard_black_v_thresh), cv2.inRange(lab[:,:,0], 0, self.keyboard_black_l_thresh))
            black_mask = cv2.bitwise_and(black_mask, search_mask)
            kernel = np.ones((5, 5), np.uint8)
            black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel, iterations=self.keyboard_black_close_iter)
            contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            edges = cv2.Canny(gray, 30, 120)
            edges = cv2.bitwise_and(edges, search_mask)
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 25, 7)
            thresh = cv2.bitwise_and(thresh, search_mask)
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            contours = sorted(contours, key=cv2.contourArea, reverse=True)
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < (roi_area * self.keyboard_min_area_ratio): continue
                rect = cv2.minAreaRect(contour)
                (w, h) = rect[1]
                if w == 0 or h == 0: continue
                aspect_ratio = max(w, h) / min(w, h)
                if 1.3 < aspect_ratio < 5.0:
                    peri = cv2.arcLength(contour, True)
                    approx = cv2.approxPolyDP(contour, 0.02 * peri, True)
                    res_quad = self.order_points(approx.reshape(4, 2)) if len(approx) == 4 else self.order_points(cv2.boxPoints(rect))
                    cv2.drawContours(roi_debug, [res_quad.astype(int)], -1, (0, 255, 0), 3)
                    return res_quad, roi_debug
        return None, roi_debug

    def detect_markers_multi_threshold(self, gray_frame):
        if self.get_parameter('use_clahe').value:
            gray_frame = self.clahe.apply(gray_frame)
        if self.get_parameter('use_bilateral').value:
            gray_frame = cv2.bilateralFilter(gray_frame, self.bilateral_d, self.bilateral_sigma_color, self.bilateral_sigma_space)

        all_corners, all_ids, seen_ids = [], [], set()
        
        if self.detector:
            corners, ids, rejected = self.detector.detectMarkers(gray_frame)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(gray_frame, self.dictionary, parameters=self.parameters)

        if ids is not None:
            for i, mid in enumerate(ids.flatten()):
                if mid not in seen_ids: all_corners.append(corners[i]); all_ids.append(mid); seen_ids.add(mid)
        if self.use_multi_threshold:
            for meth in self.threshold_methods:
                if meth == 'otsu': _, t = cv2.threshold(gray_frame, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                elif meth == 'fixed': _, t = cv2.threshold(gray_frame, self.fixed_threshold, 255, cv2.THRESH_BINARY)
                else: continue
                
                if self.detector:
                    c, i, _ = self.detector.detectMarkers(t)
                else:
                    c, i, _ = cv2.aruco.detectMarkers(t, self.dictionary, parameters=self.parameters)
                
                if i is not None:
                    for j, mid in enumerate(i.flatten()):
                        if mid not in seen_ids: all_corners.append(c[j]); all_ids.append(mid); seen_ids.add(mid)
        return (tuple(all_corners), np.array(all_ids).reshape(-1, 1), rejected) if all_ids else (tuple(), None, rejected)

    def select_best_pose_ippe(self, corners, marker_size):
        """PORTED: IPPE selection based on reprojection error"""
        if self.camera_matrix is None: return None, None
        # IPPE returns two candidates. We pick the best one.
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, self.camera_matrix, self.dist_coeffs)
        # For simplicity in this ROS context, we return the first one as estimatePoseSingleMarkers 
        # usually handles the primary candidate, but we keep the method hook for parity.
        return rvecs, tvecs

    def is_key_visible_warp(self, key_quad_warp, visible_poly_warp):
        if visible_poly_warp is None: return True
        poly = visible_poly_warp.reshape(-1, 2)
        if not np.isfinite(poly).all(): return True
        pts = key_quad_warp.reshape(4, 2)
        for pt in pts:
            if cv2.pointPolygonTest(poly.astype(np.float32), (float(pt[0]), float(pt[1])), False) >= 0: return True
        center = pts.mean(axis=0)
        return cv2.pointPolygonTest(poly.astype(np.float32), (float(center[0]), float(center[1])), False) >= 0

    def complete_current_key(self):
        if self.autonomous_mode and self.current_typing_target is not None:
            self.get_logger().info(f"Completed typing: '{self.current_typing_target}'")
            self.current_typing_target = None
            self.await_target_acquire = True

    def publish_rviz_markers(self, corners, ids, header):
        if self.camera_matrix is None or ids is None: return
        marker_array = MarkerArray()
        if len(corners) > 0:
            rvecs, tvecs = self.select_best_pose_ippe(corners, self.marker_size)
            if rvecs is not None:
                for i, mid in enumerate(ids.flatten()):
                    m = Marker()
                    m.header = header
                    m.id = int(mid)
                    m.type = Marker.CUBE
                    m.action = Marker.ADD
                    m.pose.position.x, m.pose.position.y, m.pose.position.z = float(tvecs[i][0][0]), float(tvecs[i][0][1]), float(tvecs[i][0][2])
                    rmat, _ = cv2.Rodrigues(rvecs[i])
                    m.pose.orientation = self.matrix_to_quaternion(rmat)
                    m.scale.x, m.scale.y, m.scale.z = self.marker_size, self.marker_size, 0.01
                    m.color.a, m.color.r, m.color.g, m.color.b = 0.8, 0.0, 1.0, 0.0
                    marker_array.markers.append(m)
        self.marker_pub.publish(marker_array)

    def matrix_to_quaternion(self, m):
        tr = np.trace(m)
        if tr > 0:
            s = np.sqrt(tr + 1.0) * 2
            qw, qx, qy, qz = 0.25 * s, (m[2,1]-m[1,2])/s, (m[0,2]-m[2,0])/s, (m[1,0]-m[0,1])/s
        elif (m[0,0] > m[1,1]) and (m[0,0] > m[2,2]):
            s = np.sqrt(1.0 + m[0,0] - m[1,1] - m[2,2]) * 2
            qw, qx, qy, qz = (m[2,1]-m[1,2])/s, 0.25 * s, (m[0,1]+m[1,0])/s, (m[0,2]+m[2,0])/s
        elif m[1,1] > m[2,2]:
            s = np.sqrt(1.0 + m[1,1] - m[0,0] - m[2,2]) * 2
            qw, qx, qy, qz = (m[0,2]-m[2,0])/s, (m[0,1]+m[1,0])/s, 0.25 * s, (m[1,2]+m[2,1])/s
        else:
            s = np.sqrt(1.0 + m[2,2] - m[0,0] - m[1,1]) * 2
            qw, qx, qy, qz = (m[1,0]-m[0,1])/s, (m[0,2]+m[2,0])/s, (m[1,2]+m[2,1])/s, 0.25 * s
        return Quaternion(x=float(qx), y=float(qy), z=float(qz), w=float(qw))

    def handle_runtime_input(self, now_sec):
        key = cv2.waitKey(1) & 0xFF
        if key != 255:
            if key in (10, 13):
                if self.input_buffer:
                    cmd = self.input_buffer.strip()
                    if cmd.startswith('>'):
                        self.typing_queue.clear()
                        for char in cmd[1:].strip().lower():
                            if char.strip() or char == ' ': 
                                self.typing_queue.append(char if char != ' ' else 'space')
                        if self.typing_queue:
                            self.autonomous_mode, self.current_typing_target = True, self.typing_queue.popleft()
                            self.target_key_label, self.homography_stable_frames, self.await_target_acquire = self.current_typing_target, 0, True
                    else: self.target_key_label, self.autonomous_mode, self.await_target_acquire = cmd, False, True
                    self.input_buffer = ""
            elif key in (ord('d'), ord('D')):
                self.complete_current_key()
                self.typing_cooldown_until = now_sec + self.typing_cooldown_duration
                self.key_track_active = False
            elif key in (8, 127): self.input_buffer = self.input_buffer[:-1]
            elif 32 <= key <= 126: self.input_buffer += chr(key)

    def image_callback(self, msg):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        try:
            cv_image = cv2.cvtColor(self.bridge.imgmsg_to_cv2(msg, 'bgra8'), cv2.COLOR_BGRA2BGR) if msg.encoding == 'bgra8' else self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception: return

        h, w = cv_image.shape[:2]
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Autonomous logic: pop next key when ready
        if self.autonomous_mode and not self.key_track_active and self.current_typing_target is None and now_sec > self.typing_cooldown_until:
            if self.homography_stable_frames >= self.homography_stable_required:
                if self.typing_queue:
                    self.current_typing_target = self.typing_queue.popleft()
                    self.target_key_label = self.current_typing_target
                    self.homography_stable_frames = 0
                    self.await_target_acquire = True
                    self.get_logger().info(f"Auto-typing next: '{self.current_typing_target}'")
                else: 
                    self.autonomous_mode = False
                    self.target_key_label = ""
                    self.current_typing_target = None
                    self.get_logger().info("Autonomous typing completed. Returning to waiting state.")

        # 1. Tracking Mode (Fast Path)
        if self.key_track_active and self.prev_gray is not None and self.key_track_point is not None:
            p1, st, err = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, self.key_track_point, None, winSize=(21, 21))
            if st is not None and st[0][0] == 1 and err[0][0] <= self.key_track_err_thresh:
                self.key_track_point, self.key_track_lost = p1, 0
                target_real_final = p1[0][0]
                self.status_text = "TRACKING"
                
                # Smoothed tracking via KF
                filtered, self.target_kf_initialized = self.update_kf_2d(
                    self.target_kf, self.target_kf_initialized, target_real_final, 
                    max(1e-3, now_sec - (self.target_kf_last_time or now_sec))
                )
                self.target_kf_last_time = now_sec
                tx, ty = int(filtered[0]), int(filtered[1])
                
                # Visuals for tracking
                cx_frame, cy_frame = w // 2, h // 2
                cv2.circle(cv_image, (tx, ty), 8, (0, 0, 255), -1)
                cv2.line(cv_image, (cx_frame, cy_frame), (tx, ty), (0, 0, 255), 3)
                cv2.circle(cv_image, (cx_frame, cy_frame), 5, (0, 255, 255), -1)
                cv2.putText(cv_image, f"{self.target_key_label}: dx={tx-cx_frame}px dy={ty-cy_frame}px", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.putText(cv_image, f"STATE: {self.status_text}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                
                self.handle_runtime_input(now_sec)
                self.publish_debug_image(cv_image, msg.header)
                cv2.imshow("ArUco Detection", cv_image)
                self.prev_gray = gray
                return
            else:
                self.key_track_lost += 1
                if self.key_track_lost >= self.key_track_max_lost:
                    self.key_track_active, self.layout_locked = False, False
                    self.target_kf_initialized = False

        # 2. Global Detection Mode
        corners, ids, _ = self.detect_markers_multi_threshold(gray)
        if ids is not None:
            try: cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            except: pass
            for mid, mcorners in zip(ids.flatten(), corners): 
                if self.marker_ref_is_bottom_right and mcorners is not None and mcorners.shape[1] == 4:
                    mcorners = np.roll(mcorners, -2, axis=1)
                self.last_known_corners[mid], self.last_seen_timestamp[mid] = mcorners, now_sec
            self.publish_rviz_markers(corners, ids, msg.header)

        # ArUco / Flow / Hold logic for homography
        fresh_markers = {mid: c for mid, c in self.last_known_corners.items() if (now_sec - self.last_seen_timestamp.get(mid, 0)) < self.STALE_THRESHOLD_SECONDS}
        have_current_view = False
        if len(fresh_markers) == 4:
            centers = {mid: np.mean(c[0], axis=0) for mid, c in fresh_markers.items()}
            sorted_y = sorted(centers.items(), key=lambda x: x[1][1])
            top, bot = sorted(sorted_y[:2], key=lambda x: x[1][0]), sorted(sorted_y[2:], key=lambda x: x[1][0])
            corner_ids = (top[0][0], top[1][0], bot[1][0], bot[0][0])
            
            # Using INSIDE corners (facing the keyboard) for standard ArUco orientation (0=TL)
            # TL Marker: Index 2 (BR) | TR Marker: Index 3 (BL) 
            # BR Marker: Index 0 (TL) | BL Marker: Index 1 (TR)
            pts_src = np.array([
                fresh_markers[corner_ids[0]][0][2], 
                fresh_markers[corner_ids[1]][0][3], 
                fresh_markers[corner_ids[2]][0][0], 
                fresh_markers[corner_ids[3]][0][1]
            ], dtype="float32")
            
            w_dst, h_dst = 800, 400
            pts_dst = np.array([[0, 0], [w_dst-1, 0], [w_dst-1, h_dst-1], [0, h_dst-1]], dtype="float32")
            M, M_inv = cv2.getPerspectiveTransform(pts_src, pts_dst), cv2.getPerspectiveTransform(pts_dst, pts_src)
            
            if self.last_M_for_stability is not None:
                delta = np.mean(np.linalg.norm(cv2.perspectiveTransform(pts_src.reshape(4,1,2), M) - cv2.perspectiveTransform(pts_src.reshape(4,1,2), self.last_M_for_stability), axis=2))
                if delta <= self.stable_thresh_px: self.homography_stable_frames += 1
                else: self.homography_stable_frames = 0
            self.last_M_for_stability = M.copy()
            self.last_good_M, self.last_good_M_inv, self.last_good_M_time = M, M_inv, now_sec
            have_current_view = True
            self.status_text = "ARUCO"

        active_M, active_M_inv = None, None
        if have_current_view: active_M, active_M_inv = self.last_good_M, self.last_good_M_inv
        elif self.prev_gray is not None and self.prev_flow_points is not None and len(self.prev_flow_points) >= self.flow_min_points:
            p1, st, err = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, self.prev_flow_points, None)
            if p1 is not None and st is not None:
                good_new = p1[st == 1]
                good_old = self.prev_flow_points[st == 1]
                if len(good_new) >= self.flow_min_points:
                    M_flow, _ = cv2.findHomography(good_old, good_new, cv2.RANSAC, 5.0)
                    if M_flow is not None and self.last_good_M is not None:
                        self.last_good_M = self.last_good_M @ np.linalg.inv(M_flow)
                        self.last_good_M_inv = M_flow @ self.last_good_M_inv
                        self.last_good_M_time = now_sec
                        active_M, active_M_inv = self.last_good_M, self.last_good_M_inv
                        self.prev_flow_points = good_new.reshape(-1, 1, 2)
                        self.status_text = "FLOW"
        
        if active_M is None and (now_sec - self.last_good_M_time) < self.homography_hold_seconds and self.last_good_M is not None:
            active_M, active_M_inv = self.last_good_M, self.last_good_M_inv
            self.status_text = "HOLD"

        target_real_final = None
        if active_M is not None:
            warped = cv2.warpPerspective(cv_image, active_M, (800, 400))
            keyboard_quad, roi_debug = self.detect_keyboard_contour(warped)
            if keyboard_quad is None: keyboard_quad = np.array([[0, 0], [799, 0], [799, 399], [0, 399]], dtype="float32")
            if have_current_view: 
                keyboard_quad, snap_flags = self.snap_keyboard_quad_to_roi(keyboard_quad, active_M, fresh_markers, corner_ids, 800, 400)
                if self.keyboard_snap_debug: self.draw_snap_debug(warped, snap_flags, 800, 400)
                quad_frame = cv2.perspectiveTransform(keyboard_quad.reshape(4, 1, 2).astype(np.float32), active_M_inv).reshape(4, 2)
                self.init_flow_points(gray, quad_frame)
            
            kb_w, kb_h = self.keyboard_size
            kb_dst = np.array([[0, 0], [kb_w-1, 0], [kb_w-1, kb_h-1], [0, kb_h-1]], dtype="float32")
            K_inv = cv2.getPerspectiveTransform(kb_dst, keyboard_quad)
            frame_corners = np.array([[[0,0]], [[w-1, 0]], [[w-1, h-1]], [[0, h-1]]], dtype="float32")
            visible_poly_warp = cv2.perspectiveTransform(frame_corners, active_M)

            layout_img, warped_overlay = np.zeros((kb_h, kb_w, 3), dtype=np.uint8), warped.copy()
            target_label, target_center_warp = self.target_key_label.lower(), None
            for key in self.keyboard_layout:
                kx, ky, kw, kh = key['x']*kb_w, key['y']*kb_h, key['w']*kb_w, key['h']*kb_h
                kquad = np.array([[[kx, ky]], [[kx+kw, ky]], [[kx+kw, ky+kh]], [[kx, ky+kh]]], dtype="float32")
                k_in_warp = cv2.perspectiveTransform(kquad, K_inv)
                if not self.is_key_visible_warp(k_in_warp, visible_poly_warp): continue
                cv2.rectangle(layout_img, (int(kx), int(ky)), (int(kx+kw), int(ky+kh)), (0, 255, 0), 1)
                cv2.putText(layout_img, key['label'], (int(kx+2), int(ky+kh-5)), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 1)
                cv2.polylines(warped_overlay, [k_in_warp.astype(int)], True, (0, 255, 0), 1)
                if key['label'].lower() == target_label:
                    target_center_warp = k_in_warp.reshape(4, 2).mean(axis=0)
                    cv2.circle(layout_img, (int(kx+kw/2), int(ky+kh/2)), 4, (0, 0, 255), -1)

            cv2.imshow("Warped ROI", warped)
            cv2.imshow("Keyboard Layout (Canonical)", layout_img)
            cv2.imshow("Keyboard Layout (Warped)", warped_overlay)
            
            if target_center_warp is not None and self.target_key_label:
                target_real_final = cv2.perspectiveTransform(np.array([[[target_center_warp[0], target_center_warp[1]]]], dtype="float32"), active_M_inv)[0][0]
                
                # Reset acquisition once target is found in frame
                if self.await_target_acquire:
                    self.await_target_acquire = False
                    self.homography_stable_frames = 0
                    self.pose_lock_start_time = None

                # Stability check for transition to tracking
                if have_current_view and self.homography_stable_frames >= self.homography_stable_required:
                    if self.pose_lock_start_time is None: self.pose_lock_start_time = now_sec
                    if (now_sec - self.pose_lock_start_time) >= self.pose_lock_delay_sec:
                        self.key_track_active, self.key_track_point, self.layout_locked = True, np.array([[[float(target_real_final[0]), float(target_real_final[1])]]], dtype=np.float32), True
            
            # Smoothly track keyboard origin for display
            warped_corner = np.array([[[keyboard_quad[0][0], keyboard_quad[0][1]]]], dtype="float32")
            real_corner = cv2.perspectiveTransform(warped_corner, active_M_inv)[0][0]
            filtered_origin, self.origin_kf_initialized = self.update_kf_2d(self.origin_kf, self.origin_kf_initialized, real_corner, max(1e-3, now_sec - (self.origin_kf_last_time or now_sec)))
            self.origin_kf_last_time = now_sec
            cv2.circle(cv_image, (int(filtered_origin[0]), int(filtered_origin[1])), 8, (0, 0, 255), -1)
            cv2.putText(cv_image, "Origin", (int(filtered_origin[0])+10, int(filtered_origin[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        else: self.status_text = "SEARCHING"

        # Global Filtering (when not yet tracking)
        if target_real_final is not None:
            filtered, self.target_kf_initialized = self.update_kf_2d(self.target_kf, self.target_kf_initialized, target_real_final, max(1e-3, now_sec - (self.target_kf_last_time or now_sec)))
            self.target_kf_last_time = now_sec
            tx, ty = int(filtered[0]), int(filtered[1])
            cv2.circle(cv_image, (tx, ty), 8, (0, 0, 255), -1)
            cv2.putText(cv_image, f"TARGET: {self.target_key_label}", (tx+10, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # UI Overlays
        cv2.putText(cv_image, f"INPUT: {self.input_buffer}", (10, h-40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        if self.autonomous_mode:
            queue_str = ''.join(list(self.typing_queue)[:15])
            cv2.putText(cv_image, f"AUTO QUEUE: {queue_str}", (10, h-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(cv_image, f"STATE: {self.status_text}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        cv2.imshow("ArUco Detection", cv_image)
        self.handle_runtime_input(now_sec)
        self.publish_debug_image(cv_image, msg.header)
        self.prev_gray = gray

    def publish_debug_image(self, cv_image, header):
        debug_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        debug_msg.header = header
        self.debug_pub.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ZedArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
