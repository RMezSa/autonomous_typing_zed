import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

import cv2
import numpy as np
from typing import List, Tuple, Optional
import time
from collections import deque



class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_node')

        self.bridge = CvBridge()

        self.declare_parameter('image_topic', '/image_raw')
        image_topic = self.get_parameter('image_topic').value

        self.sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        self.marker_pub = self.create_publisher(MarkerArray, 'markers', 10)
        self.debug_image_pub = self.create_publisher(Image, 'debug_image', 10)
        self.last_marker_ids = set()

        # ArUcos - Dictionary
        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )

        # Advanced detector parameters (inspired by ros_aruco_opencv)
        self.parameters = cv2.aruco.DetectorParameters_create()
        
        # --- Adaptive thresholding parameters ---
        self.parameters.adaptiveThreshWinSizeMin = 5  # Wider range for better robustness
        self.parameters.adaptiveThreshWinSizeMax = 21  # Adjusted for 640x480 resolution
        self.parameters.adaptiveThreshWinSizeStep = 4  # Finer steps
        self.parameters.adaptiveThreshConstant = 5  # Lower for better low contrast detection
        
        # --- Marker geometry constraints ---
        # More permissive for challenging conditions
        self.parameters.minMarkerPerimeterRate = 0.01  # Smaller markers allowed
        self.parameters.maxMarkerPerimeterRate = 4.0  # Standard max
        self.parameters.polygonalApproxAccuracyRate = 0.03  # Higher accuracy
        self.parameters.minCornerDistanceRate = 0.03  # Prevent merging
        self.parameters.minMarkerDistanceRate = 0.01  # Allow close markers
        self.parameters.minDistanceToBorder = 1  # Small border margin
        
        # --- Corner refinement ---
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.parameters.cornerRefinementWinSize = 5  # Window for subpixel
        self.parameters.cornerRefinementMaxIterations = 60  # More iterations
        self.parameters.cornerRefinementMinAccuracy = 0.01  # High precision
        
        # --- Bit extraction & decoding ---
        self.parameters.markerBorderBits = 1  # Standard ArUco border
        self.parameters.perspectiveRemovePixelPerCell = 4  # Resolution for bit extraction
        self.parameters.perspectiveRemoveIgnoredMarginPerCell = 0.13  # Border margin
        self.parameters.maxErroneousBitsInBorderRate = 0.35  # Error tolerance
        self.parameters.errorCorrectionRate = 0.6  # Balanced error correction
        
        # --- Detection optimization ---
        self.parameters.detectInvertedMarker = True  # Support inverted markers
        
        # Image preprocessing configuration
        self.use_clahe = False  # Contrast Limited Adaptive Histogram Equalization
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        self.use_bilateral = False  # Bilateral filter (slower but better noise reduction)
        self.bilateral_d = 5
        self.bilateral_sigma_color = 50
        self.bilateral_sigma_space = 50

        # ArUco reference corner: False means top-left (OpenCV default)
        # Set True only if your marker uses bottom-right as the red reference.
        self.marker_ref_is_bottom_right = False
        
        # Multi-threshold detection strategy
        self.use_multi_threshold = False
        self.threshold_methods = ['adaptive', 'otsu', 'fixed']
        self.fixed_threshold = 100
        
        # Pose estimation configuration
        self.use_ippe_square = True  # Better pose estimation with IPPE
        self.pose_selection_strategy = 'reprojection_error'  # or 'plane_normal'
        
        self.get_logger().info("ArUco node started")

        # Pose stability parameters
        self.stable_frames = 0
        self.required_stable_frames = 90
        self.stable_thresh_px = 3.0
        self.last_target_pos = None

        # --- State for robust detection ---
        # Store the last known corners of each marker
        self.last_known_corners = {}
        # Store the timestamp of when each marker was last seen
        self.last_seen_timestamp = {}
        
        # How long to remember a marker's position (in seconds)
        self.STALE_THRESHOLD_SECONDS = 2.5

        # Smoothing for keyboard corner output
        self.smooth_corner = None
        self.corner_alpha = 0.5  # 0=no smoothing, 1=very responsive

        # Smoothing/hold for keyboard quad to reduce blinking
        self.last_keyboard_quad = None
        self.last_keyboard_quad_time = 0.0
        self.keyboard_quad_alpha = 0.15
        self.keyboard_quad_hold_seconds = 2.0

        # Keyboard layout mapping (ISO ES 85-key style)
        self.keyboard_total_units = 18.0
        self.keyboard_rows = 6
        self.keyboard_size = (1000, 350)  # canonical size for layout mapping
        self.keyboard_layout = self.build_keyboard_layout()
        # If detected quad covers less than this fraction of ROI, assume full ROI is keyboard
        self.keyboard_full_roi_threshold = 0.5
        # Keyboard contour tuning (stage 3)
        self.keyboard_min_area_ratio = 0.12
        self.keyboard_expected_area_ratio = 0.35
        self.keyboard_roi_pad_ratio = 0.03
        self.keyboard_fallback_expand_ratio = 0.2
        # If the keyboard corner is close to the ROI corner (in warped ROI px), snap it
        self.keyboard_corner_snap_px = 10 #27
        # Marker must be close to the keyboard quad corner (in warped ROI px)
        self.keyboard_marker_to_kb_corner_px = 90
        # Debug overlay for snap behavior
        self.keyboard_snap_debug = True
        # Keyboard black-surface segmentation (stage 3)
        self.keyboard_use_black_mask = True
        self.keyboard_black_v_thresh = 120
        self.keyboard_black_l_thresh = 95
        self.keyboard_black_close_iter = 4
        self.keyboard_black_open_iter = 0

        # Target key to track (set this label, e.g., "Space")
        self.target_key_label = "Space"
        # Runtime input buffer for changing target key
        self.input_buffer = ""
        
        # Autonomous typing queue (FIFO)
        self.typing_queue = deque()
        self.autonomous_mode = False
        self.current_typing_target = None
        self.typing_cooldown_until = 0.0
        self.typing_cooldown_duration = 0.5
        
        # Homography stability for autonomous mode
        self.homography_stable_frames = 0
        self.homography_stable_required = 2

        # Optical flow fallback state
        self.prev_gray = None
        self.prev_flow_points = None
        self.last_keyboard_quad_frame = None
        
        self.flow_max_error = 20.0
        self.flow_min_points = 20
        self.flow_quality = 0.05
        self.flow_min_distance = 8

        # Persistent homography (last strong ArUco measurement)
        self.last_good_M = None
        self.last_good_M_inv = None
        self.last_good_M_time = None
        self.homography_hold_seconds = 1.0

        # Target key tracking in frame space (freeze during approach)
        self.key_track_active = False
        self.key_track_point = None
        self.key_track_err_thresh = 20.0
        self.key_track_max_lost = 8
        self.key_track_lost = 0

        # Layout lock state (freeze warp/layout during fine tracking)
        self.layout_locked = False
        self.layout_lock_M = None
        self.layout_lock_M_inv = None
        self.layout_lock_K = None
        self.layout_lock_K_inv = None
        self.layout_lock_quad = None

        # Status label for current pipeline state
        self.status_text = "INIT"

        # Kalman filters for target center and keyboard origin (x, y, vx, vy)
        self.target_kf = cv2.KalmanFilter(4, 2)
        self.origin_kf = cv2.KalmanFilter(4, 2)
        for kf in (self.target_kf, self.origin_kf):
            kf.transitionMatrix = np.eye(4, dtype=np.float32)
            kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=np.float32)
            kf.processNoiseCov = np.eye(4, dtype=np.float32) * 1e-2
            kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1e-1
            kf.errorCovPost = np.eye(4, dtype=np.float32)

        # Slightly lower process noise for keyboard origin stability
        self.origin_kf.processNoiseCov = np.eye(4, dtype=np.float32) * 1e-3

        self.target_kf_initialized = False
        self.origin_kf_initialized = False
        self.target_kf_last_time = None
        self.origin_kf_last_time = None

        self.pose_lock = False
        self.pose_lock_threshold_px = 40
        self.pose_lock_delay_sec = 0.5
        self.pose_lock_start_time = None

        # Require re-acquiring target after completing a letter
        self.await_target_acquire = False

        # Homography stability tracking
        self.last_M_for_stability = None

    def complete_current_key(self):
        """Mark current typing target as completed and prepare for next"""
        if self.autonomous_mode and self.current_typing_target is not None:
            self.get_logger().info(f"Completed typing: '{self.current_typing_target}'")
            self.current_typing_target = None
            self.await_target_acquire = True

    def order_points(self, pts):
        # Orders 4 points as: top-left, top-right, bottom-right, bottom-left
        pts = np.array(pts, dtype="float32")
        s = pts.sum(axis=1)
        diff = np.diff(pts, axis=1)

        tl = pts[np.argmin(s)]
        br = pts[np.argmax(s)]
        tr = pts[np.argmin(diff)]
        bl = pts[np.argmax(diff)]

        return np.array([tl, tr, br, bl], dtype="float32")

    def snap_keyboard_quad_to_roi(self, keyboard_quad, M, fresh_markers, corner_ids, w_dst, h_dst):
        if keyboard_quad is None or M is None:
            return keyboard_quad, [False, False, False, False]

        if not fresh_markers or corner_ids is None:
            return keyboard_quad, [False, False, False, False]

        tl_id, tr_id, br_id, bl_id = corner_ids
        if tl_id not in fresh_markers or tr_id not in fresh_markers or br_id not in fresh_markers or bl_id not in fresh_markers:
            return keyboard_quad, [False, False, False, False]

        centers_frame = np.array([
            np.mean(fresh_markers[tl_id][0], axis=0),
            np.mean(fresh_markers[tr_id][0], axis=0),
            np.mean(fresh_markers[br_id][0], axis=0),
            np.mean(fresh_markers[bl_id][0], axis=0)
        ], dtype=np.float32).reshape(-1, 1, 2)

        centers_warp = cv2.perspectiveTransform(centers_frame, M).reshape(4, 2)

        roi_corners = np.array(
            [[0, 0], [w_dst - 1, 0], [w_dst - 1, h_dst - 1], [0, h_dst - 1]],
            dtype=np.float32
        )

        adjusted = keyboard_quad.copy()
        snapped = [False, False, False, False]
        for i in range(4):
            dist_marker_to_kb = np.linalg.norm(centers_warp[i] - keyboard_quad[i])
            dist_kb_to_roi = np.linalg.norm(keyboard_quad[i] - roi_corners[i])
            if dist_marker_to_kb <= self.keyboard_marker_to_kb_corner_px and dist_kb_to_roi <= self.keyboard_corner_snap_px:
                adjusted[i] = roi_corners[i]
                snapped[i] = True

        return adjusted, snapped

    def draw_snap_debug(self, warped_roi, snapped, w_dst, h_dst):
        if warped_roi is None:
            return

        has_snap = bool(snapped) and any(snapped)

        if not has_snap:
            cv2.putText(
                warped_roi, "SNAP INACTIVE",
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (0, 255, 255), 2
            )
            return

        labels = ["TL", "TR", "BR", "BL"]
        roi_corners = np.array(
            [[0, 0], [w_dst - 1, 0], [w_dst - 1, h_dst - 1], [0, h_dst - 1]],
            dtype=np.float32
        )

        cv2.putText(
            warped_roi, "SNAP ACTIVE",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7, (0, 0, 255), 2
        )

        for i, flag in enumerate(snapped):
            if not flag:
                continue
            x, y = int(roi_corners[i][0]), int(roi_corners[i][1])
            cv2.circle(warped_roi, (x, y), 6, (0, 0, 255), -1)
            cv2.putText(
                warped_roi, f"SNAP {labels[i]}",
                (x + 6, y + 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 0, 255), 2
            )

    def build_keyboard_layout(self):
        layout = []

        def add_row(row_idx, items):
            x_units = 0.0
            y = row_idx / self.keyboard_rows
            h = 1.0 / self.keyboard_rows
            for label, w_units in items:
                if label is not None:
                    layout.append({
                        "label": label,
                        "x": x_units / self.keyboard_total_units,
                        "y": y,
                        "w": w_units / self.keyboard_total_units,
                        "h": h
                    })
                x_units += w_units

        add_row(0, [
            ("Esc", 1), (None, 1),
            ("F1", 1), ("F2", 1), ("F3", 1), ("F4", 1), (None, 0.5),
            ("F5", 1), ("F6", 1), ("F7", 1), ("F8", 1), (None, 0.5),
            ("F9", 1), ("F10", 1), ("F11", 1), ("F12", 1), (None, 0.3),
            ("PrtSc", 1), ("ScrLk", 1), ("Pause", 1)
        ])

        add_row(1, [
            ("º", 1), ("1", 1), ("2", 1), ("3", 1), ("4", 1), ("5", 1),
            ("6", 1), ("7", 1), ("8", 1), ("9", 1), ("0", 1), ("'", 1),
            ("¿", 1), ("Backspace", 2), (None, 0.3), ("Ins", 1), ("Home", 1), ("PgUp", 1)
        ])

        add_row(2, [
            ("Tab", 1.5), ("Q", 1), ("W", 1), ("E", 1), ("R", 1), ("T", 1),
            ("Y", 1), ("U", 1), ("I", 1), ("O", 1), ("P", 1), ("`", 1),
            ("+", 1), ("Enter", 1.5), (None, 0.3) ,("Del", 1), ("End", 1), ("PgDn", 1)
        ])

        add_row(3, [
            ("Caps", 1.75), ("A", 1), ("S", 1), ("D", 1), ("F", 1), ("G", 1),
            ("H", 1), ("J", 1), ("K", 1), ("L", 1), ("Ñ", 1), ("´", 1),
            ("Enter", 2.25), (None, 3.0)
        ])

        add_row(4, [
            ("Shift", 1.25), ("<", 1), ("Z", 1), ("X", 1), ("C", 1), ("V", 1),
            ("B", 1), ("N", 1), ("M", 1), (",", 1), (".", 1), ("-", 1),
            ("Shift", 2.75), (None, 1), ("Up", 1.0), (None, 1.0)
        ])

        add_row(5, [
            ("Ctrl", 1.25), ("Win", 1.25), ("Alt", 1.25), ("Space", 6),
            ("AltGr", 1.25), ("Fn", 1.25), ("Menu", 1.25), ("Ctrl", 1.25), (None, .3),
            ("Left", 1), ("Down", 1), ("Right", 1)
        ])

        return layout

    def normalize_key_label(self, label):
        return label.strip().lower()

    def init_flow_points(self, gray_frame, quad_frame):
        if quad_frame is None:
            self.prev_flow_points = None
            return

        mask = np.zeros_like(gray_frame)
        cv2.fillConvexPoly(mask, quad_frame.astype(int), 255)
        pts = cv2.goodFeaturesToTrack(
            gray_frame,
            maxCorners=200,
            qualityLevel=self.flow_quality,
            minDistance=self.flow_min_distance,
            mask=mask
        )
        self.prev_flow_points = pts

    def preprocess_image(self, gray_frame: np.ndarray) -> np.ndarray:
        """Apply image preprocessing optimizations for better marker detection."""
        processed = gray_frame.copy()
        
        # Apply CLAHE for contrast enhancement
        if self.use_clahe:
            processed = self.clahe.apply(processed)
        
        # Apply bilateral filter for noise reduction while preserving edges
        if self.use_bilateral:
            processed = cv2.bilateralFilter(
                processed, 
                self.bilateral_d, 
                self.bilateral_sigma_color, 
                self.bilateral_sigma_space
            )
        
        return processed
    
    def detect_markers_multi_threshold(self, gray_frame: np.ndarray) -> Tuple[List, np.ndarray, List]:
        """Detect markers using multiple threshold strategies and merge results."""
        all_corners = []
        all_ids = []
        seen_ids = set()
        
        preprocessed = self.preprocess_image(gray_frame)
        
        if not self.use_multi_threshold:
            # Standard single detection
            corners, ids, rejected = cv2.aruco.detectMarkers(
                preprocessed, self.dictionary, parameters=self.parameters
            )
            return corners, ids, rejected
        
        # Strategy 1: Adaptive thresholding (best for varying lighting)
        if 'adaptive' in self.threshold_methods:
            corners_adapt, ids_adapt, _ = cv2.aruco.detectMarkers(
                preprocessed, self.dictionary, parameters=self.parameters
            )
            if ids_adapt is not None:
                for i, marker_id in enumerate(ids_adapt.flatten()):
                    if marker_id not in seen_ids:
                        all_corners.append(corners_adapt[i])
                        all_ids.append(marker_id)
                        seen_ids.add(marker_id)
        
        # Strategy 2: Otsu's thresholding (good for bimodal intensity distribution)
        if 'otsu' in self.threshold_methods:
            _, thresh_otsu = cv2.threshold(
                preprocessed, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
            )
            corners_otsu, ids_otsu, _ = cv2.aruco.detectMarkers(
                thresh_otsu, self.dictionary, parameters=self.parameters
            )
            if ids_otsu is not None:
                for i, marker_id in enumerate(ids_otsu.flatten()):
                    if marker_id not in seen_ids:
                        all_corners.append(corners_otsu[i])
                        all_ids.append(marker_id)
                        seen_ids.add(marker_id)
        
        # Strategy 3: Fixed thresholding (fallback)
        if 'fixed' in self.threshold_methods:
            _, thresh_fixed = cv2.threshold(
                preprocessed, self.fixed_threshold, 255, cv2.THRESH_BINARY
            )
            corners_fixed, ids_fixed, _ = cv2.aruco.detectMarkers(
                thresh_fixed, self.dictionary, parameters=self.parameters
            )
            if ids_fixed is not None:
                for i, marker_id in enumerate(ids_fixed.flatten()):
                    if marker_id not in seen_ids:
                        all_corners.append(corners_fixed[i])
                        all_ids.append(marker_id)
                        seen_ids.add(marker_id)
        
        if len(all_ids) > 0:
            # Convert to expected format
            corners_array = tuple(all_corners)
            ids_array = np.array(all_ids).reshape(-1, 1)
            return corners_array, ids_array, []
        else:
            return tuple(), None, []
    
    def select_best_pose_ippe(self, rvecs: List[np.ndarray], tvecs: List[np.ndarray], 
                               corners: np.ndarray, camera_matrix: Optional[np.ndarray] = None,
                               dist_coeffs: Optional[np.ndarray] = None) -> Tuple[np.ndarray, np.ndarray]:
        """Select best pose from IPPE candidates based on reprojection error or plane normal."""
        if len(rvecs) == 1:
            return rvecs[0], tvecs[0]
        
        if self.pose_selection_strategy == 'reprojection_error' and camera_matrix is not None:
            # Calculate reprojection errors for each candidate
            errors = []
            for rvec, tvec in zip(rvecs, tvecs):
                # Project 3D points back to image
                marker_size = 0.02  # 2cm markers
                obj_points = np.array([
                    [-marker_size/2, marker_size/2, 0],
                    [marker_size/2, marker_size/2, 0],
                    [marker_size/2, -marker_size/2, 0],
                    [-marker_size/2, -marker_size/2, 0]
                ], dtype=np.float32)
                
                projected, _ = cv2.projectPoints(
                    obj_points, rvec, tvec, camera_matrix, dist_coeffs
                )
                projected = projected.reshape(-1, 2)
                corners_2d = corners.reshape(-1, 2)
                
                # Calculate mean reprojection error
                error = np.mean(np.linalg.norm(projected - corners_2d, axis=1))
                errors.append(error)
            
            # Return pose with minimum reprojection error
            best_idx = np.argmin(errors)
            return rvecs[best_idx], tvecs[best_idx]
        
        elif self.pose_selection_strategy == 'plane_normal':
            # Select pose where marker plane normal is most parallel to camera view
            best_idx = 0
            best_score = float('inf')
            
            for i, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
                # Convert rotation vector to matrix
                R, _ = cv2.Rodrigues(rvec)
                # Marker normal in camera frame (Z-axis of marker)
                normal = R[:, 2]
                # Camera viewing direction is -Z in camera frame
                view_dir = np.array([0, 0, -1])
                # Score: angle between normal and view direction (want them parallel)
                score = np.arccos(np.clip(np.dot(normal, view_dir), -1.0, 1.0))
                
                if score < best_score:
                    best_score = score
                    best_idx = i
            
            return rvecs[best_idx], tvecs[best_idx]
        
        # Default: return first candidate
        return rvecs[0], tvecs[0]
    
    def is_key_visible(self, key_in_frame, frame_w, frame_h):
        pts = key_in_frame.reshape(4, 2)
        inside = (
            (pts[:, 0] >= 0) & (pts[:, 0] < frame_w) &
            (pts[:, 1] >= 0) & (pts[:, 1] < frame_h)
        )
        if inside.any():
            return True

        center = pts.mean(axis=0)
        return (0 <= center[0] < frame_w) and (0 <= center[1] < frame_h)

    def is_key_visible_warp(self, key_quad_warp, visible_poly_warp):
        if visible_poly_warp is None:
            return True

        poly = visible_poly_warp.reshape(-1, 2)
        if not np.isfinite(poly).all():
            return True

        pts = key_quad_warp.reshape(4, 2)
        for pt in pts:
            if cv2.pointPolygonTest(poly.astype(np.float32), (float(pt[0]), float(pt[1])), False) >= 0:
                return True

        center = pts.mean(axis=0)
        return cv2.pointPolygonTest(poly.astype(np.float32), (float(center[0]), float(center[1])), False) >= 0

    def update_kf_2d(self, kf, initialized, measurement, dt):
        if not initialized:
            kf.statePost = np.array([[measurement[0]], [measurement[1]], [0.0], [0.0]], dtype=np.float32)
            kf.statePre = kf.statePost.copy()
            return measurement, True

        kf.transitionMatrix = np.array(
            [[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]],
            dtype=np.float32
        )
        kf.predict()
        z = np.array([[measurement[0]], [measurement[1]]], dtype=np.float32)
        corr = kf.correct(z)
        filtered = np.array([corr[0, 0], corr[1, 0]], dtype=np.float32)
        return filtered, True

    def detect_keyboard_contour(self, roi):
        # The ROI is now a clean, top-down view.
        h_roi, w_roi, _ = roi.shape
        roi_area = h_roi * w_roi

        def clamp_quad(quad, w, h):
            quad[:, 0] = np.clip(quad[:, 0], 0, w - 1)
            quad[:, 1] = np.clip(quad[:, 1], 0, h - 1)
            return quad

        def inset_quad(quad, dx, dy):
            quad = quad.copy()
            quad[:, 0] = np.clip(quad[:, 0] + np.array([dx, -dx, -dx, dx]), 0, w_roi - 1)
            quad[:, 1] = np.clip(quad[:, 1] + np.array([dy, dy, -dy, -dy]), 0, h_roi - 1)
            return quad

        def expand_quad(quad, dx, dy):
            quad = quad.copy()
            quad[:, 0] = quad[:, 0] + np.array([-dx, dx, dx, -dx])
            quad[:, 1] = quad[:, 1] + np.array([-dy, -dy, dy, dy])
            return clamp_quad(quad, w_roi, h_roi)

        def approx_quad(contour):
            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.02 * peri, True)
            if len(approx) == 4:
                return self.order_points(approx.reshape(4, 2))
            return None

        pad_x = int(w_roi * self.keyboard_roi_pad_ratio)
        pad_y = int(h_roi * self.keyboard_roi_pad_ratio)
        expected_quad = np.array(
            [[pad_x, pad_y], [w_roi - 1 - pad_x, pad_y],
             [w_roi - 1 - pad_x, h_roi - 1 - pad_y], [pad_x, h_roi - 1 - pad_y]],
            dtype="float32"
        )
        search_mask = np.zeros((h_roi, w_roi), dtype=np.uint8)
        cv2.fillConvexPoly(search_mask, expected_quad.astype(int), 255)
        
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        # Black keyboard segmentation (preferred when keyboard is clearly dark)
        black_contours = []
        if self.keyboard_use_black_mask:
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
            v = hsv[:, :, 2]
            l = lab[:, :, 0]
            black_mask = cv2.inRange(v, 0, self.keyboard_black_v_thresh)
            black_mask_l = cv2.inRange(l, 0, self.keyboard_black_l_thresh)
            black_mask = cv2.bitwise_or(black_mask, black_mask_l)
            black_mask = cv2.bitwise_and(black_mask, black_mask, mask=search_mask)

            kernel = np.ones((5, 5), np.uint8)
            black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel, iterations=self.keyboard_black_close_iter)
            black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, kernel, iterations=self.keyboard_black_open_iter)
            black_contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        contours = black_contours

        if not contours:
            # Primary: Canny edges
            edges = cv2.Canny(gray, 30, 120)
            kernel = np.ones((5, 5), np.uint8)
            edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)

            edges = cv2.bitwise_and(edges, edges, mask=search_mask)

            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Fallback: adaptive threshold + morphology to get keyboard blob
            if not contours:
                thresh = cv2.adaptiveThreshold(
                    gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 25, 7
                )
                thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)
                thresh = cv2.bitwise_and(thresh, thresh, mask=search_mask)
                contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None, roi

        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        best_quad = None
        best_area = 0.0

        for contour in contours:
            area = cv2.contourArea(contour)

            # Ensure contour is a large part of the ROI (relaxed)
            if area < (roi_area * self.keyboard_min_area_ratio):
                continue

            rect = cv2.minAreaRect(contour)
            (w, h) = rect[1]

            if w == 0 or h == 0:
                continue

            aspect_ratio = max(w, h) / min(w, h)

            # Aspect ratio for a rectified keyboard
            if 1.3 < aspect_ratio < 5.0:
                ordered = approx_quad(contour)
                if ordered is None:
                    box = cv2.boxPoints(rect)
                    ordered = self.order_points(box)

                if area > best_area:
                    best_area = area
                    best_quad = ordered

        # If we found a candidate quad, decide whether it represents the whole keyboard
        if best_quad is not None:
            # If quad is smaller than expected, expand it instead of forcing full ROI
            if best_area < (roi_area * self.keyboard_expected_area_ratio):
                expand_x = int(w_roi * self.keyboard_fallback_expand_ratio)
                expand_y = int(h_roi * self.keyboard_fallback_expand_ratio)
                best_quad = expand_quad(best_quad, expand_x, expand_y)

            cv2.drawContours(roi, [best_quad.astype(int)], -1, (0, 255, 0), 3)

            corner = (int(best_quad[0][0]), int(best_quad[0][1]))

            cv2.circle(roi, corner, 8, (0, 0, 255), -1)
            cv2.putText(
                roi, "Keyboard Origin",
                (corner[0] + 15, corner[1] + 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (0, 0, 255), 2
            )

            return best_quad, roi

        return None, roi


    def image_callback(self, msg):
        # ROS Image -> OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        header = msg.header
        h, w = frame.shape[:2]
        if w >= 2 * h:
            # Side-by-side stereo: keep only left camera image
            frame = frame[:, :w // 2]
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        
        # Autonomous mode: consume queue when ready for next target
        if self.autonomous_mode and not self.key_track_active and self.current_typing_target is None and now_sec > self.typing_cooldown_until:
            # Wait for stable homography before starting next letter
            if self.homography_stable_frames >= self.homography_stable_required:
                if self.typing_queue:
                    self.current_typing_target = self.typing_queue.popleft()
                    self.target_key_label = self.current_typing_target
                    self.homography_stable_frames = 0  # Reset for next letter
                    self.get_logger().info(f"Auto-typing next: '{self.current_typing_target}' (queue: {len(self.typing_queue)} remaining)")
                else:
                    # Queue empty, exit autonomous mode
                    self.autonomous_mode = False
                    self.current_typing_target = None
                    self.target_key_label = ""
                    self.await_target_acquire = False
                    self.pose_lock_start_time = None
                    self.get_logger().info("Autonomous typing completed")
            else:
                self.get_logger().info(f"Waiting for stable homography: {self.homography_stable_frames}/{self.homography_stable_required}", throttle_duration_sec=0.5)

        # Fine tracking mode: skip pose/warp/layout updates
        if self.key_track_active:
            self.status_text = "TRACKING"
            target_real = None
            if self.prev_gray is not None and self.key_track_point is not None:
                p1, st, err = cv2.calcOpticalFlowPyrLK(
                    self.prev_gray, gray_frame, self.key_track_point, None,
                    winSize=(21, 21), maxLevel=3,
                    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
                )

                if p1 is not None and st is not None and err is not None and st[0][0] == 1 and err[0][0] <= self.key_track_err_thresh:
                    self.key_track_point = p1
                    target_real = p1[0][0]
                    self.key_track_lost = 0
                else:
                    self.key_track_lost += 1
                    if self.key_track_lost >= self.key_track_max_lost:
                        self.key_track_active = False
                        self.key_track_point = None
                        self.key_track_lost = 0

            if target_real is not None:
                if self.target_kf_last_time is None:
                    dt = 1.0 / 30.0
                else:
                    dt = max(1e-3, now_sec - self.target_kf_last_time)

                filtered_target, self.target_kf_initialized = self.update_kf_2d(
                    self.target_kf,
                    self.target_kf_initialized,
                    target_real,
                    dt
                )

                tx, ty = int(filtered_target[0]), int(filtered_target[1])
                cx_frame, cy_frame = frame.shape[1] // 2, frame.shape[0] // 2
                dx = tx - cx_frame
                dy = ty - cy_frame

                self.pose_lock = True

                cv2.circle(frame, (tx, ty), 6, (0, 0, 255), -1)
                cv2.line(frame, (cx_frame, cy_frame), (tx, ty), (0, 0, 255), 3)
                cv2.circle(frame, (cx_frame, cy_frame), 5, (0, 255, 255), -1)
                cv2.putText(
                    frame,
                    f"{self.target_key_label}: dx={dx}px dy={dy}px",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2
                )

                self.target_kf_last_time = now_sec
                cv2.putText(frame, f"STATE: {self.status_text}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                cv2.imshow("ArUco Detection", frame)
                key = cv2.waitKey(1) & 0xFF
                if key != 255:
                    if key in (10, 13):
                        if self.input_buffer:
                            raw_input = self.input_buffer.strip()
                            if raw_input.startswith('>'):
                                raw_input = raw_input[1:].strip()

                            if len(raw_input) > 1:
                                self.typing_queue.clear()
                                for char in raw_input.lower():
                                    if char.strip():
                                        self.typing_queue.append(char)
                                if self.typing_queue:
                                    self.autonomous_mode = True
                                    self.current_typing_target = self.typing_queue.popleft()
                                    self.target_key_label = self.current_typing_target
                                    self.homography_stable_frames = 0
                                    self.pose_lock_start_time = None
                                    self.await_target_acquire = True
                                    self.get_logger().info(
                                        f"Autonomous typing queued: {raw_input} ({len(self.typing_queue)} chars)"
                                    )
                            else:
                                self.target_key_label = raw_input
                                self.autonomous_mode = False
                                self.typing_queue.clear()
                                self.current_typing_target = None
                                self.await_target_acquire = True
                                self.get_logger().info(f"Target key set to: {self.target_key_label}")

                            self.input_buffer = ""
                            self.key_track_active = False
                            self.key_track_point = None
                            self.key_track_lost = 0
                            self.target_kf_initialized = False
                            self.target_kf_last_time = None
                            self.layout_locked = False
                            self.layout_lock_M = None
                            self.layout_lock_M_inv = None
                            self.layout_lock_K = None
                            self.layout_lock_K_inv = None
                            self.layout_lock_quad = None
                    elif key in (ord('d'), ord('D')):
                        if self.autonomous_mode and self.current_typing_target is not None:
                            self.get_logger().info(f"DONE: '{self.current_typing_target}'")
                            self.complete_current_key()
                            self.typing_cooldown_until = now_sec + self.typing_cooldown_duration
                            self.key_track_active = False
                            self.key_track_point = None
                            self.key_track_lost = 0
                            self.target_kf_initialized = False
                            self.target_kf_last_time = None
                            self.pose_lock_start_time = None
                            self.layout_locked = False
                            self.layout_lock_M = None
                            self.layout_lock_M_inv = None
                            self.layout_lock_K = None
                            self.layout_lock_K_inv = None
                            self.layout_lock_quad = None
                    elif key in (8, 127):
                        self.input_buffer = self.input_buffer[:-1]
                    elif 32 <= key <= 126:
                        self.input_buffer += chr(key)

                self.prev_gray = gray_frame
                self.publish_debug_image(frame, header)
                if self.key_track_active:
                    return
            else:
                self.pose_lock = False
                self.key_track_active = False
                self.key_track_point = None
                self.key_track_lost = 0
                self.layout_locked = False
                self.layout_lock_M = None
                self.layout_lock_M_inv = None
                self.layout_lock_K = None
                self.layout_lock_K_inv = None
                self.layout_lock_quad = None

        self.status_text = "GLOBAL"
        
        # Use optimized multi-threshold detection
        corners, ids, _ = self.detect_markers_multi_threshold(gray_frame)

        # Update state with any currently visible markers
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for marker_id, marker_corners in zip(ids.flatten(), corners):
                if self.marker_ref_is_bottom_right and marker_corners is not None and marker_corners.shape[1] == 4:
                    # Rotate corners so bottom-right reference becomes top-left
                    marker_corners = np.roll(marker_corners, -2, axis=1)
                self.last_known_corners[marker_id] = marker_corners
                self.last_seen_timestamp[marker_id] = now_sec
        
        # --- ID-Agnostic Logic: work if exactly 4 markers are seen recently ---
        
        # 1. Filter for markers that are "fresh"
        fresh_markers = {}
        for mid, corner_data in self.last_known_corners.items():
            if (now_sec - self.last_seen_timestamp.get(mid, 0.0)) < self.STALE_THRESHOLD_SECONDS:
                fresh_markers[mid] = corner_data

        aruco_visible = len(fresh_markers) == 4

        have_pose = False
        M = None
        M_inv = None
        warped_roi = None
        keyboard_quad = None
        roi_debug = None
        w_dst, h_dst = 800, 400
        corner_ids = None

        # 2. Proceed if we have exactly 4 fresh markers
        if aruco_visible and not self.key_track_active:
            self.status_text = "GLOBAL:ARUCO"

            self.get_logger().info(f"Found 4 fresh markers.", throttle_duration_sec=1.0)
            # 3. Spatially sort the four markers regardless of their IDs
            centers = {mid: np.mean(c[0], axis=0) for mid, c in fresh_markers.items()}

            sorted_by_y = sorted(centers.items(), key=lambda item: item[1][1])
            top_row = sorted(sorted_by_y[:2], key=lambda item: item[1][0])
            bottom_row = sorted(sorted_by_y[2:], key=lambda item: item[1][0])

            tl_id, _ = top_row[0]
            tr_id, _ = top_row[1]
            bl_id, _ = bottom_row[0]
            br_id, _ = bottom_row[1]

            corner_ids = (tl_id, tr_id, br_id, bl_id)

            pts_src = np.array([
                fresh_markers[tl_id][0][0],
                fresh_markers[tr_id][0][1],
                fresh_markers[br_id][0][2],
                fresh_markers[bl_id][0][3]
            ], dtype="float32")

            pts_dst = np.array(
                [[0, 0], [w_dst - 1, 0], [w_dst - 1, h_dst - 1], [0, h_dst - 1]],
                dtype="float32"
            )

            M = cv2.getPerspectiveTransform(pts_src, pts_dst)
            M_inv = cv2.getPerspectiveTransform(pts_dst, pts_src)
            warped_roi = cv2.warpPerspective(frame, M, (w_dst, h_dst))
            have_pose = True

            # Stability from homography delta (image space)
            if self.last_M_for_stability is not None:
                cur = cv2.perspectiveTransform(pts_src.reshape(4, 1, 2), M)
                prev = cv2.perspectiveTransform(pts_src.reshape(4, 1, 2), self.last_M_for_stability)
                delta = np.mean(np.linalg.norm(cur - prev, axis=2))
                if delta <= self.stable_thresh_px:
                    self.stable_frames += 1
                    self.homography_stable_frames += 1
                else:
                    self.stable_frames = 0
                    self.last_target_pos = None
                    self.homography_stable_frames = 0
            else:
                self.stable_frames = 0
                self.last_target_pos = None
                self.homography_stable_frames = 0

            self.last_M_for_stability = M.copy()

            # Save strong homography
            self.last_good_M = M
            self.last_good_M_inv = M_inv
            self.last_good_M_time = now_sec

            if not self.layout_locked:
                keyboard_quad, roi_debug = self.detect_keyboard_contour(warped_roi.copy())
                cv2.imshow("Warped ROI", warped_roi)
                cv2.imshow("Keyboard Detection Debug", roi_debug)

                if keyboard_quad is None:
                    keyboard_quad = np.array(
                        [[0, 0], [w_dst - 1, 0], [w_dst - 1, h_dst - 1], [0, h_dst - 1]],
                        dtype="float32"
                    )

                if keyboard_quad is not None:
                    if aruco_visible and corner_ids is not None:
                        keyboard_quad, snap_flags = self.snap_keyboard_quad_to_roi(
                            keyboard_quad, M, fresh_markers, corner_ids, w_dst, h_dst
                        )
                        if self.keyboard_snap_debug:
                            self.draw_snap_debug(warped_roi, snap_flags, w_dst, h_dst)
                            if any(snap_flags):
                                labels = ["TL", "TR", "BR", "BL"]
                                active = [labels[i] for i, f in enumerate(snap_flags) if f]
                                self.get_logger().info(
                                    f"Snap active: {','.join(active)}",
                                    throttle_duration_sec=1.0
                                )
                    quad_frame = cv2.perspectiveTransform(
                        keyboard_quad.reshape(4, 1, 2).astype(np.float32), M_inv
                    ).reshape(4, 2)
                    self.last_keyboard_quad_frame = quad_frame
                    self.init_flow_points(gray_frame, quad_frame)

        if have_pose:
            if self.key_track_active:
                target_real = None
                if self.prev_gray is not None and self.key_track_point is not None:
                    p1, st, err = cv2.calcOpticalFlowPyrLK(
                        self.prev_gray, gray_frame, self.key_track_point, None,
                        winSize=(21, 21), maxLevel=3,
                        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
                    )

                    if p1 is not None and st is not None and err is not None and st[0][0] == 1 and err[0][0] <= self.key_track_err_thresh:
                        self.key_track_point = p1
                        target_real = p1[0][0]
                        self.key_track_lost = 0
                    else:
                        self.key_track_lost += 1
                        if self.key_track_lost >= self.key_track_max_lost:
                            self.key_track_active = False
                            self.key_track_point = None
                            self.key_track_lost = 0
                            self.layout_locked = False
                            self.layout_lock_M = None
                            self.layout_lock_M_inv = None
                            self.layout_lock_K = None
                            self.layout_lock_K_inv = None
                            self.layout_lock_quad = None

                if target_real is not None:
                    if self.target_kf_last_time is None:
                        dt = 1.0 / 30.0
                    else:
                        dt = max(1e-3, now_sec - self.target_kf_last_time)

                    filtered_target, self.target_kf_initialized = self.update_kf_2d(
                        self.target_kf,
                        self.target_kf_initialized,
                        target_real,
                        dt
                    )

                    tx, ty = int(filtered_target[0]), int(filtered_target[1])
                    cx_frame, cy_frame = frame.shape[1] // 2, frame.shape[0] // 2
                    dx = tx - cx_frame
                    dy = ty - cy_frame

                    self.pose_lock = True

                    cv2.circle(frame, (tx, ty), 6, (0, 0, 255), -1)
                    cv2.line(frame, (cx_frame, cy_frame), (tx, ty), (0, 0, 255), 3)
                    cv2.circle(frame, (cx_frame, cy_frame), 5, (0, 255, 255), -1)
                    cv2.putText(
                        frame,
                        f"{self.target_key_label}: dx={dx}px dy={dy}px",
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 0, 255),
                        2
                    )

                    self.target_kf_last_time = now_sec
                else:
                    self.pose_lock = False
                    self.layout_locked = False
                    self.layout_lock_M = None
                    self.layout_lock_M_inv = None
                    self.layout_lock_K = None
                    self.layout_lock_K_inv = None
                    self.layout_lock_quad = None

            else:
                if self.layout_locked:
                    cv2.putText(frame, f"STATE: {self.status_text}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                    cv2.imshow("ArUco Detection", frame)
                    self.prev_gray = gray_frame
                    self.publish_debug_image(frame, header)
                    return

                if not self.layout_locked and keyboard_quad is None and warped_roi is not None:
                    keyboard_quad, roi_debug = self.detect_keyboard_contour(warped_roi.copy())

                cv2.imshow("Warped ROI", warped_roi)
                if roi_debug is not None:
                    cv2.imshow("Keyboard Detection Debug", roi_debug)

                # Smooth/hold keyboard quad to reduce blinking
                now_sec = self.get_clock().now().nanoseconds * 1e-9
                if not self.layout_locked:
                    if keyboard_quad is not None:
                        self.last_keyboard_quad_time = now_sec
                    else:
                        if (self.last_keyboard_quad is not None and
                                (now_sec - self.last_keyboard_quad_time) > self.keyboard_quad_hold_seconds):
                            self.last_keyboard_quad = None

                    if self.last_keyboard_quad is not None:
                        keyboard_quad = self.last_keyboard_quad

                if keyboard_quad is not None:
                    if aruco_visible and corner_ids is not None:
                        keyboard_quad, snap_flags = self.snap_keyboard_quad_to_roi(
                            keyboard_quad, M, fresh_markers, corner_ids, w_dst, h_dst
                        )
                        if self.keyboard_snap_debug:
                            self.draw_snap_debug(warped_roi, snap_flags, w_dst, h_dst)
                            if any(snap_flags):
                                labels = ["TL", "TR", "BR", "BL"]
                                active = [labels[i] for i, f in enumerate(snap_flags) if f]
                                self.get_logger().info(
                                    f"Snap active: {','.join(active)}",
                                    throttle_duration_sec=1.0
                                )
                    # Build canonical keyboard space
                    kb_w, kb_h = self.keyboard_size
                    kb_dst = np.array(
                        [[0, 0], [kb_w - 1, 0], [kb_w - 1, kb_h - 1], [0, kb_h - 1]],
                        dtype="float32"
                    )
                    K = cv2.getPerspectiveTransform(keyboard_quad, kb_dst)
                    K_inv = cv2.getPerspectiveTransform(kb_dst, keyboard_quad)

                    frame_h, frame_w = frame.shape[:2]
                    frame_corners = np.array(
                        [[[0, 0]], [[frame_w - 1, 0]], [[frame_w - 1, frame_h - 1]], [[0, frame_h - 1]]],
                        dtype="float32"
                    )
                    visible_poly_warp = cv2.perspectiveTransform(frame_corners, M)

                    # Precompute which keys are visible in current camera view (warped space)
                    visible_labels = set()
                    for key in self.keyboard_layout:
                        x = key["x"] * kb_w
                        y = key["y"] * kb_h
                        w = key["w"] * kb_w
                        h = key["h"] * kb_h

                        key_quad = np.array([
                            [[x, y]],
                            [[x + w, y]],
                            [[x + w, y + h]],
                            [[x, y + h]]
                        ], dtype="float32")

                        key_in_warp = cv2.perspectiveTransform(key_quad, K_inv)

                        if self.is_key_visible_warp(key_in_warp, visible_poly_warp):
                            visible_labels.add(self.normalize_key_label(key["label"]))

                    # Draw layout in canonical keyboard space
                    layout_img = np.zeros((kb_h, kb_w, 3), dtype=np.uint8)
                    target_key = None
                    for key in self.keyboard_layout:
                        if self.normalize_key_label(key["label"]) not in visible_labels:
                            continue
                        x = int(key["x"] * kb_w)
                        y = int(key["y"] * kb_h)
                        w = int(key["w"] * kb_w)
                        h = int(key["h"] * kb_h)
                        cv2.rectangle(layout_img, (x, y), (x + w, y + h), (0, 255, 0), 1)
                        cv2.putText(layout_img, key["label"], (x + 3, y + int(h * 0.6)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                        cx = x + w // 2
                        cy = y + h // 2
                        is_target = self.normalize_key_label(key["label"]) == self.normalize_key_label(self.target_key_label)
                        if is_target:
                            target_key = key
                            cv2.circle(layout_img, (cx, cy), 3, (0, 0, 255), -1)
                        else:
                            cv2.circle(layout_img, (cx, cy), 2, (255, 255, 255), -1)

                    cv2.imshow("Keyboard Layout (Canonical)", layout_img)

                    # Project layout to warped ROI
                    warped_overlay = warped_roi.copy()
                    target_center_warp = None
                    for key in self.keyboard_layout:
                        if self.normalize_key_label(key["label"]) not in visible_labels:
                            continue
                        x = key["x"] * kb_w
                        y = key["y"] * kb_h
                        w = key["w"] * kb_w
                        h = key["h"] * kb_h

                        key_quad = np.array([
                            [[x, y]],
                            [[x + w, y]],
                            [[x + w, y + h]],
                            [[x, y + h]]
                        ], dtype="float32")

                        key_in_warp = cv2.perspectiveTransform(key_quad, K_inv)
                        key_in_warp_i = key_in_warp.astype(int)
                        cv2.polylines(warped_overlay, [key_in_warp_i], True, (0, 255, 0), 1)
                        center = key_in_warp.reshape(4, 2).mean(axis=0).astype(int)
                        is_target = self.normalize_key_label(key["label"]) == self.normalize_key_label(self.target_key_label)
                        if is_target:
                            target_center_warp = center
                            cv2.circle(warped_overlay, (int(center[0]), int(center[1])), 3, (0, 0, 255), -1)
                        else:
                            cv2.circle(warped_overlay, (int(center[0]), int(center[1])), 2, (255, 255, 255), -1)

                    cv2.imshow("Keyboard Layout (Warped)", warped_overlay)

                    # Map target key center to real camera frame and show coordinates
                    target_real = None
                    if target_center_warp is not None:
                        target_corner = np.array([[[target_center_warp[0], target_center_warp[1]]]], dtype="float32")

                        if self.layout_locked and self.layout_lock_M_inv is not None:
                            target_real = cv2.perspectiveTransform(
                                target_corner,
                                self.layout_lock_M_inv
                            )[0][0]
                        else:
                            target_real = cv2.perspectiveTransform(
                                target_corner,
                                M_inv
                            )[0][0]


                    if target_real is not None:
                        if self.await_target_acquire:
                            self.await_target_acquire = False
                            self.homography_stable_frames = 0
                            self.pose_lock_start_time = None
                        if self.target_kf_last_time is None:
                            dt = 1.0 / 30.0
                        else:
                            dt = max(1e-3, now_sec - self.target_kf_last_time)

                        filtered_target, self.target_kf_initialized = self.update_kf_2d(
                            self.target_kf,
                            self.target_kf_initialized,
                            target_real,
                            dt
                        )

                        tx, ty = int(filtered_target[0]), int(filtered_target[1])

                        cx_frame, cy_frame = frame_w // 2, frame_h // 2
                        dx = tx - cx_frame
                        dy = ty - cy_frame

                        stable_ready = aruco_visible and (
                            self.homography_stable_frames >= self.homography_stable_required
                        )

                        if self.autonomous_mode and self.current_typing_target is not None:
                            self.pose_lock = stable_ready
                        else:
                            if aruco_visible:
                                self.pose_lock = stable_ready
                            else:
                                self.pose_lock = (abs(dx) < self.pose_lock_threshold_px and
                                                  abs(dy) < self.pose_lock_threshold_px)

                        if self.pose_lock and not self.key_track_active and not self.await_target_acquire:
                            if self.pose_lock_start_time is None:
                                self.pose_lock_start_time = now_sec

                            if (now_sec - self.pose_lock_start_time) >= self.pose_lock_delay_sec:
                                self.key_track_active = True
                                self.key_track_point = np.array([[[float(tx), float(ty)]]], dtype=np.float32)
                                self.key_track_lost = 0
                                self.layout_locked = True
                                self.layout_lock_M = M.copy() if M is not None else None
                                self.layout_lock_M_inv = M_inv.copy() if M_inv is not None else None
                                self.layout_lock_K = K.copy() if K is not None else None
                                self.layout_lock_K_inv = K_inv.copy() if K_inv is not None else None
                                self.layout_lock_quad = keyboard_quad.copy() if keyboard_quad is not None else None

                        if not self.pose_lock:
                            self.key_track_active = False
                            self.key_track_point = None
                            self.key_track_lost = 0
                            self.layout_locked = False
                            self.layout_lock_M = None
                            self.layout_lock_M_inv = None
                            self.layout_lock_K = None
                            self.layout_lock_K_inv = None
                            self.layout_lock_quad = None
                            self.pose_lock_start_time = None

                        cv2.circle(frame, (tx, ty), 6, (0, 0, 255), -1)
                        cv2.line(frame, (cx_frame, cy_frame), (tx, ty), (0, 0, 255), 3)
                        cv2.circle(frame, (cx_frame, cy_frame), 5, (0, 255, 255), -1)
                        cv2.putText(
                            frame,
                            f"{self.target_key_label}: dx={dx}px dy={dy}px",
                            (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (0, 0, 255),
                            2
                        )
                    else:
                        self.pose_lock = False
                        self.key_track_active = False
                        self.key_track_point = None
                        self.layout_locked = False
                        self.layout_lock_M = None
                        self.layout_lock_M_inv = None
                        self.layout_lock_K = None
                        self.layout_lock_K_inv = None
                        self.layout_lock_quad = None
                        self.pose_lock_start_time = None

                    # Map keyboard origin (top-left) to frame with smoothing
                    warped_corner = np.array([[[keyboard_quad[0][0], keyboard_quad[0][1]]]], dtype="float32")
                    real_corner = cv2.perspectiveTransform(warped_corner, M_inv)[0][0]
                    if self.origin_kf_last_time is None:
                        dt = 1.0 / 30.0
                    else:
                        dt = max(1e-3, now_sec - self.origin_kf_last_time)

                    filtered_origin, self.origin_kf_initialized = self.update_kf_2d(
                        self.origin_kf,
                        self.origin_kf_initialized,
                        real_corner,
                        dt
                    )

                    cx, cy = int(filtered_origin[0]), int(filtered_origin[1])

                    cv2.circle(frame, (cx, cy), 8, (0, 0, 255), -1)
                    cv2.putText(frame, "Keyboard Origin", (cx + 10, cy + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                    self.origin_kf_last_time = now_sec

                    # Update frame-space quad for optical flow fallback
                    if not self.layout_locked:
                        quad_frame = cv2.perspectiveTransform(
                            keyboard_quad.reshape(4, 1, 2).astype(np.float32), M_inv
                        ).reshape(4, 2)
                        self.last_keyboard_quad_frame = quad_frame
        else:
            self.get_logger().info(f"Found {len(fresh_markers)} fresh markers, waiting for exactly 4.", throttle_duration_sec=1.0)

        # Display autonomous mode status
        if self.autonomous_mode:
            queue_display = ''.join(list(self.typing_queue)[:10])  # Show first 10 chars
            cv2.putText(frame, f"AUTO: [{self.current_typing_target}] Queue: {queue_display}", 
                       (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        cv2.putText(frame, f"STATE: {self.status_text}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        self.publish_markers(corners, ids, header)
        self.publish_debug_image(frame, header)
        cv2.imshow("ArUco Detection", frame)
        
        # Runtime input: type key label/word and press Enter
        key = cv2.waitKey(1) & 0xFF
        if key != 255:
            if key in (10, 13):  # Enter
                if self.input_buffer:
                    raw_input = self.input_buffer.strip()
                    if raw_input.startswith('>'):
                        raw_input = raw_input[1:].strip()

                    if len(raw_input) > 1:
                        self.typing_queue.clear()
                        for char in raw_input.lower():
                            if char.strip():
                                self.typing_queue.append(char)
                        if self.typing_queue:
                            self.autonomous_mode = True
                            self.current_typing_target = self.typing_queue.popleft()
                            self.target_key_label = self.current_typing_target
                            self.homography_stable_frames = 0
                            self.pose_lock_start_time = None
                            self.await_target_acquire = True
                            self.get_logger().info(
                                f"Autonomous typing queued: {raw_input} ({len(self.typing_queue)} chars)"
                            )
                    else:
                        # Single key mode
                        self.target_key_label = raw_input
                        self.autonomous_mode = False
                        self.typing_queue.clear()
                        self.current_typing_target = None
                        self.await_target_acquire = True
                        self.get_logger().info(f"Target key set to: {self.target_key_label}")
                    
                    self.input_buffer = ""
                    self.key_track_active = False
                    self.key_track_point = None
                    self.key_track_lost = 0
                    self.target_kf_initialized = False
                    self.target_kf_last_time = None
                    self.layout_locked = False
                    self.layout_lock_M = None
                    self.layout_lock_M_inv = None
                    self.layout_lock_K = None
                    self.layout_lock_K_inv = None
                    self.layout_lock_quad = None
            elif key in (8, 127):  # Backspace
                self.input_buffer = self.input_buffer[:-1]
            elif 32 <= key <= 126:
                self.input_buffer += chr(key)

        self.prev_gray = gray_frame

    def publish_markers(self, corners, ids, header):
        marker_array = MarkerArray()
        current_ids = set()

        if ids is not None:
            for marker_id, marker_corners in zip(ids.flatten(), corners):
                marker_id_int = int(marker_id)
                current_ids.add(marker_id_int)

                marker = Marker()
                marker.header = header
                marker.ns = 'aruco'
                marker.id = marker_id_int
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.pose.orientation.w = 1.0
                marker.scale.x = 2.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0

                pts = marker_corners.reshape(4, 2)
                for x, y in pts:
                    marker.points.append(Point(x=float(x), y=float(y), z=0.0))
                marker.points.append(Point(x=float(pts[0][0]), y=float(pts[0][1]), z=0.0))

                marker_array.markers.append(marker)

        for marker_id in self.last_marker_ids - current_ids:
            marker = Marker()
            marker.header = header
            marker.ns = 'aruco'
            marker.id = int(marker_id)
            marker.action = Marker.DELETE
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)
        self.last_marker_ids = current_ids

    def publish_debug_image(self, frame, header):
        debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        debug_msg.header = header
        self.debug_image_pub.publish(debug_msg)


def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()