
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np


class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_node')

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        # ArUcos - Using new API
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_50)
        
        # Try new API first, fallback to old API
        try:
            # New API (OpenCV 4.7+)
            self.parameters = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
            self.use_new_api = True
            self.get_logger().info("Using new ArUco API")
        except AttributeError:
            # Old API (OpenCV < 4.7)
            self.parameters = cv2.aruco.DetectorParameters_create()
            self.detector = None
            self.use_new_api = False
            self.get_logger().info("Using legacy ArUco API")
        
        # Parámetros más agresivos para detectar los 4 ArUcos simultáneamente
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.parameters.cornerRefinementWinSize = 5
        self.parameters.cornerRefinementMaxIterations = 100
        
        # Ventana adaptativa muy amplia para diferentes tamaños y condiciones de luz
        self.parameters.adaptiveThreshWinSizeMin = 3
        self.parameters.adaptiveThreshWinSizeMax = 35
        self.parameters.adaptiveThreshWinSizeStep = 4
        self.parameters.adaptiveThreshConstant = 5
        
        # Tamaños de marcador muy permisivos para marcadores pequeños/lejanos
        self.parameters.minMarkerPerimeterRate = 0.01
        self.parameters.maxMarkerPerimeterRate = 6.0
        self.parameters.polygonalApproxAccuracyRate = 0.05
        
        # Distancias reducidas para marcadores pequeños
        self.parameters.minCornerDistanceRate = 0.03
        self.parameters.minMarkerDistanceRate = 0.03
        self.parameters.minDistanceToBorder = 1
        
        # Alta tolerancia a errores para marcadores con ruido o baja resolución
        self.parameters.errorCorrectionRate = 0.8
        self.parameters.maxErroneousBitsInBorderRate = 0.5
        
        # Umbral Otsu bajo para detectar con poco contraste
        self.parameters.minOtsuStdDev = 4.0
        
        # Perspectiva más permisiva
        self.parameters.perspectiveRemovePixelPerCell = 6
        self.parameters.perspectiveRemoveIgnoredMarginPerCell = 0.13
        
        self.get_logger().info("ArUco node started - Aggressive detection for 4 markers")

        # Pose stability parameters
        self.stable_frames = 0
        self.required_stable_frames = 90
        self.stable_thresh_px = .1
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

        # Target key to track (set this label, e.g., "Space")
        self.target_key_label = "Space"
        # Runtime input buffer for changing target key
        self.input_buffer = ""

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

        # Normalized offsets (resolution-agnostic)
        self.last_dx_n = None
        self.last_dy_n = None
        self.pose_lock_threshold_n = 0.05  # 5% del semieje


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
        self.pose_lock_delay_sec = 2.0
        self.pose_lock_start_time = None

        # Homography stability tracking
        self.last_M_for_stability = None

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

    def normalized_offset(self, tx, ty, frame_w, frame_h):
        cx = frame_w * 0.5
        cy = frame_h * 0.5

        dx_n = (tx - cx) / cx
        dy_n = (ty - cy) / cy

        return dx_n, dy_n



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
        
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)

        # Primary: Canny edges
        edges = cv2.Canny(gray, 30, 120)
        kernel = np.ones((5, 5), np.uint8)
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Fallback: adaptive threshold + morphology to get keyboard blob
        if not contours:
            thresh = cv2.adaptiveThreshold(
                gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 25, 7
            )
            thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None, roi

        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        best_quad = None
        best_area = 0.0

        for contour in contours:
            area = cv2.contourArea(contour)

            # Ensure contour is a large part of the ROI (relaxed)
            if area < (roi_area * 0.2):
                continue

            rect = cv2.minAreaRect(contour)
            (w, h) = rect[1]

            if w == 0 or h == 0:
                continue

            aspect_ratio = max(w, h) / min(w, h)

            # Aspect ratio for a rectified keyboard
            if 1.3 < aspect_ratio < 5.0:
                box = cv2.boxPoints(rect)
                ordered = self.order_points(box)

                if area > best_area:
                    best_area = area
                    best_quad = ordered

        # If we found a candidate quad, decide whether it represents the whole keyboard
        if best_quad is not None:
            # If quad is too small, assume full ROI is keyboard (markers near edges)
            if best_area < (roi_area * self.keyboard_full_roi_threshold):
                full = np.array([[0, 0], [w_roi - 1, 0], [w_roi - 1, h_roi - 1], [0, h_roi - 1]], dtype="float32")
                best_quad = full

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
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        # ===== PHASE 1: DETECTION (stateless) =====
        # Obtain raw measurement from either optical flow or homography
        target_measurement = None
        
        # Fine tracking mode: skip pose/warp/layout updates
        if self.key_track_active:
            self.status_text = "TRACKING"
            if self.prev_gray is not None and self.key_track_point is not None:
                p1, st, err = cv2.calcOpticalFlowPyrLK(
                    self.prev_gray, gray_frame, self.key_track_point, None,
                    winSize=(21, 21), maxLevel=3,
                    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
                )

                if p1 is not None and st is not None and err is not None and st[0][0] == 1 and err[0][0] <= self.key_track_err_thresh:
                    self.key_track_point = p1
                    target_measurement = p1[0][0]
                    self.key_track_lost = 0
                else:
                    self.key_track_lost += 1
                    if self.key_track_lost >= self.key_track_max_lost:
                        self.key_track_active = False
                        self.key_track_point = None
                        self.key_track_lost = 0
                        self.pose_lock_start_time = None

            if target_measurement is None:
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

        # Detect ArUco markers (compatible with both old and new API)
        if self.use_new_api:
            corners, ids, rejected = self.detector.detectMarkers(gray_frame)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray_frame,
                self.dictionary,
                parameters=self.parameters
            )


        # Update state with any currently visible markers
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for marker_id, marker_corners in zip(ids.flatten(), corners):
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
        K = None
        K_inv = None
        warped_roi = None
        keyboard_quad = None
        roi_debug = None
        w_dst, h_dst = 800, 400

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
                else:
                    self.stable_frames = 0
                    self.last_target_pos = None
            else:
                self.stable_frames = 0
                self.last_target_pos = None

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
                    quad_frame = cv2.perspectiveTransform(
                        keyboard_quad.reshape(4, 1, 2).astype(np.float32), M_inv
                    ).reshape(4, 2)
                    self.last_keyboard_quad_frame = quad_frame
                    self.init_flow_points(gray_frame, quad_frame)

        if have_pose:
            if self.layout_locked:
                cv2.putText(frame, f"STATE: {self.status_text}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                cv2.imshow("ArUco Detection", frame)
                self.prev_gray = gray_frame
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

                # Map target key center to real camera frame (stateless detection)
                if not self.key_track_active and target_center_warp is not None:
                    target_corner = np.array([[[target_center_warp[0], target_center_warp[1]]]], dtype="float32")

                    if self.layout_locked and self.layout_lock_M_inv is not None:
                        target_measurement = cv2.perspectiveTransform(
                            target_corner,
                            self.layout_lock_M_inv
                        )[0][0]
                    else:
                        target_measurement = cv2.perspectiveTransform(
                            target_corner,
                            M_inv
                        )[0][0]

                if target_measurement is None:
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
            self.get_logger().info(f"Found {len(fresh_markers)} fresh markers yn, waiting for exactly 4.", throttle_duration_sec=1.0)
        
        # ===== PHASE 2: ESTIMATION (single stateful update) =====
        # Update tracker state from measurement (regardless of source)
        if target_measurement is not None:
            if self.target_kf_last_time is None:
                dt = 1.0 / 30.0
            else:
                dt = max(1e-3, now_sec - self.target_kf_last_time)

            filtered_target, self.target_kf_initialized = self.update_kf_2d(
                self.target_kf,
                self.target_kf_initialized,
                target_measurement,
                dt
            )

            tx, ty = int(filtered_target[0]), int(filtered_target[1])
            frame_h, frame_w = frame.shape[:2]
            cx_frame, cy_frame = frame_w // 2, frame_h // 2
            dx_n, dy_n = self.normalized_offset(tx, ty, frame_w, frame_h)
            
            self.last_dx_n = dx_n
            self.last_dy_n = dy_n
            self.target_kf_last_time = now_sec

            # State transition logic
            if self.pose_lock and not self.key_track_active:
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

            if not self.pose_lock and self.key_track_lost >= self.key_track_max_lost:
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

            # Visualization
            cv2.circle(frame, (tx, ty), 6, (0, 0, 255), -1)
            cv2.line(frame, (cx_frame, cy_frame), (tx, ty), (0, 0, 255), 3)
            cv2.circle(frame, (cx_frame, cy_frame), 5, (0, 255, 255), -1)
            cv2.putText(
                frame,
                f"{self.target_key_label}: dx_n={dx_n:.3f}, dy_n={dy_n:.3f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

        # Compute pose_lock from current state
        if self.last_dx_n is not None and self.last_dy_n is not None:
            self.pose_lock = (
                abs(self.last_dx_n) < self.pose_lock_threshold_n and
                abs(self.last_dy_n) < self.pose_lock_threshold_n
            )
        else:
            self.pose_lock = False

        cv2.putText(frame, f"STATE: {self.status_text}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.imshow("ArUco Detection", frame)
        # Runtime input: type key label and press Enter to set target
        key = cv2.waitKey(1) & 0xFF
        if key != 255:
            if key in (10, 13):  # Enter
                if self.input_buffer:
                    self.target_key_label = self.input_buffer
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
                    self.pose_lock_start_time = None

            elif key in (8, 127):  # Backspace
                self.input_buffer = self.input_buffer[:-1]
            elif 32 <= key <= 126:
                self.input_buffer += chr(key)

        self.prev_gray = gray_frame


def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
