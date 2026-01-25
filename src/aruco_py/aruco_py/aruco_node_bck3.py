

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

        # ArUco setup
        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_7X7_50
        )

        self.parameters = cv2.aruco.DetectorParameters_create()
        # Tune parameters for better detection
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.parameters.adaptiveThreshWinSizeMin = 3
        self.parameters.adaptiveThreshWinSizeMax = 23
        self.parameters.adaptiveThreshWinSizeStep = 10
        self.parameters.adaptiveThreshConstant = 7
        self.get_logger().info("ArUco node started")

        # --- State for robust detection ---
        # Store the last known corners of each marker
        self.last_known_corners = {}
        # Store the timestamp of when each marker was last seen
        self.last_seen_timestamp = {}
        
        # How long to remember a marker's position (in seconds)
        self.STALE_THRESHOLD_SECONDS = 2.5

        # Smoothing for final keyboard contour
        self.smooth_corners = None
        self.SMOOTH_ALPHA = 0.1

        # Debug visualization
        self.declare_parameter('debug_windows', True)
        self.declare_parameter('debug_stride', 1)
        self.debug_windows = self.get_parameter('debug_windows').get_parameter_value().bool_value
        self.debug_stride = self.get_parameter('debug_stride').get_parameter_value().integer_value
        self.debug_counter = 0

        # Warp size for top-down keyboard view
        self.declare_parameter('warp_width', 900)
        self.declare_parameter('warp_height', 350)
        self.warp_width = self.get_parameter('warp_width').get_parameter_value().integer_value
        self.warp_height = self.get_parameter('warp_height').get_parameter_value().integer_value

        # Performance tuning
        self.declare_parameter('keyboard_scale', 1.0)
        self.declare_parameter('enable_refine', False)
        self.declare_parameter('refine_max_dist', 5)
        self.keyboard_scale = float(self.get_parameter('keyboard_scale').get_parameter_value().double_value)
        self.enable_refine = self.get_parameter('enable_refine').get_parameter_value().bool_value
        self.refine_max_dist = int(self.get_parameter('refine_max_dist').get_parameter_value().integer_value)

        # Edge detection tuning
        self.declare_parameter('canny_low', 50)
        self.declare_parameter('canny_high', 150)
        self.declare_parameter('hough_threshold', 100)
        self.declare_parameter('hough_min_line_ratio', 0.5)
        self.canny_low = int(self.get_parameter('canny_low').get_parameter_value().integer_value)
        self.canny_high = int(self.get_parameter('canny_high').get_parameter_value().integer_value)
        self.hough_threshold = int(self.get_parameter('hough_threshold').get_parameter_value().integer_value)
        self.hough_min_line_ratio = float(self.get_parameter('hough_min_line_ratio').get_parameter_value().double_value)

        # Corner bias correction (pixels to expand quad outward)
        self.declare_parameter('corner_bias', 0.0)
        self.corner_bias = float(self.get_parameter('corner_bias').get_parameter_value().double_value)

        # Auto-tuning
        self.declare_parameter('auto_tune', True)
        self.auto_tune = self.get_parameter('auto_tune').get_parameter_value().bool_value
        self.bias_suggestions = []
        self.auto_applied_bias = False

        # Key grid visualization and physical spacing
        self.declare_parameter('draw_key_grid', True)
        self.declare_parameter('key_gap_mm', 2.0)
        self.declare_parameter('key_size_mm', 19.05)
        self.draw_key_grid = self.get_parameter('draw_key_grid').get_parameter_value().bool_value
        self.key_gap_mm = float(self.get_parameter('key_gap_mm').get_parameter_value().double_value)
        self.key_size_mm = float(self.get_parameter('key_size_mm').get_parameter_value().double_value)
        self.key_map_warped = {}

    def show_debug(self, name, img):
        if not self.debug_windows:
            return
        stride = max(1, int(self.debug_stride))
        if self.debug_counter % stride == 0:
            cv2.imshow(name, img)

    def order_corners(self, pts):
        s = pts.sum(axis=1)
        diff = np.diff(pts, axis=1)
        ordered = np.zeros((4, 2), dtype="float32")
        ordered[0] = pts[np.argmin(s)]      # Top-Left
        ordered[2] = pts[np.argmax(s)]      # Bottom-Right
        ordered[1] = pts[np.argmin(diff)]   # Top-Right
        ordered[3] = pts[np.argmax(diff)]   # Bottom-Left
        return ordered

    def quad_quality(self, pts, area_ratio_min=0.15, area_ratio_max=0.98, img_area=None):
        if not cv2.isContourConvex(pts.reshape(-1, 1, 2).astype(np.int32)):
            return False
        ordered = self.order_corners(pts)
        angles = []
        for i in range(4):
            p0 = ordered[i]
            p1 = ordered[(i - 1) % 4]
            p2 = ordered[(i + 1) % 4]
            v1 = p1 - p0
            v2 = p2 - p0
            denom = (np.linalg.norm(v1) * np.linalg.norm(v2)) + 1e-6
            cosang = np.clip(np.dot(v1, v2) / denom, -1.0, 1.0)
            ang = np.degrees(np.arccos(cosang))
            angles.append(ang)
        if any(a < 35 or a > 145 for a in angles):
            return False
        if img_area is not None:
            area = cv2.contourArea(pts)
            if area < img_area * area_ratio_min or area > img_area * area_ratio_max:
                return False
        return True

    def best_quad_from_points(self, pts, img_area=None):
        if pts.shape[0] < 4:
            return None
        pts_list = [p for p in pts]
        best = None
        best_area = 0
        n = len(pts_list)
        for i in range(n - 3):
            for j in range(i + 1, n - 2):
                for k in range(j + 1, n - 1):
                    for l in range(k + 1, n):
                        quad = np.array([pts_list[i], pts_list[j], pts_list[k], pts_list[l]], dtype="float32")
                        if not self.quad_quality(quad, img_area=img_area):
                            continue
                        area = cv2.contourArea(quad)
                        if area > best_area:
                            best_area = area
                            best = quad
        return best

    def find_marker_quad(self, fresh_markers):
        all_pts = np.concatenate([c[0] for c in fresh_markers.values()], axis=0)
        hull = cv2.convexHull(all_pts)
        if hull is None or len(hull) < 4:
            return None
        peri = cv2.arcLength(hull, True)
        best = None
        for eps_ratio in (0.01, 0.02, 0.03, 0.04, 0.05, 0.07):
            approx = cv2.approxPolyDP(hull, eps_ratio * peri, True)
            if len(approx) == 4:
                quad = approx.reshape(-1, 2).astype("float32")
                best = quad
                break
        if best is None:
            candidates = hull.reshape(-1, 2).astype("float32")
            if candidates.shape[0] <= 14:
                best = self.best_quad_from_points(candidates)
        if best is None:
            x, y, w, h = cv2.boundingRect(hull)
            best = np.array([[x, y], [x + w, y], [x + w, y + h], [x, y + h]], dtype="float32")
        return self.order_corners(best)

    def hough_rectangle(self, edges, shape):
        min_len = int(self.hough_min_line_ratio * min(shape[0], shape[1]))
        
        # Try detection with current params
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=self.hough_threshold,
                                minLineLength=min_len,
                                maxLineGap=30)
        
        # Auto-tune if enabled and detection fails
        if lines is None and self.auto_tune:
            self.get_logger().warn(f"No lines detected with threshold={self.hough_threshold}. Auto-tuning...")
            for thresh in [80, 60, 100, 120, 40]:
                lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=thresh,
                                       minLineLength=min_len, maxLineGap=30)
                if lines is not None and len(lines) >= 4:
                    self.get_logger().info(f"Auto-tune: Found {len(lines)} lines with threshold={thresh}")
                    self.hough_threshold = thresh
                    break
        
        if lines is None:
            self.get_logger().error("Hough detection failed. Try: corner_bias, canny thresholds, or better lighting.")
            return None

        h_lines = []
        v_lines = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            angle = abs(np.degrees(np.arctan2(y2 - y1, x2 - x1)))
            if angle < 15 or angle > 165:
                h_lines.append((x1, y1, x2, y2))
            elif 75 < angle < 105:
                v_lines.append((x1, y1, x2, y2))

        self.get_logger().info(f"Detected {len(lines)} total lines: {len(h_lines)} horizontal, {len(v_lines)} vertical", throttle_duration_sec=2.0)

        if len(h_lines) < 2 or len(v_lines) < 2:
            self.get_logger().warn(f"Insufficient lines: {len(h_lines)}H, {len(v_lines)}V. Need at least 2 of each.")
            if self.auto_tune:
                self.get_logger().info("Try lowering hough_threshold or canny_high, or improve keyboard contrast.")
            return None

        def line_from_points(p1, p2):
            x1, y1 = float(p1[0]), float(p1[1])
            x2, y2 = float(p2[0]), float(p2[1])
            a = y1 - y2
            b = x2 - x1
            c = x1 * y2 - x2 * y1
            norm = np.hypot(a, b) + 1e-6
            return a / norm, b / norm, c / norm

        def intersect(l1, l2):
            a1, b1, c1 = l1
            a2, b2, c2 = l2
            d = a1 * b2 - a2 * b1
            if abs(d) < 1e-6:
                return None
            x = float((b1 * c2 - b2 * c1) / d)
            y = float((a2 * c1 - a1 * c2) / d)
            return np.array([x, y], dtype="float32")

        h_lines.sort(key=lambda l: (l[1] + l[3]) / 2.0)
        v_lines.sort(key=lambda l: (l[0] + l[2]) / 2.0)
        top = h_lines[0]
        bottom = h_lines[-1]
        left = v_lines[0]
        right = v_lines[-1]

        l_top = line_from_points((top[0], top[1]), (top[2], top[3]))
        l_bottom = line_from_points((bottom[0], bottom[1]), (bottom[2], bottom[3]))
        l_left = line_from_points((left[0], left[1]), (left[2], left[3]))
        l_right = line_from_points((right[0], right[1]), (right[2], right[3]))

        tl = intersect(l_top, l_left)
        tr = intersect(l_top, l_right)
        br = intersect(l_bottom, l_right)
        bl = intersect(l_bottom, l_left)
        if tl is None or tr is None or br is None or bl is None:
            return None
        return np.array([tl, tr, br, bl], dtype="float32")

    def refine_quad_with_edges(self, quad, edges, max_dist=6):
        # Refine quad corners by fitting lines to edge points near each side
        h, w = edges.shape[:2]
        edge_pts = np.column_stack(np.where(edges > 0))  # (y, x)
        if edge_pts.shape[0] < 50:
            return quad

        # Subsample edge points to speed up fitting
        if edge_pts.shape[0] > 5000:
            step = int(np.ceil(edge_pts.shape[0] / 5000))
            edge_pts = edge_pts[::step]

        quad = self.order_corners(quad)

        def line_from_points(p1, p2):
            x1, y1 = float(p1[0]), float(p1[1])
            x2, y2 = float(p2[0]), float(p2[1])
            a = y1 - y2
            b = x2 - x1
            c = x1 * y2 - x2 * y1
            norm = np.hypot(a, b) + 1e-6
            return a / norm, b / norm, c / norm

        def point_line_dist(line, x, y):
            a, b, c = line
            return abs(a * x + b * y + c)

        def intersect(l1, l2):
            a1, b1, c1 = l1
            a2, b2, c2 = l2
            d = a1 * b2 - a2 * b1
            if abs(d) < 1e-6:
                return None
            x = float((b1 * c2 - b2 * c1) / d)
            y = float((a2 * c1 - a1 * c2) / d)
            return np.array([x, y], dtype="float32")

        lines = []
        for i in range(4):
            p1 = quad[i]
            p2 = quad[(i + 1) % 4]
            base_line = line_from_points(p1, p2)

            # filter edge points near this side and within side's projection
            x1, y1 = p1
            x2, y2 = p2
            vx, vy = (x2 - x1), (y2 - y1)
            seg_len2 = vx * vx + vy * vy + 1e-6

            near_pts = []
            for (yy, xx) in edge_pts:
                if point_line_dist(base_line, xx, yy) > max_dist:
                    continue
                # projection parameter to keep within segment bounds
                t = ((xx - x1) * vx + (yy - y1) * vy) / seg_len2
                if -0.1 <= t <= 1.1:
                    near_pts.append((xx, yy))

            if len(near_pts) < 30:
                lines.append(base_line)
                continue

            pts = np.array(near_pts, dtype=np.float32)
            [vx, vy, x0, y0] = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)
            vx = float(vx)
            vy = float(vy)
            x0 = float(x0)
            y0 = float(y0)
            a = -vy
            b = vx
            c = -(a * x0 + b * y0)
            norm = np.hypot(a, b) + 1e-6
            lines.append((a / norm, b / norm, c / norm))

        refined = []
        for i in range(4):
            p = intersect(lines[i], lines[(i - 1) % 4])
            if p is None:
                return quad
            refined.append(p)
        refined = np.array(refined, dtype="float32")

        # clamp to image bounds
        refined[:, 0] = np.clip(refined[:, 0], 0, w - 1)
        refined[:, 1] = np.clip(refined[:, 1], 0, h - 1)
        return self.order_corners(refined)

    def detect_keyboard_quad(self, roi):
        h_roi, w_roi, _ = roi.shape
        if w_roi == 0 or h_roi == 0:
            return None, roi

        scale = float(self.keyboard_scale) if self.keyboard_scale > 0 else 1.0
        scale = min(1.0, max(0.3, scale))
        if scale < 1.0:
            roi_small = cv2.resize(roi, (int(w_roi * scale), int(h_roi * scale)), interpolation=cv2.INTER_AREA)
        else:
            roi_small = roi

        h_s, w_s, _ = roi_small.shape

        gray = cv2.cvtColor(roi_small, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(gray, self.canny_low, self.canny_high)
        edges = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=1)
        
        # Diagnostics
        edge_density = np.count_nonzero(edges) / (edges.shape[0] * edges.shape[1])
        self.get_logger().info(f"Edge density: {edge_density:.3f} (Canny: {self.canny_low}/{self.canny_high})", throttle_duration_sec=2.0)
        
        # Auto-tune Canny if edges are too sparse or dense
        if self.auto_tune:
            if edge_density < 0.01:
                self.get_logger().warn(f"Very few edges ({edge_density:.3f}). Lowering Canny: {self.canny_low}/{self.canny_high} → ", throttle_duration_sec=1.0)
                self.canny_low = max(20, self.canny_low - 10)
                self.canny_high = max(50, self.canny_high - 10)
                self.get_logger().warn(f"{self.canny_low}/{self.canny_high}", throttle_duration_sec=1.0)
                edges = cv2.Canny(gray, self.canny_low, self.canny_high)
                edges = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=1)
            elif edge_density > 0.15:
                self.get_logger().warn(f"Too many edges ({edge_density:.3f}). Raising Canny: {self.canny_low}/{self.canny_high} → ", throttle_duration_sec=1.0)
                self.canny_low = min(80, self.canny_low + 10)
                self.canny_high = min(200, self.canny_high + 10)
                self.get_logger().warn(f"{self.canny_low}/{self.canny_high}", throttle_duration_sec=1.0)
                edges = cv2.Canny(gray, self.canny_low, self.canny_high)
                edges = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=1)
        
        self.show_debug("Keyboard Edges", edges)

        quad = self.hough_rectangle(edges, roi_small.shape)
        if quad is None:
            self.get_logger().error("Failed to detect keyboard quad. Check lighting and ArUco markers.")
            return None, roi

        if self.enable_refine:
            quad = self.refine_quad_with_edges(quad, edges, max_dist=max(2, self.refine_max_dist))

        # Apply corner bias correction (expand outward)
        if abs(self.corner_bias) > 0.01:
            center = np.mean(quad, axis=0)
            for i in range(4):
                vec = quad[i] - center
                vec_len = np.linalg.norm(vec) + 1e-6
                quad[i] = quad[i] + (vec / vec_len) * self.corner_bias
        
        # Auto-suggest corner bias if corners look wrong
        if self.auto_tune and abs(self.corner_bias) < 0.01:
            # Check if quad is suspiciously small compared to ROI
            quad_area = cv2.contourArea(quad)
            roi_area = w_s * h_s
            area_ratio = quad_area / roi_area
            
            suggested_bias = None
            if area_ratio < 0.25:
                suggested_bias = 5.0
            elif area_ratio > 0.95:
                suggested_bias = -3.0
            
            if suggested_bias is not None:
                self.bias_suggestions.append(suggested_bias)
                
                # Auto-apply if we've seen consistent suggestions (5 frames)
                if len(self.bias_suggestions) > 5:
                    avg_suggestion = sum(self.bias_suggestions[-5:]) / 5
                    if not self.auto_applied_bias and abs(avg_suggestion - suggested_bias) < 1.0:
                        self.corner_bias = suggested_bias
                        self.auto_applied_bias = True
                        self.get_logger().info(f"✓ AUTO-APPLIED corner_bias={self.corner_bias} based on {len(self.bias_suggestions)} detections")
                        # Reapply with new bias
                        center = np.mean(quad, axis=0)
                        for i in range(4):
                            vec = quad[i] - center
                            vec_len = np.linalg.norm(vec) + 1e-6
                            quad[i] = quad[i] + (vec / vec_len) * self.corner_bias
                    else:
                        self.get_logger().warn(f"Quad area is {area_ratio:.2%} of ROI. Collecting samples... ({len(self.bias_suggestions)}/5)", throttle_duration_sec=5.0)
                else:
                    self.get_logger().warn(f"Quad area is {area_ratio:.2%} of ROI. Suggestion: corner_bias:={suggested_bias} (sample {len(self.bias_suggestions)}/5)", throttle_duration_sec=2.0)

        quad = quad / scale
        cv2.drawContours(roi, [np.int0(quad)], 0, (255, 0, 0), 2)
        return self.order_corners(quad), roi

    def get_la_qwerty_layout(self):
        # Full 104-key ISO Spanish layout with all sections
        # Format: {"row": N, "offset": X, "keys": [(label, width), ...]}
        # Sections: Main (15u) + gap (0.25u) + Nav (3u) + gap (0.25u) + Numpad (4u) = ~22.5u total
        return [
            # F-row with gaps
            {"row": 0, "offset": 0, "keys": [
                ("Esc", 1), (None, 1), ("F1", 1), ("F2", 1), ("F3", 1), ("F4", 1), (None, 0.5),
                ("F5", 1), ("F6", 1), ("F7", 1), ("F8", 1), (None, 0.5),
                ("F9", 1), ("F10", 1), ("F11", 1), ("F12", 1), (None, 0.25),
                ("PrtSc", 1), ("ScrLk", 1), ("Pause", 1),
            ]},
            # Number row + nav top + numpad top
            {"row": 1.5, "offset": 0, "keys": [
                ("º", 1), ("1", 1), ("2", 1), ("3", 1), ("4", 1), ("5", 1),
                ("6", 1), ("7", 1), ("8", 1), ("9", 1), ("0", 1), ("'", 1), ("¿", 1), ("Backspace", 2),
                (None, 0.25), ("Ins", 1), ("Home", 1), ("PgUp", 1),
                (None, 0.25), ("NumLk", 1), ("/", 1), ("*", 1), ("-", 1),
            ]},
            # QWERTY row + nav mid + numpad row 1
            {"row": 2.5, "offset": 0, "keys": [
                ("Tab", 1.5), ("Q", 1), ("W", 1), ("E", 1), ("R", 1), ("T", 1),
                ("Y", 1), ("U", 1), ("I", 1), ("O", 1), ("P", 1), ("´", 1), ("+", 1),
                (None, 0.25), ("Del", 1), ("End", 1), ("PgDn", 1),
                (None, 0.25), ("Num7", 1), ("Num8", 1), ("Num9", 1),
            ]},
            # ASDF row (with ISO Enter spanning) + numpad row 2
            {"row": 3.5, "offset": 0, "keys": [
                ("Caps", 1.75), ("A", 1), ("S", 1), ("D", 1), ("F", 1), ("G", 1),
                ("H", 1), ("J", 1), ("K", 1), ("L", 1), ("Ñ", 1), ("{", 1), ("}", 1), ("Enter", 1.25),
                (None, 0.25), (None, 3),
                (None, 0.25), ("Num4", 1), ("Num5", 1), ("Num6", 1), ("+", 1),
            ]},
            # ZXCV row (ISO with extra key) + arrow up + numpad row 3
            {"row": 4.5, "offset": 0, "keys": [
                ("Shift", 1.25), ("<", 1), ("Z", 1), ("X", 1), ("C", 1), ("V", 1),
                ("B", 1), ("N", 1), ("M", 1), (",", 1), (".", 1), ("-", 1), ("ShiftR", 2.75),
                (None, 1.5), ("Up", 1), (None, 1),
                (None, 0.25), ("Num1", 1), ("Num2", 1), ("Num3", 1),
            ]},
            # Bottom row + arrow keys + numpad bottom
            {"row": 5.5, "offset": 0, "keys": [
                ("Ctrl", 1.25), ("Win", 1.25), ("Alt", 1.25), ("Space", 6.25),
                ("AltGr", 1.25), ("Fn", 1.25), ("Menu", 1.25), ("CtrlR", 1.25),
                (None, 0.25), ("Left", 1), ("Down", 1), ("Right", 1),
                (None, 0.25), ("Num0", 2), ("NumDot", 1), ("NumEnter", 1),
            ]},
        ]

    def map_keys_on_warp(self, warped, keyboard_quad):
        if keyboard_quad is None:
            return {}, warped

        quad = self.order_corners(keyboard_quad.astype("float32"))
        w_top = np.linalg.norm(quad[1] - quad[0])
        w_bottom = np.linalg.norm(quad[2] - quad[3])
        h_left = np.linalg.norm(quad[3] - quad[0])
        h_right = np.linalg.norm(quad[2] - quad[1])
        w_rect = int(max(w_top, w_bottom))
        h_rect = int(max(h_left, h_right))
        if w_rect < 10 or h_rect < 10:
            return {}, warped

        dst = np.array([[0, 0], [w_rect - 1, 0], [w_rect - 1, h_rect - 1], [0, h_rect - 1]], dtype="float32")
        M = cv2.getPerspectiveTransform(quad, dst)
        M_inv = cv2.getPerspectiveTransform(dst, quad)
        rectified = cv2.warpPerspective(warped, M, (w_rect, h_rect))

        layout = self.get_la_qwerty_layout()
        
        # Calculate physical dimensions with gaps
        key_unit_mm = self.key_size_mm + self.key_gap_mm
        row_count = len(layout)
        
        # Find max row width in units
        max_row_units = 0
        for row_def in layout:
            row_units = row_def["offset"] + sum(w for _, w in row_def["keys"])
            max_row_units = max(max_row_units, row_units)
        
        # Map units to pixels
        unit_w = w_rect / max_row_units
        unit_h = h_rect / row_count

        key_map = {}
        debug = rectified.copy()

        for row_def in layout:
            r = row_def["row"]
            offset = row_def["offset"]
            keys = row_def["keys"]
            
            y0 = int(r * unit_h)
            y1 = int((r + 1) * unit_h)
            
            x_cursor = offset * unit_w
            
            for label, width_u in keys:
                key_w = width_u * unit_w
                x0 = int(x_cursor)
                x1 = int(x_cursor + key_w)
                
                if label is not None:
                    # Add gap margin for keycap vs switch center
                    gap_px = int((self.key_gap_mm / key_unit_mm) * unit_w / 2)
                    cx = int((x0 + x1) / 2)
                    cy = int((y0 + y1) / 2)
                    key_map[label] = (cx, cy)

                    if self.draw_key_grid:
                        # Draw outer boundary
                        cv2.rectangle(debug, (x0, y0), (x1, y1), (100, 100, 100), 1)
                        # Draw keycap area (with gap)
                        cv2.rectangle(debug, (x0 + gap_px, y0 + gap_px), (x1 - gap_px, y1 - gap_px), (0, 255, 0), 1)
                        cv2.putText(debug, label, (x0 + 3, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0), 1)
                        cv2.circle(debug, (cx, cy), 2, (0, 0, 255), -1)

                x_cursor += key_w

        # Map key centers back to warped image coordinates
        key_map_warped = {}
        for label, (cx, cy) in key_map.items():
            pt = np.array([[[cx, cy]]], dtype="float32")
            warped_pt = cv2.perspectiveTransform(pt, M_inv)[0][0]
            key_map_warped[label] = (float(warped_pt[0]), float(warped_pt[1]))

        if self.draw_key_grid:
            self.show_debug("Keyboard Grid (Rectified)", debug)

        return key_map_warped, rectified

    def image_callback(self, msg):
        # ROS Image -> OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.debug_counter += 1

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.parameters
        )

        # Update state with any currently visible markers
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for marker_id, marker_corners in zip(ids.flatten(), corners):
                self.last_known_corners[marker_id] = marker_corners
                self.last_seen_timestamp[marker_id] = current_time
        
        # --- ID-Agnostic Logic: work if exactly 4 markers are seen recently ---
        
        # 1. Filter for markers that are "fresh"
        fresh_markers = {}
        for mid, corner_data in self.last_known_corners.items():
            if (current_time - self.last_seen_timestamp.get(mid, 0)) < self.STALE_THRESHOLD_SECONDS:
                fresh_markers[mid] = corner_data

        # 2. Proceed if we have 4 fresh markers to define the plane
        if len(fresh_markers) >= 4:
            self.get_logger().info(f"Found {len(fresh_markers)} fresh markers.", throttle_duration_sec=1.0)

            marker_quad = self.find_marker_quad(fresh_markers)
            if marker_quad is None:
                self.get_logger().warn("Failed to build marker quad.")
                self.show_debug("ArUco Detection", frame)
                cv2.waitKey(1)
                return

            cv2.polylines(frame, [np.int32(marker_quad)], isClosed=True, color=(0, 255, 255), thickness=2)

            w_dst = int(self.warp_width) if self.warp_width and self.warp_width > 0 else 900
            h_dst = int(self.warp_height) if self.warp_height and self.warp_height > 0 else 350

            pts_dst = np.array([[0, 0], [w_dst - 1, 0], [w_dst - 1, h_dst - 1], [0, h_dst - 1]], dtype="float32")
            M = cv2.getPerspectiveTransform(marker_quad, pts_dst)
            M_inv = cv2.getPerspectiveTransform(pts_dst, marker_quad)
            warped = cv2.warpPerspective(frame, M, (int(w_dst), int(h_dst)))

            keyboard_corners, roi_debug = self.detect_keyboard_quad(warped.copy())
            self.show_debug("Warped ROI", warped)
            self.show_debug("Keyboard Detection", roi_debug)

            if keyboard_corners is not None:
                warped_pts = keyboard_corners.reshape(-1, 1, 2).astype("float32")
                real_pts = cv2.perspectiveTransform(warped_pts, M_inv).reshape(-1, 2)

                if self.smooth_corners is None:
                    self.smooth_corners = real_pts
                else:
                    self.smooth_corners = (self.SMOOTH_ALPHA * real_pts) + ((1.0 - self.SMOOTH_ALPHA) * self.smooth_corners)

                smoothed_pts = self.smooth_corners
                cv2.polylines(frame, [np.int32(smoothed_pts)], isClosed=True, color=(255, 0, 255), thickness=3)

                cx, cy = np.int32(smoothed_pts[0])
                cv2.circle(frame, (cx, cy), 8, (255, 0, 255), -1)
                cv2.putText(frame, "Keyboard Origin", (cx + 10, cy + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

                # Build key map on warped view
                key_map, rectified = self.map_keys_on_warp(warped, keyboard_corners)
                self.key_map_warped = key_map
                if self.draw_key_grid:
                    self.show_debug("Keyboard Rectified", rectified)
        else:
            self.get_logger().info(f"Found {len(fresh_markers)} fresh markers, waiting for 4.", throttle_duration_sec=1.0)

        self.show_debug("ArUco Detection", frame)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
