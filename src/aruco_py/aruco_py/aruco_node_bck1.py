


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

    def detect_keyboard_contour(self, roi):
        # The ROI is now a clean, top-down view.
        h_roi, w_roi, _ = roi.shape
        roi_area = h_roi * w_roi
        
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        # No blur needed, the warped image is cleaner
        # Adjusted Canny thresholds for high-contrast warped image
        edges = cv2.Canny(gray, 50, 150)

        contours, _ = cv2.findContours(
            edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            return None, roi

        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        for contour in contours:
            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.02 * peri, True)

            if len(approx) == 4:
                area = cv2.contourArea(contour)
                
                # --- Critical Check: Ensure contour is a large part of the ROI ---
                # This prevents mistaking a single key for the whole keyboard.
                if area < (roi_area * 0.3):
                    continue
                # ----------------------------------------------------------------

                (x, y, w, h) = cv2.boundingRect(approx)
                
                if h == 0:
                    continue
                
                aspect_ratio = float(w) / h

                # Aspect ratio for a rectified keyboard
                if 1.5 < aspect_ratio < 4.0:
                    cv2.drawContours(roi, [approx], -1, (0, 255, 0), 3) # Thicker line
                    
                    corner = (x, y)

                    cv2.circle(roi, corner, 8, (0, 0, 255), -1)
                    cv2.putText(
                        roi, "Keyboard Origin",
                        (corner[0] + 15, corner[1] + 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 255), 2
                    )

                    return corner, roi

        return None, roi


    def image_callback(self, msg):
        # ROS Image -> OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        current_time = self.get_clock().now().seconds_nanoseconds()[0]

        gray = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                     cv2.THRESH_BINARY, 11, 2)

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

        # 2. Proceed if we have exactly 4 fresh markers
        if len(fresh_markers) == 4:
            
            self.get_logger().info(f"Found 4 fresh markers.", throttle_duration_sec=1.0)
            # 3. Spatially sort the four markers regardless of their IDs
            # Calculate centers for sorting
            centers = {mid: np.mean(c[0], axis=0) for mid, c in fresh_markers.items()}

            # Sort centers by y-coordinate to get top and bottom rows
            sorted_by_y = sorted(centers.items(), key=lambda item: item[1][1])
            
            top_row = sorted(sorted_by_y[:2], key=lambda item: item[1][0])
            bottom_row = sorted(sorted_by_y[2:], key=lambda item: item[1][0])

            tl_id, _ = top_row[0]
            tr_id, _ = top_row[1]
            bl_id, _ = bottom_row[0]
            br_id, _ = bottom_row[1]

            # 4. Build the source points from the spatially sorted corners
            pts_src = np.array([
                fresh_markers[tl_id][0][0], # Top-left corner of TL marker
                fresh_markers[tr_id][0][1], # Top-right corner of TR marker
                fresh_markers[br_id][0][2], # Bottom-right corner of BR marker
                fresh_markers[bl_id][0][3]  # Bottom-left corner of BL marker
            ], dtype="float32")

            # 5. Define destination and perform warp
            w_dst, h_dst = 800, 400
            pts_dst = np.array([[0, 0], [w_dst - 1, 0], [w_dst - 1, h_dst - 1], [0, h_dst - 1]], dtype="float32")
            
            M = cv2.getPerspectiveTransform(pts_src, pts_dst)
            M_inv = cv2.getPerspectiveTransform(pts_dst, pts_src)
            warped_roi = cv2.warpPerspective(frame, M, (w_dst, h_dst))

            # 6. Detect keyboard and map it back
            corner, roi_debug = self.detect_keyboard_contour(warped_roi.copy())
            cv2.imshow("Warped ROI", warped_roi)
            cv2.imshow("Keyboard Detection Debug", roi_debug)
            
            if corner is not None:
                warped_corner = np.array([[[corner[0], corner[1]]]], dtype="float32")
                real_corner = cv2.perspectiveTransform(warped_corner, M_inv)[0][0]
                cx, cy = int(real_corner[0]), int(real_corner[1])

                cv2.circle(frame, (cx, cy), 8, (0, 0, 255), -1)
                cv2.putText(frame, "Keyboard Origin", (cx + 10, cy + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        else:
            self.get_logger().info(f"Found {len(fresh_markers)} fresh markers baby, waiting for exactly 4.", throttle_duration_sec=1.0)

        cv2.imshow("ArUco Detection", frame)
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
