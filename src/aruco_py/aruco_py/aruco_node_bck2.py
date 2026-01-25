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
        # Adjusted Canny thresholds for high-contrast warped image
        edges = cv2.Canny(gray, 50, 150)
        cv2.imshow("Canny Edges", edges) # DEBUG WINDOW

        contours, _ = cv2.findContours(
            edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            self.get_logger().warn("Canny found 0 contours.")
            return None, roi

        self.get_logger().info(f"Found {len(contours)} raw contours. Analyzing the largest ones.")
        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        for i, contour in enumerate(contours[:5]): # Check top 5 largest
            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.02 * peri, True)

            area = cv2.contourArea(contour)
            self.get_logger().info(f"Contour #{i}: Area={area:.0f}, Vertices={len(approx)}")

            if len(approx) == 4:
                self.get_logger().info(f"  -> Is a quadrilateral.")
                # --- Critical Check: Ensure contour is a large part of the ROI ---
                if area < (roi_area * 0.3):
                    self.get_logger().info(f"  -> REJECTED: Area is too small ({area/roi_area:.2f} < 0.3).")
                    continue
                # ----------------------------------------------------------------

                (x, y, w, h) = cv2.boundingRect(approx)
                
                if h == 0:
                    continue
                
                aspect_ratio = float(w) / h
                self.get_logger().info(f"  -> Aspect Ratio: {aspect_ratio:.2f}")

                # Aspect ratio for a rectified keyboard
                if 1.5 < aspect_ratio < 4.0:
                    self.get_logger().info(f"  -> ACCEPTED as keyboard contour.")
                    cv2.drawContours(roi, [approx], -1, (0, 255, 0), 3) # Thicker line
                    
                    bbox = (x, y, w, h)

                    cv2.circle(roi, (x, y), 8, (0, 0, 255), -1)
                    cv2.putText(
                        roi, "Keyboard Origin",
                        (x + 15, y + 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 255), 2
                    )

                    return bbox, roi
                else:
                    self.get_logger().info(f"  -> REJECTED: Aspect ratio out of range [1.5, 4.0].")

        self.get_logger().warn("No suitable keyboard contour found after checking top 5.")
        return None, roi


    def find_spacebar(self, keyboard_roi):
        h_roi, w_roi, _ = keyboard_roi.shape
        if h_roi == 0 or w_roi == 0:
            return None, None

        gray = cv2.cvtColor(keyboard_roi, cv2.COLOR_BGR2GRAY)
        
        # Otsu's Binarization is great for separating keys from the board
        _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        
        # A bit of morphology to clean up noise. Let's try slightly larger kernels.
        kernel = np.ones((3, 3), np.uint8)
        morphed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        # Opening can sometimes remove whole keys, let's be careful
        # morphed = cv2.morphologyEx(morphed, cv2.MORPH_OPEN, kernel)

        cv2.imshow("Spacebar Thresh", morphed) # Debug window

        contours, _ = cv2.findContours(
            morphed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            return None, keyboard_roi

        # --- Find the spacebar ---
        # It's usually the widest contour with a certain aspect ratio.
        self.get_logger().info("--- Analyzing Contours for Spacebar ---")
        possible_spacebars = []
        for i, contour in enumerate(contours):
            (x, y, w, h) = cv2.boundingRect(contour)
            
            if h == 0: continue

            aspect_ratio = float(w) / h
            
            # Log properties of every contour
            self.get_logger().info(
                f"Contour #{i}: "
                f"Aspect Ratio={aspect_ratio:.2f} | "
                f"Rel Width={w/w_roi:.2f} | "
                f"Rel Height={h/h_roi:.2f}"
            )

            # Heuristics for a spacebar (relaxed):
            # - It should be very wide (high aspect ratio)
            # - It shouldn't be too tall compared to the keyboard height
            # - It should have a significant width compared to the keyboard width
            if aspect_ratio > 3.5 and h < (h_roi * 0.5) and w > (w_roi * 0.25):
                self.get_logger().info(f"  -> Contour #{i} is a candidate.")
                possible_spacebars.append(contour)
        
        if not possible_spacebars:
            self.get_logger().warn("No spacebar candidates found after filtering.")
            return None, keyboard_roi
            
        # The spacebar should be the largest of the candidates
        spacebar_contour = max(possible_spacebars, key=cv2.contourArea)

        # Draw for debugging
        cv2.drawContours(keyboard_roi, [spacebar_contour], -1, (255, 0, 0), 2)

        return spacebar_contour, keyboard_roi


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

            # 6. Detect keyboard contour
            bbox, roi_debug = self.detect_keyboard_contour(warped_roi.copy())
            
            # Show the debug view of the keyboard contour detection
            cv2.imshow("Keyboard Contour Debug", roi_debug)

            if bbox is not None:
                x, y, w, h = bbox
                # Crop the keyboard from the warped image
                keyboard_roi = warped_roi[y:y+h, x:x+w]
                
                if keyboard_roi.size > 0:
                    cv2.imshow("Cropped Keyboard", keyboard_roi)

                    # 7. Find the spacebar within the keyboard ROI
                    spacebar_contour, kbd_debug = self.find_spacebar(keyboard_roi.copy())
                    
                    if spacebar_contour is not None:
                        cv2.imshow("Keyboard with Spacebar", kbd_debug)

                # Map corner back to the original frame (example)
                warped_corner = np.array([[[x, y]]], dtype="float32")
                real_corner = cv2.perspectiveTransform(warped_corner, M_inv)[0][0]
                cx, cy = int(real_corner[0]), int(real_corner[1])

                cv2.circle(frame, (cx, cy), 8, (0, 0, 255), -1)
                cv2.putText(frame, "Keyboard Origin", (cx + 10, cy + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        else:
            self.get_logger().info(f"Found {len(fresh_markers)} fresh markers, waiting for exactly 4.", throttle_duration_sec=1.0)

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
