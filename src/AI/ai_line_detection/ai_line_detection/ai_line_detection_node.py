#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
AI Line Detection Node for ROS2
Subscribes to CSI camera image, performs line segmentation using YOLOv8,
and overlays the result in yellow using OpenCV.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import time
from ultralytics import YOLO


class AILineDetectionNode(Node):
    def __init__(self):
        super().__init__('ai_line_detection_node')

        # Lane spacing learning (global to instance)
        self.learned_lane_spacing = None

        # Previous frame lane positions for temporal continuity
        self.prev_left_line_x = None
        self.prev_right_line_x = None
        self.search_window = 80  # Search within ±80 pixels of previous position

        # Previous XTE and lane center for fallback when detection fails
        self.prev_xte = 0.0
        self.prev_lane_center_x = None
        self.detection_failure_count = 0  # Track consecutive failures

        # Parameters
        self.declare_parameter('model_path', 'weights/best.pt')
        self.declare_parameter('conf_threshold', 0.3)
        self.declare_parameter('camera_topic', '/camera/ai_lane_detect')
        self.declare_parameter('overlay_alpha', 0.5)
        self.declare_parameter('show_window', True)  # Show OpenCV window

        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        camera_topic = self.get_parameter('camera_topic').value
        self.overlay_alpha = self.get_parameter('overlay_alpha').value
        self.show_window = self.get_parameter('show_window').value

        # Get absolute path for model
        if not os.path.isabs(model_path):
            # Try to get package share directory (for installed package)
            try:
                from ament_index_python.packages import get_package_share_directory
                package_share_dir = get_package_share_directory('ai_line_detection')
                model_path = os.path.join(package_share_dir, model_path)
            except:
                # Fallback to source directory
                package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
                model_path = os.path.join(package_dir, model_path)

        self.get_logger().info(f'Model path: {model_path}')
        self.get_logger().info(f'Confidence threshold: {self.conf_threshold}')
        self.get_logger().info(f'Camera topic: {camera_topic}')
        self.get_logger().info(f'Overlay alpha: {self.overlay_alpha}')

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Load YOLOv8 model
        self.model = None
        self.load_model(model_path)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )

        # Publishers
        self.overlay_pub = self.create_publisher(
            Image,
            '/ai_line_detection/overlay_image',
            10
        )

        self.mask_pub = self.create_publisher(
            Image,
            '/ai_line_detection/mask_image',
            10
        )

        self.xte_pub = self.create_publisher(
            Float32,
            '/xte/vision',
            10
        )

        # Publishers for mission control integration
        self.lane_status_pub = self.create_publisher(
            String,
            '/lane_status',
            10
        )

        self.stop_line_position_pub = self.create_publisher(
            Float32,
            '/stop_line_position',
            10
        )

        # Performance tracking
        self.frame_count = 0
        self.total_time = 0
        self.last_fps_time = time.time()
        self.fps = 0.0

        # OpenCV window setup
        if self.show_window:
            cv2.namedWindow('AI Line Detection - Overlay', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('AI Line Detection - Overlay', 1280, 200)
            # Force window to appear and process events with delay
            for _ in range(5):
                cv2.waitKey(10)  # Process GUI events
                time.sleep(0.01)  # Small delay
            # Try to bring window to front
            try:
                cv2.setWindowProperty('AI Line Detection - Overlay', cv2.WND_PROP_TOPMOST, 1)
                cv2.waitKey(10)
                time.sleep(0.05)
                cv2.setWindowProperty('AI Line Detection - Overlay', cv2.WND_PROP_TOPMOST, 0)
                cv2.waitKey(10)
            except:
                pass
            self.get_logger().info('OpenCV window enabled')

        self.get_logger().info('AI Line Detection Node initialized')

    def load_model(self, model_path):
        """Load YOLOv8 segmentation model"""
        try:
            if not os.path.exists(model_path):
                self.get_logger().error(f'Model file not found: {model_path}')
                return False

            self.get_logger().info(f'Loading YOLOv8 model: {model_path}')
            self.model = YOLO(model_path)
            self.get_logger().info('YOLOv8 model loaded successfully!')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            return False

    def create_masks(self, image, results):
        """
        Create binary masks for 3 classes:
        - Class 0: lane (차선)
        - Class 1: line (라인 마킹)
        - Class 2: stop_line (정지선)
        """
        lane_mask = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)
        line_mask = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)
        stop_line_mask = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)

        if results[0].masks is not None:
            masks = results[0].masks.data.cpu().numpy()
            classes = results[0].boxes.cls.cpu().numpy().astype(int)
            confidences = results[0].boxes.conf.cpu().numpy()

            for mask, cls, conf in zip(masks, classes, confidences):
                if conf >= self.conf_threshold:
                    # Resize mask to image size
                    mask_resized = cv2.resize(mask, (image.shape[1], image.shape[0]))
                    mask_binary = (mask_resized > 0.5).astype(np.uint8) * 255

                    if cls == 0:  # lane class (차선)
                        lane_mask = cv2.bitwise_or(lane_mask, mask_binary)
                    elif cls == 1:  # line class (라인 마킹)
                        line_mask = cv2.bitwise_or(line_mask, mask_binary)
                    elif cls == 2:  # stop_line class (정지선)
                        stop_line_mask = cv2.bitwise_or(stop_line_mask, mask_binary)

        return lane_mask, line_mask, stop_line_mask

    def calculate_lane_angle(self, mask, image_height):
        """Calculate lane angle from mask"""
        if mask is None or np.sum(mask) == 0:
            return 0

        y_center = image_height // 2
        y_top = image_height // 4

        # Get x positions at center and top
        center_row = mask[y_center, :]
        top_row = mask[y_top, :]

        center_x_positions = np.where(center_row > 0)[0]
        top_x_positions = np.where(top_row > 0)[0]

        if len(center_x_positions) == 0 or len(top_x_positions) == 0:
            return 0

        center_x = np.mean(center_x_positions)
        top_x = np.mean(top_x_positions)

        # Calculate angle
        dx = top_x - center_x
        dy = y_center - y_top
        angle = np.arctan2(dx, dy) * 180 / np.pi

        return angle

    def calculate_line_width(self, mask, image_height, lanes):
        """Calculate perpendicular width of individual line"""
        if mask is None or np.sum(mask) == 0:
            return 0

        if lanes is None or len(lanes) == 0:
            return 0

        # Get lane angle
        angle = self.calculate_lane_angle(mask, image_height)
        angle_rad = angle * np.pi / 180

        # Calculate width of each individual line
        individual_widths = []
        for lane_left, lane_right in lanes:
            horizontal_width = lane_right - lane_left + 1
            perpendicular_width = horizontal_width * np.cos(angle_rad)
            individual_widths.append(perpendicular_width)

        if len(individual_widths) == 0:
            return 0

        # Return average width of individual lines
        return int(np.mean(individual_widths))

    def detect_lane_edges(self, mask, image_height, image_width):
        """Detect left and right edges of lane(s) at y-axis center with temporal continuity"""
        if mask is None or np.sum(mask) == 0:
            return None

        y_center = image_height // 2
        row = mask[y_center, :]

        # Find all continuous segments
        x_positions = np.where(row > 0)[0]

        if len(x_positions) == 0:
            # Try nearby rows
            for offset in range(1, image_height // 2):
                if y_center - offset >= 0:
                    row = mask[y_center - offset, :]
                    x_positions = np.where(row > 0)[0]
                    if len(x_positions) > 0:
                        break
                if y_center + offset < image_height:
                    row = mask[y_center + offset, :]
                    x_positions = np.where(row > 0)[0]
                    if len(x_positions) > 0:
                        break

        if len(x_positions) == 0:
            return None

        # Find gaps to detect multiple lanes
        gaps = []
        for i in range(len(x_positions) - 1):
            if x_positions[i + 1] - x_positions[i] > 10:  # Gap threshold
                gaps.append(i)

        lanes = []
        if len(gaps) == 0:
            # Single lane detected
            lanes.append((x_positions[0], x_positions[-1]))
        else:
            # Multiple lanes detected
            start = 0
            for gap_idx in gaps:
                lanes.append((x_positions[start], x_positions[gap_idx]))
                start = gap_idx + 1
            lanes.append((x_positions[start], x_positions[-1]))

        # Merge lanes that are too close together (likely same wide line with a gap)
        # Keep merging until all remaining lanes are far enough apart
        while len(lanes) >= 2:
            # Find the closest pair
            min_distance = float('inf')
            min_idx = -1

            for i in range(len(lanes) - 1):
                center1 = (lanes[i][0] + lanes[i][1]) // 2
                center2 = (lanes[i + 1][0] + lanes[i + 1][1]) // 2
                distance = abs(center2 - center1)

                if distance < min_distance:
                    min_distance = distance
                    min_idx = i

            # If the closest pair is less than 300px apart, merge them
            if min_distance < 300:
                # When merging, select the lane closer to image center
                image_center = image_width // 2
                lane1_center = (lanes[min_idx][0] + lanes[min_idx][1]) // 2
                lane2_center = (lanes[min_idx + 1][0] + lanes[min_idx + 1][1]) // 2

                dist1_to_center = abs(lane1_center - image_center)
                dist2_to_center = abs(lane2_center - image_center)

                # Select the lane closer to center
                if dist1_to_center <= dist2_to_center:
                    selected_lane = lanes[min_idx]
                else:
                    selected_lane = lanes[min_idx + 1]

                # Replace both lanes with the selected one
                new_lanes = lanes[:min_idx] + [selected_lane] + lanes[min_idx + 2:]
                lanes = new_lanes

                self.get_logger().info(
                    f'Merged close lanes at idx {min_idx}, distance={min_distance}px, selected lane center={((selected_lane[0] + selected_lane[1]) // 2)}px (closer to center)',
                    throttle_duration_sec=1.0
                )
            else:
                # All remaining lanes are far enough apart
                break

        # If more than 2 lanes detected, select the best pair
        if len(lanes) > 2:
            lane_centers = [(l[0] + l[1]) // 2 for l in lanes]

            # Expected lane spacing
            expected_spacing = 480

            # Find the pair of lanes with spacing closest to expected
            best_pair = None
            best_spacing_diff = float('inf')

            for i in range(len(lanes)):
                for j in range(i + 1, len(lanes)):
                    spacing = abs(lane_centers[j] - lane_centers[i])
                    spacing_diff = abs(spacing - expected_spacing)

                    # Valid spacing range: 200-600px
                    if 200 <= spacing <= 600 and spacing_diff < best_spacing_diff:
                        best_spacing_diff = spacing_diff
                        best_pair = (i, j)

            # If found a good pair, keep only those lanes
            if best_pair is not None:
                i, j = best_pair
                filtered_lanes = [lanes[i], lanes[j]]
                # Sort by x position (left to right)
                filtered_lanes.sort(key=lambda l: (l[0] + l[1]) // 2)
                lanes = filtered_lanes

                self.get_logger().info(
                    f'Selected best pair from {len(lane_centers)} lanes: spacing={abs(lane_centers[j] - lane_centers[i])}px',
                    throttle_duration_sec=1.0
                )

        return lanes

    def calculate_lane_center_error(self, mask, image_width):
        """Calculate cross-track error from lane mask"""
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) == 0:
            return 0.0

        # Get the largest contour (assumed to be the lane)
        largest_contour = max(contours, key=cv2.contourArea)

        # Calculate moments to find centroid
        M = cv2.moments(largest_contour)
        if M['m00'] == 0:
            return 0.0

        # Calculate centroid x-coordinate
        cx = int(M['m10'] / M['m00'])

        # Calculate error from image center
        image_center = image_width / 2
        error = cx - image_center

        return error

    def overlay_masks_on_image(self, image, lane_mask, line_mask, stop_line_mask, alpha=0.5):
        """
        Overlay lane, line, and stop_line masks with lane center calculation
        - Lane (class 0): pink (203, 192, 255) in BGR
        - Line (class 1): green (0, 255, 0) in BGR
        - Stop line (class 2): blue (255, 0, 0) in BGR
        """
        # Create colored overlay
        overlay = image.copy()
        overlay[lane_mask > 0] = (203, 192, 255)  # Pink for lane (차선)
        overlay[line_mask > 0] = (0, 255, 0)      # Green for line (라인 마킹)
        overlay[stop_line_mask > 0] = (255, 0, 0)  # Blue for stop_line (정지선)

        # Blend with original image
        result = cv2.addWeighted(image, 1 - alpha, overlay, alpha, 0)

        h, w = image.shape[:2]
        y_center = h // 2

        # Draw image center line (vertical)
        cv2.line(result, (w // 2, 0), (w // 2, h), (128, 128, 128), 1, cv2.LINE_AA)

        # Draw horizontal center line
        cv2.line(result, (0, y_center), (w, y_center), (128, 128, 128), 1, cv2.LINE_AA)

        # Detect individual lines (lane markings) - use line_mask (class 1)
        lines = self.detect_lane_edges(line_mask, h, w)
        line_width = self.calculate_line_width(line_mask, h, lines)

        left_line_x = None
        right_line_x = None
        is_virtual = False

        if lines is not None and len(lines) > 0:
            # Debug: log detected lines
            self.get_logger().info(
                f'Detected {len(lines)} line(s): {[(l[0], l[1], (l[0]+l[1])//2) for l in lines]}',
                throttle_duration_sec=1.0
            )

            if len(lines) == 1:
                # Single line detected - offset by half of lane spacing (480px / 2 = 240px)
                line_left, line_right = lines[0]
                detected_line_center = (line_left + line_right) // 2

                # Fixed lane spacing
                lane_spacing = 480
                half_spacing = 240

                # Determine if detected line is on left or right side using temporal continuity
                is_left_line = False
                is_right_line = False

                # If we have previous positions, use them to determine which line this is
                if self.prev_left_line_x is not None and self.prev_right_line_x is not None:
                    dist_to_left = abs(detected_line_center - self.prev_left_line_x)
                    dist_to_right = abs(detected_line_center - self.prev_right_line_x)

                    # Choose based on which previous position is closer
                    if dist_to_left < dist_to_right and dist_to_left < self.search_window:
                        is_left_line = True
                    elif dist_to_right < self.search_window:
                        is_right_line = True
                    else:
                        # Too far from both - use image center as fallback
                        is_left_line = detected_line_center < w // 2
                        is_right_line = not is_left_line
                else:
                    # No previous position - use image center
                    is_left_line = detected_line_center < w // 2
                    is_right_line = not is_left_line

                if is_left_line:
                    # Left line detected - lane center is 200px to the right
                    left_line_x = detected_line_center
                    right_line_x = detected_line_center + lane_spacing
                    is_virtual = True
                else:  # is_right_line
                    # Right line detected - lane center is 200px to the left
                    right_line_x = detected_line_center
                    left_line_x = detected_line_center - lane_spacing
                    is_virtual = True

            elif len(lines) >= 2:
                # Two or more lines detected
                line1_center = (lines[0][0] + lines[0][1]) // 2
                line2_center = (lines[1][0] + lines[1][1]) // 2

                # Check distance between two lines
                distance = abs(line2_center - line1_center)

                # Debug: log line distance
                self.get_logger().info(
                    f'Line1 center: {line1_center}px, Line2 center: {line2_center}px, Distance: {distance}px',
                    throttle_duration_sec=1.0
                )

                # Valid lane spacing: 200-600px
                min_distance = 200
                max_distance = 600

                if min_distance <= distance <= max_distance:
                    # Valid lane boundaries - learn the spacing
                    left_line_x = min(line1_center, line2_center)
                    right_line_x = max(line1_center, line2_center)
                    self.learned_lane_spacing = distance

                    self.get_logger().info(
                        f'Valid spacing detected: {distance}px, Left: {left_line_x}px, Right: {right_line_x}px',
                        throttle_duration_sec=1.0
                    )
                else:
                    # Distance not valid - use first line and create virtual
                    detected_line_center = line1_center
                    if line_width > 0:
                        lane_spacing = int(line_width * 20)
                    else:
                        lane_spacing = 340

                    if detected_line_center < w // 2:
                        left_line_x = detected_line_center
                        right_line_x = detected_line_center + lane_spacing
                        is_virtual = True
                    else:
                        right_line_x = detected_line_center
                        left_line_x = detected_line_center - lane_spacing
                        is_virtual = True

        # Draw lane center (between left and right lines)
        lane_center_x = None
        if left_line_x is not None and right_line_x is not None:
            # Update previous frame positions for temporal continuity
            self.prev_left_line_x = left_line_x
            self.prev_right_line_x = right_line_x

            # Calculate lane center
            lane_center_x = (left_line_x + right_line_x) // 2

            # Draw lane center line (bright yellow - main indicator)
            cv2.line(result, (lane_center_x, 0), (lane_center_x, h), (0, 255, 255), 3, cv2.LINE_AA)
            cv2.circle(result, (lane_center_x, y_center), 8, (0, 255, 255), -1)

            # Draw left and right line positions (thin, less prominent)
            left_color = (100, 100, 150) if is_virtual else (203, 192, 255)
            right_color = (100, 100, 150) if is_virtual else (203, 192, 255)

            cv2.line(result, (left_line_x, 0), (left_line_x, h), left_color, 1, cv2.LINE_AA)
            cv2.line(result, (right_line_x, 0), (right_line_x, h), right_color, 1, cv2.LINE_AA)

        return result, lane_center_x

    def image_callback(self, msg):
        """Process incoming camera images"""
        if self.model is None:
            self.get_logger().warn('Model not loaded, skipping frame')
            return

        try:
            # Convert ROS Image to OpenCV format
            start_time = time.time()
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Run YOLOv8 inference
            results = self.model(cv_image, verbose=False)

            # Create lane, line, and stop_line masks (3 classes)
            lane_mask, line_mask, stop_line_mask = self.create_masks(cv_image, results)

            # Store results for lane status detection
            self.latest_results = results

            # Overlay masks on original image and get lane center
            overlay_image, lane_center_x = self.overlay_masks_on_image(
                cv_image,
                lane_mask,
                line_mask,
                stop_line_mask,
                alpha=self.overlay_alpha
            )

            # Calculate cross-track error from lane center
            if lane_center_x is not None:
                # Detection successful - calculate new XTE
                image_center = cv_image.shape[1] / 2
                xte = lane_center_x - image_center

                # Save current values for fallback
                self.prev_xte = xte
                self.prev_lane_center_x = lane_center_x
                self.detection_failure_count = 0
            else:
                # Detection failed - use previous values to maintain steering
                xte = self.prev_xte
                lane_center_x = self.prev_lane_center_x
                self.detection_failure_count += 1

                self.get_logger().warn(
                    f'Line detection failed (count: {self.detection_failure_count}), '
                    f'maintaining previous XTE: {xte:.1f} px',
                    throttle_duration_sec=0.5
                )

            # Add FPS and XTE info to overlay image
            current_time = time.time()
            if current_time - self.last_fps_time >= 1.0:
                self.fps = self.frame_count / (current_time - self.last_fps_time)
                self.frame_count = 0
                self.last_fps_time = current_time

            self.frame_count += 1

            # Draw info text
            cv2.putText(overlay_image, f'FPS: {self.fps:.1f}',
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(overlay_image, f'XTE: {xte:.1f} px',
                       (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Publish overlay image
            overlay_msg = self.bridge.cv2_to_imgmsg(overlay_image, encoding='bgr8')
            overlay_msg.header = msg.header
            self.overlay_pub.publish(overlay_msg)

            # Publish mask image (for debugging) - use line_mask (class 1)
            mask_colored = cv2.cvtColor(line_mask, cv2.COLOR_GRAY2BGR)
            mask_msg = self.bridge.cv2_to_imgmsg(mask_colored, encoding='bgr8')
            mask_msg.header = msg.header
            self.mask_pub.publish(mask_msg)

            # Publish cross-track error
            xte_msg = Float32()
            xte_msg.data = float(xte)
            self.xte_pub.publish(xte_msg)

            # Publish stop line position if detected (class 2)
            if np.sum(stop_line_mask) > 0:
                # Find the bottom-most Y coordinate of stop line
                y_coords, _ = np.where(stop_line_mask > 0)
                stop_line_y = float(np.max(y_coords))

                stop_line_msg = Float32()
                stop_line_msg.data = stop_line_y
                self.stop_line_position_pub.publish(stop_line_msg)

                self.get_logger().info(
                    f'Stop line detected at y={stop_line_y:.1f}px',
                    throttle_duration_sec=2.0
                )

            # Publish lane status (for mission control)
            # Detect line presence on LEFT and RIGHT sides based on YOLO confidence
            lane_status_msg = String()

            # Determine line detection status based on segmentation confidence
            left_detected = "CLEAR"
            right_detected = "CLEAR"

            if results[0].masks is not None and results[0].boxes is not None:
                masks = results[0].masks.data.cpu().numpy()
                classes = results[0].boxes.cls.cpu().numpy().astype(int)
                confidences = results[0].boxes.conf.cpu().numpy()
                boxes = results[0].boxes.xyxy.cpu().numpy()  # Get bounding boxes

                image_width = cv_image.shape[1]
                image_center = image_width // 2

                # Check each detection for line class (class 1)
                for mask, cls, conf, box in zip(masks, classes, confidences, boxes):
                    if cls == 1 and conf >= self.conf_threshold:  # Line class with sufficient confidence
                        # Get center x of bounding box
                        x1, y1, x2, y2 = box
                        box_center_x = (x1 + x2) / 2

                        # Determine if line is on left or right side
                        if box_center_x < image_center:
                            left_detected = "DETECTED"
                        else:
                            right_detected = "DETECTED"

            # Format: "LEFT,RIGHT"
            lane_status_msg.data = f"{left_detected},{right_detected}"
            self.lane_status_pub.publish(lane_status_msg)

            # Calculate processing time
            processing_time = (time.time() - start_time) * 1000
            self.get_logger().info(
                f'Processing time: {processing_time:.1f}ms, XTE: {xte:.1f}px',
                throttle_duration_sec=1.0
            )

            # Show OpenCV window if enabled
            if self.show_window:
                cv2.imshow('AI Line Detection - Overlay', overlay_image)
                cv2.waitKey(1)  # Process window events

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = AILineDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Exception during spin: {e}')
    finally:
        # Close OpenCV windows
        try:
            cv2.destroyAllWindows()
        except:
            pass
        try:
            node.destroy_node()
        except:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
