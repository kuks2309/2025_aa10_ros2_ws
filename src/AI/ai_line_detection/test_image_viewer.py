#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Simple Image Viewer with Line Overlay
Just displays images with green line overlay - no file saving
"""

import cv2
import numpy as np
import os
import sys
import glob
import torch
from ultralytics import YOLO

# Set environment variables to reduce GPU memory usage
os.environ['CUDA_MODULE_LOADING'] = 'LAZY'

# Clear GPU cache
if torch.cuda.is_available():
    torch.cuda.empty_cache()

# Global variable to store learned lane spacing
learned_lane_spacing = None


def create_masks(model, image, conf_threshold=0.3):
    """Create binary masks for lane (class 0), line (class 1), and stop_line (class 2)"""
    # Keep original shape for aspect ratio (640x100)
    original_shape = image.shape[:2]

    # Use original image size for inference (already small at 640x100)
    # This is much smaller than 640x640, so no memory issue
    results = model(image, verbose=False, imgsz=640)

    # Create masks at original size - 3 classes
    lane_mask = np.zeros(original_shape, dtype=np.uint8)
    line_mask = np.zeros(original_shape, dtype=np.uint8)
    stop_line_mask = np.zeros(original_shape, dtype=np.uint8)

    if results[0].masks is not None:
        masks = results[0].masks.data.cpu().numpy()
        classes = results[0].boxes.cls.cpu().numpy().astype(int)
        confidences = results[0].boxes.conf.cpu().numpy()

        for mask, cls, conf in zip(masks, classes, confidences):
            if conf >= conf_threshold:
                # Resize mask to original image size
                mask_resized = cv2.resize(mask, (original_shape[1], original_shape[0]))
                mask_binary = (mask_resized > 0.5).astype(np.uint8) * 255

                if cls == 0:  # lane class (차선)
                    lane_mask = cv2.bitwise_or(lane_mask, mask_binary)
                elif cls == 1:  # line class (라인 마킹)
                    line_mask = cv2.bitwise_or(line_mask, mask_binary)
                elif cls == 2:  # stop_line class (정지선)
                    stop_line_mask = cv2.bitwise_or(stop_line_mask, mask_binary)

    # Clear GPU cache after inference
    if torch.cuda.is_available():
        torch.cuda.empty_cache()

    return lane_mask, line_mask, stop_line_mask


def calculate_lane_angle(mask, image_height):
    """Calculate lane angle (tilt) from vertical"""
    if mask is None or np.sum(mask) == 0:
        return 0

    # Find lane positions at different y coordinates
    y_positions = []
    x_positions = []

    y_center = image_height // 2

    # Sample from bottom to top
    for y_offset in range(-20, 21, 5):
        y = y_center + y_offset
        if 0 <= y < image_height:
            row = mask[y, :]
            x_pos = np.where(row > 0)[0]
            if len(x_pos) > 0:
                x_center = int(np.mean(x_pos))
                y_positions.append(y)
                x_positions.append(x_center)

    if len(y_positions) < 2:
        return 0

    # Calculate angle using linear regression
    y_arr = np.array(y_positions)
    x_arr = np.array(x_positions)

    # Calculate slope
    if len(y_arr) > 1:
        slope = np.polyfit(y_arr, x_arr, 1)[0]
        # Convert slope to angle in degrees
        angle = np.arctan(slope) * 180 / np.pi
        return angle

    return 0


def calculate_line_width(mask, image_height):
    """Calculate perpendicular width of individual line (not total span)"""
    if mask is None or np.sum(mask) == 0:
        return 0

    y_center = image_height // 2

    # Get lane angle
    angle = calculate_lane_angle(mask, image_height)
    angle_rad = angle * np.pi / 180

    # Detect individual lines first
    lanes = detect_lane_edges(mask, image_height)
    if lanes is None or len(lanes) == 0:
        return 0

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


def detect_lane_edges(mask, image_height):
    """Detect left and right edges of lane(s) at y-axis center"""
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
        if x_positions[i + 1] - x_positions[i] > 10:  # Gap threshold increased to 10px
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

        # If the closest pair is less than 150px apart, merge them
        # (typical lane spacing is 330-350px, so 150px means they're too close)
        if min_distance < 150:
            # Merge lanes[min_idx] and lanes[min_idx + 1]
            merged_lane = (lanes[min_idx][0], lanes[min_idx + 1][1])
            new_lanes = lanes[:min_idx] + [merged_lane] + lanes[min_idx + 2:]
            lanes = new_lanes
        else:
            # All remaining lanes are far enough apart
            break

    return lanes


def calculate_line_center_x(mask, image_height):
    """Calculate x position of line center at y-axis center (middle of image)"""
    if mask is None or np.sum(mask) == 0:
        return None

    lanes = detect_lane_edges(mask, image_height)
    if lanes is None or len(lanes) == 0:
        return None

    # Return center of first detected lane
    x_center = int((lanes[0][0] + lanes[0][1]) / 2)
    return x_center


def overlay_lines(image, lane_mask, line_mask, stop_line_mask, alpha=0.5):
    """Overlay colors: lane(pink), line(green), stop_line(blue) with center markers"""
    global learned_lane_spacing

    overlay = image.copy()
    overlay[lane_mask > 0] = (203, 192, 255)  # Pink for lane (차선) (BGR)
    overlay[line_mask > 0] = (0, 255, 0)  # Green for line (라인 마킹) (BGR)
    overlay[stop_line_mask > 0] = (255, 0, 0)  # Blue for stop_line (정지선) (BGR)
    result = cv2.addWeighted(image, 1 - alpha, overlay, alpha, 0)

    h, w = image.shape[:2]
    y_center = h // 2

    # Draw image center line (vertical)
    cv2.line(result, (w // 2, 0), (w // 2, h), (128, 128, 128), 1, cv2.LINE_AA)

    # Draw horizontal center line
    cv2.line(result, (0, y_center), (w, y_center), (128, 128, 128), 1, cv2.LINE_AA)

    # Detect individual lines (lane markings) - use line_mask (class 1)
    lines = detect_lane_edges(line_mask, h)
    line_width = calculate_line_width(line_mask, h)

    left_line_x = None
    right_line_x = None
    is_virtual = False

    # Get lane angle for information display
    lane_angle = calculate_lane_angle(line_mask, h)

    if lines is not None and len(lines) > 0:
        print(f"[DEBUG] Detected {len(lines)} line(s): {lines}")
        print(f"[DEBUG] Line width: {line_width}px")

        if len(lines) == 1:
            # Single line detected - create virtual second line
            line_left, line_right = lines[0]
            detected_line_center = (line_left + line_right) // 2
            print(f"[DEBUG] Single line: edges=({line_left}, {line_right}), center={detected_line_center}")

            # Use learned lane spacing if available, otherwise estimate
            if learned_lane_spacing is not None:
                lane_spacing = learned_lane_spacing
                print(f"[DEBUG] Using learned lane spacing: {lane_spacing}px")
            elif line_width > 0:
                lane_spacing = int(line_width * 20)  # 20x line width = ~340px
                print(f"[DEBUG] Using estimated lane spacing: {lane_spacing}px (line_width * 20)")
            else:
                lane_spacing = 340  # Default spacing in pixels (typical measured value)
                print(f"[DEBUG] Using default lane spacing: {lane_spacing}px")

            # Determine if detected line is on left or right side
            if detected_line_center < w // 2:
                # Line on left side - create virtual right line
                left_line_x = detected_line_center
                right_line_x = detected_line_center + lane_spacing
                is_virtual = True
                print(f"[DEBUG] Left line detected, creating virtual right: L={left_line_x}, R={right_line_x} [Virtual]")
            else:
                # Line on right side - create virtual left line
                right_line_x = detected_line_center
                left_line_x = detected_line_center - lane_spacing
                is_virtual = True
                print(f"[DEBUG] Right line detected, creating virtual left: L={left_line_x} [Virtual], R={right_line_x}")

        elif len(lines) >= 2:
            # Two or more lines detected
            # Calculate center of each line
            line1_center = (lines[0][0] + lines[0][1]) // 2
            line2_center = (lines[1][0] + lines[1][1]) // 2
            print(f"[DEBUG] Line 1: edges=({lines[0][0]}, {lines[0][1]}), center={line1_center}")
            print(f"[DEBUG] Line 2: edges=({lines[1][0]}, {lines[1][1]}), center={line2_center}")

            # Check distance between two lines
            distance = abs(line2_center - line1_center)
            print(f"[DEBUG] Distance between line centers: {distance}px")

            # Valid lane spacing: based on typical measurements
            # Measured lane spacing: 330-350px typically
            # Use absolute values with some tolerance
            min_distance = 200   # Minimum lane spacing (200px)
            max_distance = 550   # Maximum lane spacing (550px)

            print(f"[DEBUG] Valid distance range: {min_distance}~{max_distance}px (line_width={line_width}px)")

            if min_distance <= distance <= max_distance:
                # Valid lane boundaries - learn the spacing for future use
                left_line_x = min(line1_center, line2_center)
                right_line_x = max(line1_center, line2_center)
                learned_lane_spacing = distance  # Store for single line detection
                print(f"[DEBUG] Valid lane detected: L={left_line_x}, R={right_line_x}")
                print(f"[DEBUG] Learned lane spacing: {learned_lane_spacing}px (will use for single line)")
            else:
                # Distance not valid - use first line and create virtual
                detected_line_center = line1_center
                if line_width > 0:
                    lane_spacing = int(line_width * 2.5)
                else:
                    lane_spacing = 150

                print(f"[DEBUG] Distance invalid ({distance}px not in range), using line 1 with virtual")
                if detected_line_center < w // 2:
                    left_line_x = detected_line_center
                    right_line_x = detected_line_center + lane_spacing
                    is_virtual = True
                    print(f"[DEBUG] Creating virtual right: L={left_line_x}, R={right_line_x} [Virtual]")
                else:
                    right_line_x = detected_line_center
                    left_line_x = detected_line_center - lane_spacing
                    is_virtual = True
                    print(f"[DEBUG] Creating virtual left: L={left_line_x} [Virtual], R={right_line_x}")

    # Draw lane center (between left and right lines) - THIS IS THE MAIN OUTPUT
    if left_line_x is not None and right_line_x is not None:
        # Calculate lane center
        lane_center_x = (left_line_x + right_line_x) // 2
        print(f"[DEBUG] Lane center calculated: ({left_line_x} + {right_line_x}) / 2 = {lane_center_x}")

        # Draw lane center line (bright pink/yellow - main indicator)
        cv2.line(result, (lane_center_x, 0), (lane_center_x, h), (128, 255, 255), 3, cv2.LINE_AA)
        cv2.circle(result, (lane_center_x, y_center), 8, (128, 255, 255), -1)

        # Draw left and right line positions (thin, less prominent)
        left_color = (100, 100, 150) if is_virtual else (203, 192, 255)
        right_color = (100, 100, 150) if is_virtual else (203, 192, 255)

        cv2.line(result, (left_line_x, 0), (left_line_x, h), left_color, 1, cv2.LINE_AA)
        cv2.circle(result, (left_line_x, y_center), 3, left_color, -1)

        cv2.line(result, (right_line_x, 0), (right_line_x, h), right_color, 1, cv2.LINE_AA)
        cv2.circle(result, (right_line_x, y_center), 3, right_color, -1)

        # Display information
        offset_x = lane_center_x - (w // 2)
        distance_between_lines = right_line_x - left_line_x

        label = f'Lane Center:{lane_center_x} (Off:{offset_x:+d}) Angle:{lane_angle:+.1f}deg'
        if is_virtual:
            label += ' [Virtual]'
        cv2.putText(result, label, (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (128, 255, 255), 1)

        # Additional info: line positions and distance
        info = f'L:{left_line_x} R:{right_line_x} Dist:{distance_between_lines}px'
        cv2.putText(result, info, (5, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (203, 192, 255), 1)

    # Calculate and draw stop line center
    stop_x = calculate_line_center_x(stop_line_mask, h)
    if stop_x is not None:
        # Draw vertical line at stop line center
        cv2.line(result, (stop_x, 0), (stop_x, h), (0, 255, 0), 2, cv2.LINE_AA)
        # Draw marker at y-center
        cv2.circle(result, (stop_x, y_center), 5, (0, 255, 0), -1)
        # Draw x position text
        offset_x = stop_x - (w // 2)
        cv2.putText(result, f'Stop X:{stop_x} (Off:{offset_x:+d})',
                   (5, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0), 1)

    return result


def main():
    # Configuration
    script_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(script_dir, 'weights', 'best.pt')
    image_dir = os.path.join(script_dir, 'test_image', 'images')

    # Parse arguments
    if len(sys.argv) > 1:
        image_dir = sys.argv[1]

    conf_threshold = 0.3
    if len(sys.argv) > 2:
        conf_threshold = float(sys.argv[2])

    alpha = 0.5
    if len(sys.argv) > 3:
        alpha = float(sys.argv[3])

    print("=" * 70)
    print("Image Viewer - Lane (Pink) & Stop Line (Green) Overlay")
    print("=" * 70)
    print(f"Model: {model_path}")
    print(f"Images: {image_dir}")
    print(f"Confidence: {conf_threshold}")
    print(f"Alpha: {alpha}")
    print("=" * 70)

    # Load model
    if not os.path.exists(model_path):
        print(f"❌ Model not found: {model_path}")
        sys.exit(1)

    print("📥 Loading model...")

    # Check GPU availability
    device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
    print(f"🔧 Using device: {device}")

    if torch.cuda.is_available():
        # Get GPU info
        gpu_name = torch.cuda.get_device_name(0)
        total_memory = torch.cuda.get_device_properties(0).total_memory / 1024**3
        print(f"   GPU: {gpu_name}")
        print(f"   Total Memory: {total_memory:.2f} GB")

        # Set memory fraction to use only 50% of available GPU memory
        torch.cuda.set_per_process_memory_fraction(0.5, device=0)
        print(f"   Using 50% GPU memory limit")

    # Load model with device specification
    model = YOLO(model_path)

    # Note: Not using FP16 to avoid dtype mismatch issues
    # The model is small enough to run on GPU without FP16

    print("✅ Model loaded!\n")

    # Get image files
    image_files = []
    for ext in ['.jpg', '.jpeg', '.png', '.JPG', '.JPEG', '.PNG']:
        image_files.extend(glob.glob(os.path.join(image_dir, f'*{ext}')))

    image_files.sort()

    if len(image_files) == 0:
        print(f"❌ No images found in {image_dir}")
        sys.exit(1)

    print(f"📁 Found {len(image_files)} images\n")
    print("Controls:")
    print("  N / Space / → : Next image")
    print("  P / ←         : Previous image")
    print("  +             : Increase confidence")
    print("  -             : Decrease confidence")
    print("  A             : Decrease alpha (more transparent)")
    print("  D             : Increase alpha (more opaque)")
    print("  Q / ESC       : Quit")
    print("=" * 70 + "\n")

    current_idx = 0
    window_name = 'Lane & Stop Line Viewer (Pink/Green)'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    while True:
        # Load image
        image_path = image_files[current_idx]
        image = cv2.imread(image_path)

        if image is None:
            print(f"❌ Failed to load: {image_path}")
            current_idx = (current_idx + 1) % len(image_files)
            continue

        filename = os.path.basename(image_path)
        print(f"\r[{current_idx + 1}/{len(image_files)}] {filename} (Conf: {conf_threshold:.2f}, Alpha: {alpha:.2f})    ", end='', flush=True)

        # Process image - get lane, line, and stop line masks (3 classes)
        try:
            lane_mask, line_mask, stop_line_mask = create_masks(model, image, conf_threshold)
            overlay_image = overlay_lines(image, lane_mask, line_mask, stop_line_mask, alpha)
        except Exception as e:
            print(f"\n❌ Error processing image: {e}")
            # Clear GPU cache on error
            if torch.cuda.is_available():
                torch.cuda.empty_cache()
            current_idx = (current_idx + 1) % len(image_files)
            continue

        # Add info text (smaller font size and adjusted position for 640x100)
        h, w = overlay_image.shape[:2]

        # Top left - file info (moved down to avoid overlap with line info)
        cv2.putText(overlay_image, f'[{current_idx + 1}/{len(image_files)}] {filename[:35]}',
                   (5, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)

        # Top right - settings
        cv2.putText(overlay_image, f'Conf:{conf_threshold:.2f} Alpha:{alpha:.2f}',
                   (w - 150, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        # Bottom - control hints (smaller)
        cv2.putText(overlay_image, 'N:Next P:Prev +/-:Conf A/D:Alpha Q:Quit',
                   (5, h - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)

        # Show image
        cv2.imshow(window_name, overlay_image)

        # Release memory
        del lane_mask, stop_line_mask, overlay_image
        if torch.cuda.is_available():
            torch.cuda.empty_cache()

        # Handle keyboard
        key = cv2.waitKey(0) & 0xFF

        if key == ord('q') or key == 27:  # Q or ESC
            break
        elif key == ord('n') or key == ord(' ') or key == 83:  # N, Space, or Right Arrow
            current_idx = (current_idx + 1) % len(image_files)
        elif key == ord('p') or key == 81:  # P or Left Arrow
            current_idx = (current_idx - 1) % len(image_files)
        elif key == ord('+') or key == ord('='):
            conf_threshold = min(1.0, conf_threshold + 0.05)
            print(f"\n📊 Confidence: {conf_threshold:.2f}")
        elif key == ord('-') or key == ord('_'):
            conf_threshold = max(0.0, conf_threshold - 0.05)
            print(f"\n📊 Confidence: {conf_threshold:.2f}")
        elif key == ord('a'):
            alpha = max(0.0, alpha - 0.1)
            print(f"\n🎨 Alpha: {alpha:.2f}")
        elif key == ord('d'):
            alpha = min(1.0, alpha + 0.1)
            print(f"\n🎨 Alpha: {alpha:.2f}")

    cv2.destroyAllWindows()
    print("\n\n✅ Viewer closed")


if __name__ == '__main__':
    main()
