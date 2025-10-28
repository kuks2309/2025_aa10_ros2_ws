#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Interactive Viewer for AI Line Detection Test Results
Browse through test images with keyboard controls
"""

import cv2
import numpy as np
import os
import sys
import glob
from ultralytics import YOLO


class InteractiveViewer:
    def __init__(self, model_path, conf_threshold=0.3, overlay_alpha=0.5):
        self.model_path = model_path
        self.conf_threshold = conf_threshold
        self.overlay_alpha = overlay_alpha
        self.model = None
        self.current_index = 0
        self.image_files = []
        self.cache = {}

    def load_model(self):
        """Load YOLOv8 model"""
        try:
            if not os.path.exists(self.model_path):
                print(f"❌ Model file not found: {self.model_path}")
                return False

            print(f"📥 Loading YOLOv8 model...")
            self.model = YOLO(self.model_path)
            print("✅ Model loaded!")
            return True
        except Exception as e:
            print(f"❌ Failed to load model: {e}")
            return False

    def load_images(self, image_dir, extensions=['.jpg', '.jpeg', '.png']):
        """Load all images from directory"""
        self.image_files = []
        for ext in extensions:
            self.image_files.extend(glob.glob(os.path.join(image_dir, f'*{ext}')))
            self.image_files.extend(glob.glob(os.path.join(image_dir, f'*{ext.upper()}')))

        self.image_files.sort()
        print(f"📁 Loaded {len(self.image_files)} images")
        return len(self.image_files) > 0

    def create_lane_mask(self, image, results):
        """Create binary mask for lanes"""
        combined_mask = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)

        if results[0].masks is not None:
            masks = results[0].masks.data.cpu().numpy()
            classes = results[0].boxes.cls.cpu().numpy().astype(int)
            confidences = results[0].boxes.conf.cpu().numpy()

            for mask, cls, conf in zip(masks, classes, confidences):
                if conf >= self.conf_threshold and cls == 0:
                    mask_resized = cv2.resize(mask, (image.shape[1], image.shape[0]))
                    mask_binary = (mask_resized > 0.5).astype(np.uint8) * 255
                    combined_mask = cv2.bitwise_or(combined_mask, mask_binary)

        return combined_mask

    def calculate_lane_center_error(self, mask, image_width):
        """Calculate XTE"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) == 0:
            return 0.0, None

        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)

        if M['m00'] == 0:
            return 0.0, None

        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        error = cx - image_width / 2

        return error, (cx, cy)

    def process_image(self, image_path):
        """Process single image"""
        # Check cache
        if image_path in self.cache:
            return self.cache[image_path]

        image = cv2.imread(image_path)
        if image is None:
            return None

        # Run inference
        results = self.model(image, verbose=False)
        lane_mask = self.create_lane_mask(image, results)
        xte, centroid = self.calculate_lane_center_error(lane_mask, image.shape[1])

        # Create overlay
        overlay = image.copy()
        overlay[lane_mask > 0] = (0, 255, 255)  # Yellow
        overlay_image = cv2.addWeighted(image, 1 - self.overlay_alpha, overlay, self.overlay_alpha, 0)

        # Draw visualizations
        h, w = image.shape[:2]
        cv2.line(overlay_image, (w // 2, 0), (w // 2, h), (0, 0, 255), 2)  # Red center

        if centroid is not None:
            cv2.circle(overlay_image, centroid, 10, (255, 0, 0), -1)  # Blue centroid
            cv2.line(overlay_image, (w // 2, centroid[1]), (centroid[0], centroid[1]), (255, 0, 255), 2)  # Error line

        result = {
            'original': image,
            'mask': lane_mask,
            'overlay': overlay_image,
            'xte': xte,
            'centroid': centroid
        }

        # Cache result
        self.cache[image_path] = result
        return result

    def create_display_image(self, result, filename):
        """Create display with multiple views"""
        if result is None:
            return None

        # Resize images for display
        original = cv2.resize(result['original'], (640, 480))
        overlay = cv2.resize(result['overlay'], (640, 480))

        # Create colored mask
        mask_colored = cv2.applyColorMap(result['mask'], cv2.COLORMAP_JET)
        mask_colored = cv2.resize(mask_colored, (640, 480))

        # Create comparison view
        top_row = np.hstack([original, overlay])
        bottom_row = np.hstack([mask_colored, overlay])
        display = np.vstack([top_row, bottom_row])

        # Add labels
        cv2.putText(display, 'Original', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(display, 'Overlay (Yellow)', (650, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(display, 'Mask (Heat Map)', (10, 510), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(display, 'Result', (650, 510), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        # Add info panel at bottom
        info_panel = np.zeros((100, 1280, 3), dtype=np.uint8)
        cv2.putText(info_panel, f'File: {filename}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(info_panel, f'XTE: {result["xte"]:.1f} px', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(info_panel, f'Image {self.current_index + 1}/{len(self.image_files)}', (500, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(info_panel, f'Conf: {self.conf_threshold}', (500, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # Add controls
        cv2.putText(info_panel, '[N]ext | [P]rev | [+/-]Conf | [A/D]Alpha | [S]ave | [Q]uit', (800, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        display = np.vstack([display, info_panel])

        return display

    def run(self, image_dir, output_dir=None):
        """Run interactive viewer"""
        if not self.load_images(image_dir):
            print("❌ No images found!")
            return

        if output_dir:
            os.makedirs(output_dir, exist_ok=True)

        print("\n" + "=" * 70)
        print("INTERACTIVE VIEWER CONTROLS")
        print("=" * 70)
        print("  N / Right Arrow  - Next image")
        print("  P / Left Arrow   - Previous image")
        print("  + / =            - Increase confidence threshold")
        print("  - / _            - Decrease confidence threshold")
        print("  A                - Decrease alpha (more transparent)")
        print("  D                - Increase alpha (more opaque)")
        print("  S                - Save current result")
        print("  Q / ESC          - Quit")
        print("=" * 70 + "\n")

        window_name = 'AI Line Detection - Interactive Viewer'
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

        while True:
            # Process current image
            image_path = self.image_files[self.current_index]
            filename = os.path.basename(image_path)

            print(f"\r[{self.current_index + 1}/{len(self.image_files)}] {filename} (Conf: {self.conf_threshold:.2f}, Alpha: {self.overlay_alpha:.2f})    ", end='', flush=True)

            result = self.process_image(image_path)
            display = self.create_display_image(result, filename)

            if display is not None:
                cv2.imshow(window_name, display)

            # Handle keyboard input
            key = cv2.waitKey(0) & 0xFF

            if key == ord('q') or key == 27:  # Q or ESC
                break
            elif key == ord('n') or key == 83:  # N or Right Arrow
                self.current_index = (self.current_index + 1) % len(self.image_files)
            elif key == ord('p') or key == 81:  # P or Left Arrow
                self.current_index = (self.current_index - 1) % len(self.image_files)
            elif key == ord('+') or key == ord('='):
                self.conf_threshold = min(1.0, self.conf_threshold + 0.05)
                self.cache.clear()
                print(f"\n📊 Confidence threshold: {self.conf_threshold:.2f}")
            elif key == ord('-') or key == ord('_'):
                self.conf_threshold = max(0.0, self.conf_threshold - 0.05)
                self.cache.clear()
                print(f"\n📊 Confidence threshold: {self.conf_threshold:.2f}")
            elif key == ord('a'):
                self.overlay_alpha = max(0.0, self.overlay_alpha - 0.1)
                self.cache.clear()
                print(f"\n🎨 Alpha: {self.overlay_alpha:.2f}")
            elif key == ord('d'):
                self.overlay_alpha = min(1.0, self.overlay_alpha + 0.1)
                self.cache.clear()
                print(f"\n🎨 Alpha: {self.overlay_alpha:.2f}")
            elif key == ord('s'):
                if output_dir and result is not None:
                    base_name = os.path.splitext(filename)[0]
                    save_path = os.path.join(output_dir, f"{base_name}_overlay.jpg")
                    cv2.imwrite(save_path, result['overlay'])
                    print(f"\n💾 Saved: {save_path}")

        cv2.destroyAllWindows()
        print("\n\n✅ Viewer closed")


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(script_dir, 'weights', 'best.pt')
    default_image_dir = os.path.join(script_dir, 'test_image', 'images')
    default_output_dir = os.path.join(script_dir, 'test_image', 'output')

    image_dir = sys.argv[1] if len(sys.argv) > 1 else default_image_dir
    output_dir = sys.argv[2] if len(sys.argv) > 2 else default_output_dir

    print("=" * 70)
    print("AI Line Detection - Interactive Viewer")
    print("=" * 70)

    viewer = InteractiveViewer(model_path, conf_threshold=0.3, overlay_alpha=0.5)

    if not viewer.load_model():
        sys.exit(1)

    viewer.run(image_dir, output_dir)


if __name__ == '__main__':
    main()
