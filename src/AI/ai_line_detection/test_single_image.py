#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test AI Line Detection on a Single Image
Standalone script without ROS2 dependencies
"""

import cv2
import numpy as np
import os
import sys
import time
from ultralytics import YOLO


class LineDetectionTester:
    def __init__(self, model_path, conf_threshold=0.3, overlay_alpha=0.5):
        self.model_path = model_path
        self.conf_threshold = conf_threshold
        self.overlay_alpha = overlay_alpha
        self.model = None

    def load_model(self):
        """Load YOLOv8 segmentation model"""
        try:
            if not os.path.exists(self.model_path):
                print(f"❌ Model file not found: {self.model_path}")
                return False

            print(f"📥 Loading YOLOv8 model: {self.model_path}")
            self.model = YOLO(self.model_path)
            print("✅ YOLOv8 model loaded successfully!")
            return True
        except Exception as e:
            print(f"❌ Failed to load model: {e}")
            return False

    def create_lane_mask(self, image, results):
        """Create binary mask for lane segmentation (class 0: lane)"""
        combined_mask = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)

        if results[0].masks is not None:
            masks = results[0].masks.data.cpu().numpy()
            classes = results[0].boxes.cls.cpu().numpy().astype(int)
            confidences = results[0].boxes.conf.cpu().numpy()

            for mask, cls, conf in zip(masks, classes, confidences):
                # Only process lane class (class 0) with sufficient confidence
                if conf >= self.conf_threshold and cls == 0:
                    # Resize mask to image size
                    mask_resized = cv2.resize(mask, (image.shape[1], image.shape[0]))
                    mask_binary = (mask_resized > 0.5).astype(np.uint8) * 255
                    combined_mask = cv2.bitwise_or(combined_mask, mask_binary)

        return combined_mask

    def calculate_lane_center_error(self, mask, image_width):
        """Calculate cross-track error from lane mask"""
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) == 0:
            return 0.0, None

        # Get the largest contour (assumed to be the lane)
        largest_contour = max(contours, key=cv2.contourArea)

        # Calculate moments to find centroid
        M = cv2.moments(largest_contour)
        if M['m00'] == 0:
            return 0.0, None

        # Calculate centroid coordinates
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        # Calculate error from image center
        image_center = image_width / 2
        error = cx - image_center

        return error, (cx, cy)

    def overlay_mask_on_image(self, image, mask, color=(0, 255, 255), alpha=0.5):
        """
        Overlay mask on image with specified color and transparency
        Default color is yellow (0, 255, 255) in BGR
        """
        # Create colored overlay
        overlay = image.copy()
        overlay[mask > 0] = color

        # Blend with original image
        result = cv2.addWeighted(image, 1 - alpha, overlay, alpha, 0)

        return result

    def process_image(self, image_path, show_steps=False):
        """Process a single image and return results"""
        if self.model is None:
            print("❌ Model not loaded!")
            return None

        # Read image
        image = cv2.imread(image_path)
        if image is None:
            print(f"❌ Failed to read image: {image_path}")
            return None

        print(f"\n📸 Processing image: {os.path.basename(image_path)}")
        print(f"   Image size: {image.shape[1]}x{image.shape[0]}")

        # Run inference
        start_time = time.time()
        results = self.model(image, verbose=False)
        inference_time = (time.time() - start_time) * 1000

        # Create lane mask
        lane_mask = self.create_lane_mask(image, results)

        # Calculate cross-track error
        xte, centroid = self.calculate_lane_center_error(lane_mask, image.shape[1])

        # Create overlay image with yellow lines
        overlay_image = self.overlay_mask_on_image(
            image,
            lane_mask,
            color=(0, 255, 255),  # Yellow in BGR
            alpha=self.overlay_alpha
        )

        # Draw center line and lane centroid
        image_height, image_width = image.shape[:2]
        cv2.line(overlay_image,
                (image_width // 2, 0),
                (image_width // 2, image_height),
                (0, 0, 255), 2)  # Red center line

        if centroid is not None:
            cv2.circle(overlay_image, centroid, 10, (255, 0, 0), -1)  # Blue centroid
            cv2.line(overlay_image,
                    (image_width // 2, centroid[1]),
                    (centroid[0], centroid[1]),
                    (255, 0, 255), 2)  # Magenta error line

        # Add info text
        cv2.putText(overlay_image, f'Inference: {inference_time:.1f}ms',
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(overlay_image, f'XTE: {xte:.1f} px',
                   (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(overlay_image, f'Confidence: {self.conf_threshold}',
                   (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # Print results
        print(f"   ⏱️  Inference time: {inference_time:.1f}ms")
        print(f"   📏 Cross-track error: {xte:.1f} pixels")
        if centroid:
            print(f"   🎯 Lane centroid: ({centroid[0]}, {centroid[1]})")

        # Show intermediate steps if requested
        if show_steps:
            # Create mask visualization (colorized)
            mask_colored = cv2.applyColorMap(lane_mask, cv2.COLORMAP_JET)

            # Stack images for comparison
            original_resized = cv2.resize(image, (640, 480))
            mask_resized = cv2.resize(mask_colored, (640, 480))
            overlay_resized = cv2.resize(overlay_image, (640, 480))

            top_row = np.hstack([original_resized, mask_resized])
            bottom_row = np.hstack([overlay_resized, overlay_resized])
            comparison = np.vstack([top_row, bottom_row])

            cv2.imshow('Processing Steps', comparison)
            cv2.waitKey(0)

        return {
            'original': image,
            'mask': lane_mask,
            'overlay': overlay_image,
            'xte': xte,
            'centroid': centroid,
            'inference_time': inference_time
        }


def main():
    # Configuration
    script_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(script_dir, 'weights', 'best.pt')

    # Parse command line arguments
    if len(sys.argv) < 2:
        print("Usage: python3 test_single_image.py <image_path> [conf_threshold] [show_steps]")
        print("\nExample:")
        print("  python3 test_single_image.py test_image/images/image1.jpg")
        print("  python3 test_single_image.py test_image/images/image1.jpg 0.5")
        print("  python3 test_single_image.py test_image/images/image1.jpg 0.5 --show")
        sys.exit(1)

    image_path = sys.argv[1]
    conf_threshold = float(sys.argv[2]) if len(sys.argv) > 2 else 0.3
    show_steps = '--show' in sys.argv or '-s' in sys.argv

    # Check if image exists
    if not os.path.exists(image_path):
        print(f"❌ Image not found: {image_path}")
        sys.exit(1)

    # Initialize tester
    print("=" * 70)
    print("AI Line Detection - Single Image Test")
    print("=" * 70)

    tester = LineDetectionTester(
        model_path=model_path,
        conf_threshold=conf_threshold,
        overlay_alpha=0.5
    )

    # Load model
    if not tester.load_model():
        sys.exit(1)

    # Process image
    results = tester.process_image(image_path, show_steps=show_steps)

    if results is not None:
        # Save output
        output_dir = os.path.join(script_dir, 'test_image', 'output')
        os.makedirs(output_dir, exist_ok=True)

        base_name = os.path.splitext(os.path.basename(image_path))[0]
        overlay_path = os.path.join(output_dir, f"{base_name}_overlay.jpg")
        mask_path = os.path.join(output_dir, f"{base_name}_mask.jpg")

        cv2.imwrite(overlay_path, results['overlay'])
        cv2.imwrite(mask_path, results['mask'])

        print(f"\n✅ Results saved:")
        print(f"   📁 Overlay: {overlay_path}")
        print(f"   📁 Mask: {mask_path}")

        # Display result
        print("\n👁️  Displaying result (press any key to close)...")
        cv2.imshow('Yellow Line Overlay', results['overlay'])
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    print("\n" + "=" * 70)
    print("Test completed!")
    print("=" * 70)


if __name__ == '__main__':
    main()
