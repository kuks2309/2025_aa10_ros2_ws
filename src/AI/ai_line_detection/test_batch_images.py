#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test AI Line Detection on Multiple Images (Batch Processing)
Processes all images in a directory and generates a summary report
"""

import cv2
import numpy as np
import os
import sys
import time
from ultralytics import YOLO
import glob
from pathlib import Path


class BatchLineDetectionTester:
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
        """Create binary mask for lane segmentation"""
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
        """Calculate cross-track error from lane mask"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) == 0:
            return 0.0, None

        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)

        if M['m00'] == 0:
            return 0.0, None

        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        image_center = image_width / 2
        error = cx - image_center

        return error, (cx, cy)

    def overlay_mask_on_image(self, image, mask, color=(0, 255, 255), alpha=0.5):
        """Overlay mask on image with yellow color"""
        overlay = image.copy()
        overlay[mask > 0] = color
        result = cv2.addWeighted(image, 1 - alpha, overlay, alpha, 0)
        return result

    def process_image(self, image_path):
        """Process a single image"""
        if self.model is None:
            return None

        image = cv2.imread(image_path)
        if image is None:
            return None

        # Run inference
        start_time = time.time()
        results = self.model(image, verbose=False)
        inference_time = (time.time() - start_time) * 1000

        # Create lane mask
        lane_mask = self.create_lane_mask(image, results)

        # Calculate XTE
        xte, centroid = self.calculate_lane_center_error(lane_mask, image.shape[1])

        # Create overlay
        overlay_image = self.overlay_mask_on_image(image, lane_mask, (0, 255, 255), self.overlay_alpha)

        # Draw visualizations
        image_height, image_width = image.shape[:2]
        cv2.line(overlay_image, (image_width // 2, 0), (image_width // 2, image_height), (0, 0, 255), 2)

        if centroid is not None:
            cv2.circle(overlay_image, centroid, 10, (255, 0, 0), -1)
            cv2.line(overlay_image, (image_width // 2, centroid[1]), (centroid[0], centroid[1]), (255, 0, 255), 2)

        # Add text
        cv2.putText(overlay_image, f'Time: {inference_time:.1f}ms', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(overlay_image, f'XTE: {xte:.1f}px', (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Check if lane detected
        lane_detected = np.sum(lane_mask > 0) > 100  # At least 100 pixels

        return {
            'overlay': overlay_image,
            'mask': lane_mask,
            'xte': xte,
            'inference_time': inference_time,
            'lane_detected': lane_detected,
            'centroid': centroid
        }

    def process_batch(self, image_dir, output_dir, image_extensions=['.jpg', '.jpeg', '.png']):
        """Process all images in directory"""
        # Get all image files
        image_files = []
        for ext in image_extensions:
            image_files.extend(glob.glob(os.path.join(image_dir, f'*{ext}')))
            image_files.extend(glob.glob(os.path.join(image_dir, f'*{ext.upper()}')))

        if len(image_files) == 0:
            print(f"❌ No images found in {image_dir}")
            return

        print(f"\n📁 Found {len(image_files)} images")
        print("=" * 70)

        # Create output directory
        os.makedirs(output_dir, exist_ok=True)

        # Statistics
        total_time = 0
        successful = 0
        failed = 0
        lane_detected_count = 0
        xte_values = []

        # Process each image
        for idx, image_path in enumerate(image_files, 1):
            base_name = os.path.basename(image_path)
            print(f"\n[{idx}/{len(image_files)}] Processing: {base_name}")

            try:
                results = self.process_image(image_path)

                if results is not None:
                    # Save outputs
                    name_no_ext = os.path.splitext(base_name)[0]
                    overlay_path = os.path.join(output_dir, f"{name_no_ext}_overlay.jpg")
                    mask_path = os.path.join(output_dir, f"{name_no_ext}_mask.jpg")

                    cv2.imwrite(overlay_path, results['overlay'])
                    cv2.imwrite(mask_path, results['mask'])

                    # Update statistics
                    total_time += results['inference_time']
                    successful += 1
                    if results['lane_detected']:
                        lane_detected_count += 1
                        xte_values.append(results['xte'])

                    print(f"   ⏱️  {results['inference_time']:.1f}ms | XTE: {results['xte']:6.1f}px | Detected: {results['lane_detected']}")
                else:
                    failed += 1
                    print(f"   ❌ Failed to process")

            except Exception as e:
                failed += 1
                print(f"   ❌ Error: {e}")

        # Print summary
        print("\n" + "=" * 70)
        print("BATCH PROCESSING SUMMARY")
        print("=" * 70)
        print(f"Total images:          {len(image_files)}")
        print(f"Successfully processed: {successful}")
        print(f"Failed:                {failed}")
        print(f"Lane detected:         {lane_detected_count} ({lane_detected_count/successful*100:.1f}%)" if successful > 0 else "Lane detected: 0")

        if successful > 0:
            avg_time = total_time / successful
            print(f"\nAverage inference time: {avg_time:.1f}ms")
            print(f"FPS (average):         {1000/avg_time:.1f}")

        if len(xte_values) > 0:
            print(f"\nCross-Track Error Statistics:")
            print(f"  Mean XTE:   {np.mean(xte_values):6.1f}px")
            print(f"  Std XTE:    {np.std(xte_values):6.1f}px")
            print(f"  Min XTE:    {np.min(xte_values):6.1f}px")
            print(f"  Max XTE:    {np.max(xte_values):6.1f}px")

        print(f"\n📁 Output directory: {output_dir}")
        print("=" * 70)

        # Create a summary grid
        self.create_summary_grid(image_files[:16], output_dir, max_images=16)

    def create_summary_grid(self, image_files, output_dir, max_images=16):
        """Create a grid of overlay images for quick review"""
        print(f"\n📊 Creating summary grid...")

        overlay_images = []
        for image_path in image_files[:max_images]:
            name_no_ext = os.path.splitext(os.path.basename(image_path))[0]
            overlay_path = os.path.join(output_dir, f"{name_no_ext}_overlay.jpg")

            if os.path.exists(overlay_path):
                img = cv2.imread(overlay_path)
                if img is not None:
                    # Resize to fixed size for grid
                    img_resized = cv2.resize(img, (320, 240))
                    overlay_images.append(img_resized)

        if len(overlay_images) == 0:
            print("   ⚠️  No overlay images found")
            return

        # Create grid (4 columns)
        cols = 4
        rows = (len(overlay_images) + cols - 1) // cols

        # Pad with black images if needed
        while len(overlay_images) < rows * cols:
            black_img = np.zeros((240, 320, 3), dtype=np.uint8)
            overlay_images.append(black_img)

        # Create grid
        grid_rows = []
        for r in range(rows):
            row_images = overlay_images[r*cols:(r+1)*cols]
            grid_row = np.hstack(row_images)
            grid_rows.append(grid_row)

        grid = np.vstack(grid_rows)

        # Save grid
        grid_path = os.path.join(output_dir, 'summary_grid.jpg')
        cv2.imwrite(grid_path, grid)
        print(f"   ✅ Summary grid saved: {grid_path}")


def main():
    # Configuration
    script_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(script_dir, 'weights', 'best.pt')
    default_image_dir = os.path.join(script_dir, 'test_image', 'images')
    default_output_dir = os.path.join(script_dir, 'test_image', 'output')

    # Parse arguments
    image_dir = sys.argv[1] if len(sys.argv) > 1 else default_image_dir
    output_dir = sys.argv[2] if len(sys.argv) > 2 else default_output_dir
    conf_threshold = float(sys.argv[3]) if len(sys.argv) > 3 else 0.3

    print("=" * 70)
    print("AI Line Detection - Batch Image Test")
    print("=" * 70)
    print(f"Image directory:  {image_dir}")
    print(f"Output directory: {output_dir}")
    print(f"Model path:       {model_path}")
    print(f"Conf threshold:   {conf_threshold}")

    # Initialize tester
    tester = BatchLineDetectionTester(
        model_path=model_path,
        conf_threshold=conf_threshold,
        overlay_alpha=0.5
    )

    # Load model
    if not tester.load_model():
        sys.exit(1)

    # Process batch
    tester.process_batch(image_dir, output_dir)


if __name__ == '__main__':
    main()
