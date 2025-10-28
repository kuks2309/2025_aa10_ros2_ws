#!/usr/bin/env python3
"""Debug single image to understand line detection"""

import cv2
import numpy as np
from ultralytics import YOLO
import torch
import os

# GPU memory setup
if torch.cuda.is_available():
    torch.cuda.set_per_process_memory_fraction(0.5, device=0)

# Load model
script_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(script_dir, 'weights', 'best.pt')
model = YOLO(model_path)

# Find image with 2 lanes
image_dir = os.path.join(script_dir, 'test_image', 'images')
images = sorted([f for f in os.listdir(image_dir) if f.lower().endswith(('.jpg', '.png', '.jpeg'))])

# Test multiple images to find one with 2 lanes
for test_idx in range(min(50, len(images))):
    image_path = os.path.join(image_dir, images[test_idx])
    image = cv2.imread(image_path)
    h, w = image.shape[:2]

    # Run inference
    results = model.predict(image, conf=0.3, verbose=False)

    # Get masks
    lane_mask = np.zeros((h, w), dtype=np.uint8)

    if results[0].masks is not None:
        for i, cls in enumerate(results[0].boxes.cls):
            mask = results[0].masks.data[i].cpu().numpy()
            mask_resized = cv2.resize(mask, (w, h))
            mask_binary = (mask_resized > 0.5).astype(np.uint8) * 255

            if int(cls) == 0:  # lane
                lane_mask = cv2.bitwise_or(lane_mask, mask_binary)

    y_center = h // 2
    row = lane_mask[y_center, :]
    x_positions = np.where(row > 0)[0]

    # Find gaps to count lines
    gaps = []
    for i in range(len(x_positions) - 1):
        if x_positions[i + 1] - x_positions[i] > 5:
            gaps.append(i)

    num_lines = len(gaps) + 1 if len(x_positions) > 0 else 0

    if num_lines >= 2:
        print(f"\n{'='*60}")
        print(f"Found image with {num_lines} lines: {images[test_idx]}")
        print(f"Image size: {w}x{h}")
        break
else:
    print("No image with 2+ lines found in first 50 images, using last tested")
    image_path = os.path.join(image_dir, images[test_idx])
    image = cv2.imread(image_path)
    h, w = image.shape[:2]
    print(f"\nTesting image: {images[test_idx]}")
    print(f"Image size: {w}x{h}")

# Run inference
results = model.predict(image, conf=0.3, verbose=False)

# Get masks
lane_mask = np.zeros((h, w), dtype=np.uint8)
stop_line_mask = np.zeros((h, w), dtype=np.uint8)

if results[0].masks is not None:
    for i, cls in enumerate(results[0].boxes.cls):
        mask = results[0].masks.data[i].cpu().numpy()
        mask_resized = cv2.resize(mask, (w, h))
        mask_binary = (mask_resized > 0.5).astype(np.uint8) * 255

        if int(cls) == 0:  # lane
            lane_mask = cv2.bitwise_or(lane_mask, mask_binary)
        elif int(cls) == 1:  # stop line
            stop_line_mask = cv2.bitwise_or(stop_line_mask, mask_binary)

print(f"\nLane mask pixels: {np.sum(lane_mask > 0)}")
print(f"Stop line mask pixels: {np.sum(stop_line_mask > 0)}")

# Analyze center row
y_center = h // 2
row = lane_mask[y_center, :]
x_positions = np.where(row > 0)[0]

print(f"\nCenter row (y={y_center}) analysis:")
print(f"Total lane pixels: {len(x_positions)}")

if len(x_positions) > 0:
    print(f"First pixel: {x_positions[0]}")
    print(f"Last pixel: {x_positions[-1]}")

    # Find gaps
    gaps = []
    for i in range(len(x_positions) - 1):
        gap = x_positions[i + 1] - x_positions[i]
        if gap > 5:
            gaps.append((i, gap, x_positions[i], x_positions[i+1]))

    print(f"\nGaps found (>5px): {len(gaps)}")
    for i, (idx, gap, x1, x2) in enumerate(gaps):
        print(f"  Gap {i+1}: {gap}px at positions {x1}-{x2}")

    # Detect lines
    if len(gaps) == 0:
        lines = [(x_positions[0], x_positions[-1])]
    else:
        lines = []
        start = 0
        for gap_idx, _, _, _ in gaps:
            lines.append((x_positions[start], x_positions[gap_idx]))
            start = gap_idx + 1
        lines.append((x_positions[start], x_positions[-1]))

    print(f"\nDetected lines: {len(lines)}")
    for i, (left, right) in enumerate(lines):
        center = (left + right) // 2
        width = right - left + 1
        print(f"  Line {i+1}: edges=({left}, {right}), center={center}, width={width}px")

torch.cuda.empty_cache()
