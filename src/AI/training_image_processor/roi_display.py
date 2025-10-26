#!/usr/bin/env python3
import cv2
import json
import os
import argparse
from pathlib import Path
import numpy as np

class ROIDisplay:
    def __init__(self, image_folder, roi_config_file, crop_folder=None):
        self.image_folder = Path(image_folder)
        self.roi_config = self.load_roi_config(roi_config_file)
        self.current_index = 0
        self.image_files = sorted([f for f in self.image_folder.glob("*.jpg")])
        self.crop_folder = Path(crop_folder) if crop_folder else Path("crop")
        
        if not self.image_files:
            self.image_files = sorted([f for f in self.image_folder.glob("*.png")])
        
        if not self.image_files:
            raise ValueError(f"No images found in {image_folder}")
            
        print(f"Found {len(self.image_files)} images")
        
        self.crop_folder.mkdir(exist_ok=True)
        print(f"Crop folder: {self.crop_folder}")
    
    def load_roi_config(self, config_file):
        with open(config_file, 'r') as f:
            config = json.load(f)
        return config
    
    def draw_roi(self, image):
        x = self.roi_config.get("x", 0)
        y = self.roi_config.get("y", 0)
        width = self.roi_config.get("width", 100)
        height = self.roi_config.get("height", 100)
        
        color = (0, 255, 0)
        thickness = 2
        
        cv2.rectangle(image, (x, y), (x + width, y + height), color, thickness)
        
        return image
    
    def crop_and_save(self, image, filename):
        x = self.roi_config.get("x", 0)
        y = self.roi_config.get("y", 0)
        width = self.roi_config.get("width", 100)
        height = self.roi_config.get("height", 100)
        
        cropped = image[y:y+height, x:x+width]
        
        new_width = width // 2
        new_height = height // 2
        resized = cv2.resize(cropped, (new_width, new_height), interpolation=cv2.INTER_AREA)
        
        output_path = self.crop_folder / f"crop_{filename}"
        cv2.imwrite(str(output_path), resized)
        
        return output_path
    
    def display_image_with_roi(self):
        print("\nControls:")
        print("  'n' or 'Space' - Next image")
        print("  'p' - Previous image")
        print("  'c' - Crop and save current ROI")
        print("  'a' - Crop and save all images")
        print("  'q' or 'ESC' - Quit")
        print(f"\nStarting with image 1/{len(self.image_files)}")
        print(f"\nROI: x={self.roi_config['x']}, y={self.roi_config['y']}, width={self.roi_config['width']}, height={self.roi_config['height']}")
        print(f"Output size: {self.roi_config['width']//2} x {self.roi_config['height']//2}")
        
        while True:
            current_file = self.image_files[self.current_index]
            image = cv2.imread(str(current_file))
            
            if image is None:
                print(f"Error loading image: {current_file}")
                self.current_index = (self.current_index + 1) % len(self.image_files)
                continue
            
            display_image = self.draw_roi(image.copy())
            
            window_name = f"ROI Display - {current_file.name} ({self.current_index + 1}/{len(self.image_files)})"
            cv2.imshow(window_name, display_image)
            
            key = cv2.waitKey(0) & 0xFF
            
            cv2.destroyAllWindows()
            
            if key in [ord('q'), 27]:
                print("Quitting...")
                break
            elif key in [ord('n'), ord(' ')]:
                self.current_index = (self.current_index + 1) % len(self.image_files)
                print(f"Image {self.current_index + 1}/{len(self.image_files)}")
            elif key == ord('p'):
                self.current_index = (self.current_index - 1) % len(self.image_files)
                print(f"Image {self.current_index + 1}/{len(self.image_files)}")
            elif key == ord('c'):
                output_path = self.crop_and_save(image, current_file.name)
                print(f"Saved cropped image to {output_path}")
            elif key == ord('a'):
                print("Cropping all images...")
                for i, img_file in enumerate(self.image_files):
                    img = cv2.imread(str(img_file))
                    if img is not None:
                        self.crop_and_save(img, img_file.name)
                        print(f"Progress: {i+1}/{len(self.image_files)}", end='\r')
                print(f"\nCompleted! Cropped {len(self.image_files)} images to {self.crop_folder}")
            elif key == ord('s'):
                save_path = f"roi_{current_file.name}"
                cv2.imwrite(save_path, display_image)
                print(f"Saved image with ROI to {save_path}")

def main():
    parser = argparse.ArgumentParser(description="Display images with ROI from JSON configuration")
    parser.add_argument("--image-folder", "-i", 
                       default="/home/amap/2025_aa10_ros2_ws/images/20251026-2",
                       help="Path to the folder containing images")
    parser.add_argument("--roi-config", "-r",
                       default="roi_config.json",
                       help="Path to ROI configuration JSON file")
    parser.add_argument("--crop-folder", "-o",
                       default="crop",
                       help="Path to output folder for cropped images")
    
    args = parser.parse_args()
    
    if not os.path.exists(args.image_folder):
        print(f"Error: Image folder '{args.image_folder}' does not exist")
        return
    
    if not os.path.exists(args.roi_config):
        print(f"Error: ROI configuration file '{args.roi_config}' does not exist")
        print("Creating a sample configuration file...")
        
        sample_config = {
            "x": 150,
            "y": 200,
            "width": 340,
            "height": 200
        }
        
        with open(args.roi_config, 'w') as f:
            json.dump(sample_config, f, indent=4)
        
        print(f"Created sample configuration file: {args.roi_config}")
        print("Please edit this file to define your ROI and run the program again.")
        return
    
    try:
        roi_display = ROIDisplay(args.image_folder, args.roi_config, args.crop_folder)
        roi_display.display_image_with_roi()
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()