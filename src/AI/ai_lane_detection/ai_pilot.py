#ai_pilot.py

import sys
import time
import asyncio
import cv2
import numpy as np
import json
import signal
import logging
import os
from pathlib import Path
import time

# YOLOv8 관련 import
from ultralytics import YOLO

# DSS SDK 관련 import
from dss_sdk.core.idsssdk import IDSSSDK
from dss_sdk.config.sdk_config import *
from dss_sdk.core.osi_manager import OSIManager
from dss_sdk.protobuf import dss_pb2

# =============== 로그 레벨 설정 ===============
logging.getLogger().setLevel(logging.CRITICAL)
logging.getLogger('nats').setLevel(logging.CRITICAL)
logging.getLogger('asyncio').setLevel(logging.CRITICAL)
logging.getLogger('DSSSDK_PY').setLevel(logging.CRITICAL)
logging.disable(logging.CRITICAL)

class SuppressOutput:
    def __enter__(self):
        self._original_stdout = sys.stdout
        self._original_stderr = sys.stderr
        sys.stdout = open(os.devnull, 'w')
        sys.stderr = open(os.devnull, 'w')
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        sys.stdout.close()
        sys.stderr.close()
        sys.stdout = self._original_stdout
        sys.stderr = self._original_stderr

class YOLOv8LaneDetector:
    def __init__(self, model_path, conf_threshold=0.25):
        self.model_path = model_path
        self.conf_threshold = conf_threshold
        self.model = None
        self.load_model()
    
    def load_model(self):
        try:
            if not os.path.exists(self.model_path):
                print(f"❌ 모델 파일을 찾을 수 없습니다: {self.model_path}")
                return False
            
            print(f"📥 YOLOv8 모델 로드 중: {self.model_path}")
            self.model = YOLO(self.model_path)
            print("✅ YOLOv8 모델 로드 완료!")
            return True
        except Exception as e:
            print(f"❌ 모델 로드 실패: {e}")
            return False
    
    def create_blue_overlay(self, image, mask, alpha=0.4, blue_color=(255, 0, 0)):
        overlay_image = image.copy()
        
        if mask is not None and mask.sum() > 0:
            if len(mask.shape) == 2:
                mask_3d = np.stack([mask] * 3, axis=-1)
            else:
                mask_3d = mask
            
            blue_layer = np.zeros_like(image)
            blue_layer[:, :] = blue_color
            
            mask_bool = mask_3d > 0
            overlay_image[mask_bool] = (
                alpha * blue_layer[mask_bool] + 
                (1 - alpha) * overlay_image[mask_bool]
            ).astype(np.uint8)
        
        return overlay_image
    
    def detect_car_region(self, image):
        height, width = image.shape[:2]
        
        car_region = {
            'x1': int(width * 0.25),
            'y1': int(height * 0.65),
            'x2': int(width * 0.75),
            'y2': int(height * 0.95)
        }
        
        return car_region
    
    def draw_anchor_line_excluding_car(self, image, y, width, car_region, color=(0, 0, 255), thickness=2):
        if y >= car_region['y1'] and y <= car_region['y2']:
            cv2.line(image, (0, y), (car_region['x1'], y), color, thickness)
            cv2.line(image, (car_region['x2'], y), (width, y), color, thickness)
        else:
            cv2.line(image, (0, y), (width, y), color, thickness)

class DSSYOLOController:
    def __init__(self, model_path):
        self.running = True
        self.rgb_image = None
        self.dss_instance = None
        self.last_key_pressed = None
        
        self.drive_state = {
            'throttle': 0.0,
            'steer': 0.0,
            'brake': 0.0
        }
        
        self.lane_detector = YOLOv8LaneDetector(model_path)
        
        # Anchor line 설정 (픽셀 값으로 관리, 위에서 0 기준)
        self.anchor_lines = []
        self.show_anchor_lines = True
        self.selected_line_index = 0
        self.edit_mode = False
        self.image_height = 480
        self.initialize_default_anchor_lines()
        
        # 성능 측정 변수
        self.fps_counter = 0
        self.fps_start_time = time.time()
        self.fps = 0
        
        print("🚗 DSS + YOLOv8 실시간 차선 감지 시스템 초기화 완료!")
    
    def initialize_default_anchor_lines(self):
        default_positions = [200, 250, 300, 350, 400, 450]
        self.anchor_lines = [pos for pos in default_positions]
        print(f"📏 기본 Anchor lines 초기화: {len(self.anchor_lines)}개 (픽셀 값)")
        self.print_anchor_positions()
    
    def update_image_height(self, height):
        if self.image_height != height:
            self.image_height = height
            print(f"📐 이미지 높이 업데이트: {height}px")
    
    def get_anchor_y_positions(self, image_height):
        self.update_image_height(image_height)
        valid_positions = [y for y in self.anchor_lines if 0 <= y < image_height]
        return valid_positions
    
    def add_anchor_line(self, pixel_position=None):
        if pixel_position is None:
            if self.anchor_lines:
                pixel_position = min(self.image_height - 20, max(self.anchor_lines) + 50)
            else:
                pixel_position = 300
        
        pixel_position = max(10, min(self.image_height - 10, pixel_position))
        
        self.anchor_lines.append(pixel_position)
        self.anchor_lines.sort()
        print(f"➕ Anchor line 추가: Y={pixel_position}px")
        self.print_anchor_positions()
    
    def remove_anchor_line(self, index=None):
        if not self.anchor_lines:
            return
        
        if index is None:
            index = self.selected_line_index
        
        if 0 <= index < len(self.anchor_lines):
            removed_pos = self.anchor_lines.pop(index)
            print(f"➖ Anchor line 제거: Y={removed_pos}px")
            
            if self.selected_line_index >= len(self.anchor_lines):
                self.selected_line_index = max(0, len(self.anchor_lines) - 1)
            
            self.print_anchor_positions()
    
    def move_selected_line(self, direction, step=5):
        if not self.anchor_lines or not (0 <= self.selected_line_index < len(self.anchor_lines)):
            return
        
        current_pos = self.anchor_lines[self.selected_line_index]
        
        if direction == 'up':
            new_pos = max(10, current_pos - step)
        else:
            new_pos = min(self.image_height - 10, current_pos + step)
        
        self.anchor_lines[self.selected_line_index] = new_pos
        self.anchor_lines.sort()
        
        self.selected_line_index = self.anchor_lines.index(new_pos)
        
        print(f"📍 Line {self.selected_line_index + 1} 이동: Y={new_pos}px")
    
    def select_next_line(self):
        if self.anchor_lines:
            self.selected_line_index = (self.selected_line_index + 1) % len(self.anchor_lines)
            selected_y = self.anchor_lines[self.selected_line_index]
            print(f"🎯 Line {self.selected_line_index + 1} 선택됨 (Y={selected_y}px)")
    
    def select_prev_line(self):
        if self.anchor_lines:
            self.selected_line_index = (self.selected_line_index - 1) % len(self.anchor_lines)
            selected_y = self.anchor_lines[self.selected_line_index]
            print(f"🎯 Line {self.selected_line_index + 1} 선택됨 (Y={selected_y}px)")
    
    def toggle_edit_mode(self):
        self.edit_mode = not self.edit_mode
        mode_text = "편집 모드" if self.edit_mode else "일반 모드"
        print(f"🔧 {mode_text} 전환")
        if self.edit_mode:
            self.print_anchor_positions()
    
    def print_anchor_status(self):
        print(f"\n📏 Anchor Lines 상태 (이미지 높이: {self.image_height}px):")
        for i, pos in enumerate(self.anchor_lines):
            marker = "👉" if i == self.selected_line_index else "  "
            print(f"{marker} Line {i+1}: Y={pos}px")
        print(f"편집 모드: {'ON' if self.edit_mode else 'OFF'}")
        print()
    
    def print_anchor_positions(self):
        if self.anchor_lines:
            positions_str = ", ".join([f"Y{pos}" for pos in self.anchor_lines])
            print(f"📍 Anchor 위치: {positions_str}")
    
    def save_anchor_config(self, filename="anchor_config.json"):
        config = {
            'anchor_lines': self.anchor_lines,
            'show_anchor_lines': self.show_anchor_lines,
            'image_height': self.image_height,
            'format': 'pixels_from_top'
        }
        try:
            with open(filename, 'w') as f:
                json.dump(config, f, indent=2)
            print(f"💾 Anchor 설정 저장: {filename}")
            self.print_anchor_positions()
        except Exception as e:
            print(f"❌ 저장 실패: {e}")
    
    def load_anchor_config(self, filename="anchor_config.json"):
        try:
            with open(filename, 'r') as f:
                config = json.load(f)
            
            if config.get('format') == 'pixels_from_top':
                self.anchor_lines = config.get('anchor_lines', [])
            else:
                old_ratios = config.get('anchor_lines', [])
                self.anchor_lines = [int(ratio * self.image_height) for ratio in old_ratios]
                print("⚡ 기존 비율 설정을 픽셀 값으로 변환했습니다.")
            
            self.show_anchor_lines = config.get('show_anchor_lines', True)
            self.selected_line_index = 0
            print(f"📂 Anchor 설정 로드: {filename}")
            self.print_anchor_positions()
            
        except Exception as e:
            print(f"❌ 로드 실패: {e}")
            self.initialize_default_anchor_lines()
    
    def find_available_models(self, base_path):
        training_dir = os.path.join(base_path, "DSS_AI_training")
        available_models = []
        
        if os.path.exists(training_dir):
            for experiment in os.listdir(training_dir):
                model_path = os.path.join(training_dir, experiment, "weights", "best.pt")
                if os.path.exists(model_path):
                    available_models.append({
                        'name': experiment,
                        'path': model_path,
                        'size': os.path.getsize(model_path) / (1024*1024)
                    })
        
        return available_models
    
    def switch_model(self, key):
        base_path = r"C:\Project\DSS\AI_Academy\yolov8"
        models = self.find_available_models(base_path)
        
        if key >= ord('1') and key <= ord('9'):
            model_index = key - ord('1')
            if model_index < len(models):
                new_model_path = models[model_index]['path']
                print(f"🔄 모델 전환 중: {models[model_index]['name']}")
                
                old_detector = self.lane_detector
                self.lane_detector = YOLOv8LaneDetector(new_model_path)
                
                if self.lane_detector.model is not None:
                    print(f"✅ 모델 전환 완료: {models[model_index]['name']}")
                    del old_detector
                else:
                    print(f"❌ 모델 전환 실패, 기존 모델 유지")
                    self.lane_detector = old_detector
    
    def handle_key_control(self, key):
        step = 0.05
        
        if key >= ord('1') and key <= ord('9'):
            self.switch_model(key)
            return
        
        if self.edit_mode:
            if key == ord('w') or key == 72:
                self.move_selected_line('up')
                return
            elif key == ord('s') or key == 80:
                self.move_selected_line('down')
                return
            elif key == ord('d') or key == 77:
                self.select_next_line()
                return
            elif key == ord('a') or key == 75:
                self.select_prev_line()
                return
            elif key == ord('q'):
                self.add_anchor_line()
                return
            elif key == ord('e'):
                self.remove_anchor_line()
                return
            elif key == ord('p'):
                self.print_anchor_status()
                return
        
        if key == ord('=') or key == ord('+'):
            self.add_anchor_line()
            return
        elif key == ord('-') or key == ord('_'):
            self.remove_anchor_line()
            return
        elif key == ord('t'):
            self.show_anchor_lines = not self.show_anchor_lines
            status = "ON" if self.show_anchor_lines else "OFF"
            print(f"[ANCHOR] Anchor lines: {status}")
            return
        elif key == ord('g'):
            self.toggle_edit_mode()
            return
        elif key == ord('h'):
            self.print_help()
            return
        elif key == ord('m'):
            self.save_anchor_config()
            return
        elif key == ord('n'):
            self.load_anchor_config()
            return
        
        if not self.edit_mode:
            if key == ord('w'):
                self.drive_state['throttle'] = min(1.0, self.drive_state['throttle'] + step)
            elif key == ord('s'):
                self.drive_state['throttle'] = max(0.0, self.drive_state['throttle'] - step)
            elif key == ord('j'):
                self.drive_state['steer'] = max(-1.0, self.drive_state['steer'] - step)
            elif key == ord('l'):
                self.drive_state['steer'] = min(1.0, self.drive_state['steer'] + step)
            elif key == ord('k'):
                self.drive_state['steer'] = 0.0
            elif key == ord('x'):
                self.drive_state['brake'] = min(1.0, self.drive_state['brake'] + step)
            elif key == ord('z'):
                self.drive_state['brake'] = max(0.0, self.drive_state['brake'] - step)
            elif key == ord('r'):
                self.drive_state = {'throttle': 0.0, 'steer': 0.0, 'brake': 0.0}
            
            print(f"[CONTROL] T:{self.drive_state['throttle']:.2f} S:{self.drive_state['steer']:.2f} B:{self.drive_state['brake']:.2f}")
    
    def print_help(self):
        print("\n" + "="*60)
        print("🎮 키보드 조작법")
        print("="*60)
        print("【차량 제어】")
        print("  W/S: 가속/감속")
        print("  J/L: 좌/우 조향")
        print("  K: 조향 중앙")
        print("  X/Z: 브레이크")
        print("  R: 모든 제어 리셋")
        print()
        print("【모델 관리】")
        print("  1-9: 모델 전환")
        print()
        print("【Anchor Line 제어】")
        print("  T: Anchor line 표시 토글")
        print("  G: 편집 모드 토글")
        print("  +: 라인 추가")
        print("  -: 라인 제거")
        print("  M: 설정 저장")
        print("  N: 설정 로드")
        print("  H: 도움말")
        print()
        print("【편집 모드에서】")
        print("  W/S 또는 ↑/↓: 선택된 라인 위/아래 이동 (5px 단위)")
        print("  A/D 또는 ←/→: 이전/다음 라인 선택")
        print("  Q: 라인 추가")
        print("  E: 선택된 라인 제거")
        print("  P: 현재 상태 출력")
        print("="*60)
        print()
    
    def calculate_fps(self):
        self.fps_counter += 1
        current_time = time.time()
        if current_time - self.fps_start_time >= 1.0:
            self.fps = self.fps_counter
            self.fps_counter = 0
            self.fps_start_time = current_time
    
    def add_overlay_info(self, image):
        # FPS 표시
        cv2.putText(image, f'FPS: {self.fps}', (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # 차량 상태 표시
        info_text = f'T:{self.drive_state["throttle"]:.2f} S:{self.drive_state["steer"]:.2f} B:{self.drive_state["brake"]:.2f}'
        cv2.putText(image, info_text, (10, 70), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Anchor line 정보 표시
        anchor_info = f'Lines: {len(self.anchor_lines)} | Mode: {"EDIT" if self.edit_mode else "DRIVE"}'
        if self.edit_mode and self.anchor_lines:
            anchor_info += f' | Selected: {self.selected_line_index + 1}'
        cv2.putText(image, anchor_info, (10, 110), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # 조작법 표시
        if self.edit_mode:
            cv2.putText(image, 'EDIT: W/S:Move A/D:Select Q:Add E:Del P:Status G:Exit H:Help', 
                       (10, image.shape[0] - 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.putText(image, 'ESC:Exit', 
                       (10, image.shape[0] - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        else:
            cv2.putText(image, 'W/S:Throttle J/L:Steer K:Center X/Z:Brake R:Reset', 
                       (10, image.shape[0] - 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            cv2.putText(image, '1-9:Model T:Toggle G:Edit +/-:Lines M/N:Save/Load H:Help ESC:Exit', 
                       (10, image.shape[0] - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        return image
    
    def draw_anchor_lines_and_center(self, image, mask):
        if mask is None or mask.sum() == 0:
            return image
        
        result_image = image.copy()
        height, width = image.shape[:2]
        
        self.update_image_height(height)
        
        car_region = self.lane_detector.detect_car_region(image)
        
        y_positions = self.get_anchor_y_positions(height)
        
        center_points = []
        
        for i, y in enumerate(y_positions):
            if y < 0 or y >= height:
                continue
            
            line_mask = mask[y, :]
            lane_pixels = np.where(line_mask > 0)[0]
            
            if len(lane_pixels) > 0:
                left_most = lane_pixels[0]
                right_most = lane_pixels[-1]
                center_x = (left_most + right_most) // 2
                center_points.append((center_x, y))
                
                line_color = (0, 255, 255) if self.edit_mode and i == self.selected_line_index else (0, 0, 255)
                line_thickness = 4 if self.edit_mode and i == self.selected_line_index else 2
                self.lane_detector.draw_anchor_line_excluding_car(result_image, y, width, car_region, line_color, line_thickness)
                
                point_color = (255, 255, 0) if self.edit_mode and i == self.selected_line_index else (0, 255, 0)
                cv2.circle(result_image, (center_x, y), 4, point_color, -1)
                
                if self.edit_mode:
                    cv2.putText(result_image, f"L{i+1}:Y{y}", (center_x + 10, y - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        # 중심점들을 연결하는 녹색선 그리기
        if len(center_points) >= 2:
            for i in range(len(center_points) - 1):
                cv2.line(result_image, center_points[i], center_points[i + 1], (0, 255, 0), 3)
        
        return result_image

    def normalize_angle(self, angle):
        """–π ~ +π 범위로 angle 정규화"""
        return (angle + np.pi) % (2 * np.pi) - np.pi
        
    def compute_stanley_control(self, center_points, vehicle_position_x, heading, k=0.5):
        if len(center_points) < 3:
            return 0.0  # insufficient anchor points

        target_point = center_points[2]  # (x, y)

        dx = target_point[0] - vehicle_position_x  # 좌우 오차 (가로 방향)
        dy = self.image_height - target_point[1]   # 전방 거리 (세로 방향, 위->아래 flip 보정)


        print(f"[Stanley Debug] vehicle_x: {vehicle_position_x}, target_x: {target_point[0]}, dx: {dx}")
        print(f"[Stanley Debug] vehicle_x: {vehicle_position_x}, target_point: {target_point}, dx: {dx}")

        path_angle = np.arctan2(dx, 3*dy)  # ⬅️ 이제 dy가 양수
        heading_error = path_angle - heading

        heading_error = self.normalize_angle(path_angle - heading)

        cross_track_error = dx / max(1e-3, np.hypot(dx, dy))
        steer = heading_error + np.arctan2(k * cross_track_error, 1.0)
        steer_deg = np.degrees(steer)

         # 디버깅 출력 추가
        print(f"[Stanley Debug] dx: {dx:.2f}, dy: {dy:.2f}, path_angle(deg): {np.degrees(path_angle):.2f}, heading(deg): {np.degrees(heading):.2f}, heading_error(deg): {np.degrees(heading_error):.2f}")
        print(f"[Stanley] dx: {dx:.2f}, dy: {dy:.2f}, cross_track_error: {cross_track_error:.4f}, heading_error: {heading_error:.4f}, steer(deg): {steer_deg:.4f}")


        steer = np.clip(steer, -np.radians(30), np.radians(30))  # 제한 각도
        return steer


    def draw_anchor_lines_and_center(self, image, mask):
        if mask is None or mask.sum() == 0:
            return image, []

        result_image = image.copy()
        height, width = image.shape[:2]

        self.update_image_height(height)
        car_region = self.lane_detector.detect_car_region(image)
        y_positions = self.get_anchor_y_positions(height)

        center_points = []

        for i, y in enumerate(y_positions):
            if y < 0 or y >= height:
                continue

            line_mask = mask[y, :]
            lane_pixels = np.where(line_mask > 0)[0]

            if len(lane_pixels) > 0:
                left_most = lane_pixels[0]
                right_most = lane_pixels[-1]
                center_x = (left_most + right_most) // 2
                center_points.append((center_x, y))

                line_color = (0, 255, 255) if self.edit_mode and i == self.selected_line_index else (0, 0, 255)
                line_thickness = 4 if self.edit_mode and i == self.selected_line_index else 2
                self.lane_detector.draw_anchor_line_excluding_car(result_image, y, width, car_region, line_color, line_thickness)

                point_color = (255, 255, 0) if self.edit_mode and i == self.selected_line_index else (0, 255, 0)
                cv2.circle(result_image, (center_x, y), 4, point_color, -1)

                if self.edit_mode:
                    cv2.putText(result_image, f"L{i+1}:Y{y}", (center_x + 10, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        if len(center_points) >= 2:
            for i in range(len(center_points) - 1):
                cv2.line(result_image, center_points[i], center_points[i + 1], (0, 255, 0), 3)

        return result_image, center_points
    
    def on_camera_data(self, image: dss_pb2.DSSImage):
        if not self.running:
            return
        
        start_time = time.time()  # 🔸 시작 시간

        try:
            jpg_data = np.frombuffer(image.data, dtype=np.uint8)
            self.rgb_image = cv2.imdecode(jpg_data, cv2.IMREAD_COLOR)

            if self.rgb_image is None:
                return

            lane_detected_image = self.rgb_image

            if self.lane_detector.model is not None:
                results = self.lane_detector.model(self.rgb_image, conf=self.lane_detector.conf_threshold, verbose=False)
                if results and results[0].masks:
                    masks = results[0].masks.data.cpu().numpy()
                    combined_mask = np.zeros((self.rgb_image.shape[0], self.rgb_image.shape[1]), dtype=np.uint8)
                    for mask in masks:
                        mask_resized = cv2.resize(mask, (self.rgb_image.shape[1], self.rgb_image.shape[0]))
                        mask_binary = (mask_resized > 0.5).astype(np.uint8) * 255
                        combined_mask = cv2.bitwise_or(combined_mask, mask_binary)

                    overlay_image = self.lane_detector.create_blue_overlay(self.rgb_image, combined_mask, alpha=0.1)

                    if self.show_anchor_lines:
                        lane_detected_image, center_points = self.draw_anchor_lines_and_center(overlay_image, combined_mask)

                        if len(center_points) >= 3:
                            vehicle_x = self.rgb_image.shape[1] // 2
                            heading = 0.0  # 정면 기준
                            steer_rad = self.compute_stanley_control(center_points, vehicle_x, heading)
                            steer_deg = np.degrees(steer_rad)
                            steer_cmd = steer_deg * 0.2  # ⬅️ 차량용 스케일 조향값
                            self.drive_state['steer'] = steer_cmd

                            # 디버깅 출력
                            print(f"[Stanley] steer_deg: {steer_deg:.2f}, steer_cmd: {steer_cmd:.2f}")
                        else:
                            lane_detected_image = overlay_image

            final_image = self.add_overlay_info(lane_detected_image)
            self.calculate_fps()

            cv2.imshow('DSS Camera + Lane Detection', final_image)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                self.running = False
                cv2.destroyAllWindows()
            elif key != 255:
                self.last_key_pressed = chr(key) if 32 <= key <= 126 else str(key)
                self.handle_key_control(key)

        except Exception as e:
            print(f"❌ 사이너 데이터 처리 오류: {e}")
        finally:
            end_time = time.time()  # 🔸 끝 시간
            duration_ms = (end_time - start_time) * 1000  # ms 단위
            print(f"[⏱️ 처리 시간] on_camera_data duration: {duration_ms:.2f} ms")
    
    def on_lidar_data(self, lidar_data):
        if not self.running:
            return
    
    def on_imu_data(self, imu):
        if not self.running:
            return
    
    def on_gps_data(self, gps):
        if not self.running:
            return
    
    def on_odom_data(self, odom):
        if not self.running:
            return
    
    def on_ground_truth_data(self, gt_data):
        if not self.running:
            return
    
    def on_sim_started(self):
        print("🟢 시뮬레이션 시작!")
    
    def on_sim_ended(self):
        print("🔴 시뮬레이션 종료!")
        self.running = False
    
    def on_sim_aborted(self):
        print("⚠️ 시뮬레이션 중단!")
        self.running = False
    
    def on_sim_error(self):
        print("❌ 시뮬레이션 오류!")
        self.running = False
    
    def update_vehicle_control(self):
        if not self.running or not self.dss_instance:
            return
        
        control = DSSSDKCarControl(
            throttle=self.drive_state['throttle'],
            steer=self.drive_state['steer'],
            brake=self.drive_state['brake'],
            park_brake=False,
            target_gear=1
        )
        
        try:
            with SuppressOutput():
                self.dss_instance.set_car_control(control)
        except Exception as e:
            print(f"❌ 차량 제어 오류: {e}")
    
    def signal_handler(self, sig, frame):
        print("\n🛑 프로그램 종료 중...")
        self.running = False
        try:
            loop = asyncio.get_event_loop()
            loop.call_soon_threadsafe(loop.stop)
        except RuntimeError:
            pass
    
    def run(self, server_ip="127.0.0.1"):
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        os.environ['PYTHONPATH'] = ''
        
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        try:
            print(f"🔗 DSS 서버 연결 중: {server_ip}")
            
            with SuppressOutput():
                dss = IDSSSDK.create(
                    loop=loop,
                    nats_address=f'nats://{server_ip}:4222'
                )
                self.dss_instance = dss
                
                init_params = DSSSDKInitParams(
                    server=server_ip,
                    heartbeat_port=8886,
                    nats_port=4222
                )
                
                dss.initialize(init_params)
                dss.register_sensor_callback('camera', self.on_camera_data)
                # dss.register_sensor_callback('lidar', self.on_lidar_data)
                # dss.register_sensor_callback('imu', self.on_imu_data)
                # dss.register_sensor_callback('gps', self.on_gps_data)
                # dss.register_sensor_callback('odom', self.on_odom_data)
                # dss.register_sensor_callback('ground_truth', self.on_ground_truth_data)
                dss.register_simulation_callback('started', self.on_sim_started)
                dss.register_simulation_callback('ended', self.on_sim_ended)
                dss.register_simulation_callback('aborted', self.on_sim_aborted)
                dss.register_simulation_callback('error', self.on_sim_error)
                
                dss.start()
            
            print("✅ DSS 연결 완료!")
            print("🎮 키보드 조작법:")
            print("   W/S: 가속/감속")
            print("   J/L: 좌회전/우회전") 
            print("   K: 조향 중앙")
            print("   X/Z: 브레이크")
            print("   R: 리셋")
            print("   1-9: 모델 전환")
            print("   T: Anchor line 표시 토글")
            print("   G: 편집 모드 전환")
            print("   +/-: Anchor line 추가/제거")
            print("   M/N: 설정 저장/로드")
            print("   H: 도움말")
            print("   ESC: 종료")
            print("=" * 50)
            
            while self.running:
                self.update_vehicle_control()
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\n⌨️  키보드 인터럽트로 종료")
            self.running = False
        except Exception as e:
            print(f"❌ 실행 중 오류: {e}")
            self.running = False
        finally:
            if self.dss_instance:
                try:
                    with SuppressOutput():
                        self.dss_instance.cleanup()
                except Exception:
                    pass
            cv2.destroyAllWindows()
            print("🏁 프로그램 종료 완료")

def main():
    print("=" * 60)
    print("🚗 DSS + YOLOv8 실시간 차선 감지 주행 시스템")
    print("=" * 60)
    
    BASE_PATH = r"C:\Project\DSS\AI_Academy\yolov8"
    
    EXPERIMENT_NAME = "DSS_experiment_1"
    
    MODEL_PATH = os.path.join(BASE_PATH, "DSS_AI_training", EXPERIMENT_NAME, "weights", "best.pt")
    
    print(f"📂 모델 경로: {MODEL_PATH}")
    
    def find_models(base_path):
        training_dir = os.path.join(base_path, "DSS_AI_training")
        available_models = []
        if os.path.exists(training_dir):
            for experiment in os.listdir(training_dir):
                model_path = os.path.join(training_dir, experiment, "weights", "best.pt")
                if os.path.exists(model_path):
                    available_models.append({
                        'name': experiment,
                        'path': model_path,
                        'size': os.path.getsize(model_path) / (1024*1024)
                    })
        return available_models
    
    available_models = find_models(BASE_PATH)
    
    if available_models:
        print(f"\n📋 사용 가능한 모델들:")
        for i, model in enumerate(available_models, 1):
            print(f"   {i}: {model['name']} ({model['size']:.1f}MB)")
        print("   실행 중 1-9 키로 모델 전환 가능")
    
    if not os.path.exists(MODEL_PATH):
        print(f"❌ 모델 파일을 찾을 수 없습니다: {MODEL_PATH}")
        print("먼저 YOLOv8 모델을 훈련해주세요.")
        return
    
    server_ip = "127.0.0.1"
    
    try:
        controller = DSSYOLOController(MODEL_PATH)
        controller.run(server_ip)
        
    except Exception as e:
        print(f"❌ 시스템 오류: {e}")
    finally:
        print("👋 시스템 종료")

if __name__ == "__main__":
    main()
