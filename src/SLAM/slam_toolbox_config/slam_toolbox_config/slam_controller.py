#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import signal
import os
import sys

class SlamController(Node):
    def __init__(self):
        super().__init__('slam_controller')
        self.slam_process = None
        
    def start_slam(self):
        """SLAM 노드 시작"""
        if self.slam_process is not None:
            self.get_logger().warn("SLAM is already running!")
            return False
            
        try:
            # SLAM launch 명령 실행
            cmd = [
                'ros2', 'launch', 
                'slam_toolbox_config', 
                'slam_launch.py', 
                'slam_mode:=scan_matching'
            ]
            
            self.slam_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # 프로세스 그룹 생성
            )
            
            self.get_logger().info("✅ SLAM started successfully!")
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to start SLAM: {e}")
            return False
    
    def stop_slam(self):
        """SLAM 노드 종료"""
        if self.slam_process is None:
            self.get_logger().warn("SLAM is not running!")
            return False
            
        try:
            # 프로세스 그룹 전체 종료 (launch와 자식 프로세스들)
            os.killpg(os.getpgid(self.slam_process.pid), signal.SIGTERM)
            
            # 종료 대기
            self.slam_process.wait(timeout=5)
            self.slam_process = None
            
            self.get_logger().info("✅ SLAM stopped successfully!")
            return True
            
        except subprocess.TimeoutExpired:
            # 강제 종료
            os.killpg(os.getpgid(self.slam_process.pid), signal.SIGKILL)
            self.slam_process = None
            self.get_logger().info("✅ SLAM force stopped!")
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to stop SLAM: {e}")
            return False
    
    def is_running(self):
        """SLAM 실행 상태 확인"""
        if self.slam_process is None:
            return False

        # 프로세스가 아직 살아있는지 확인
        if self.slam_process.poll() is None:
            return True
        else:
            self.slam_process = None
            return False

    def reset_slam(self):
        """SLAM 완전 재시작 (맵과 위치 초기화)"""
        self.get_logger().info("🔄 Resetting SLAM (stop + start)...")

        # 먼저 정지
        if self.is_running():
            if not self.stop_slam():
                return False

            # 완전히 종료될 때까지 대기
            import time
            time.sleep(2)

        # 다시 시작
        if self.start_slam():
            self.get_logger().info("✅ SLAM reset complete! Starting fresh from (0,0,0)")
            return True
        else:
            return False

def main_on():
    """slambox_on 명령어"""
    rclpy.init()
    controller = SlamController()
    
    if controller.start_slam():
        print("🚀 SLAM started! Use 'ros2 run slam_toolbox_config slambox_off' to stop.")
        try:
            rclpy.spin(controller)
        except KeyboardInterrupt:
            controller.stop_slam()
    
    controller.destroy_node()
    rclpy.shutdown()

def main_off():
    """slambox_off 명령어"""
    rclpy.init()
    controller = SlamController()
    
    if controller.stop_slam():
        print("🛑 SLAM stopped successfully!")
    else:
        print("⚠️  SLAM was not running or failed to stop.")
    
    controller.destroy_node()
    rclpy.shutdown()

def main_status():
    """slambox_status 명령어"""
    rclpy.init()
    controller = SlamController()

    status = "🟢 RUNNING" if controller.is_running() else "🔴 STOPPED"
    print(f"SLAM Status: {status}")

    controller.destroy_node()
    rclpy.shutdown()

def main_reset():
    """slambox_reset 명령어 - SLAM 완전 재시작"""
    rclpy.init()
    controller = SlamController()

    print("🔄 Resetting SLAM (complete restart with position reset to 0,0,0)...")

    if controller.reset_slam():
        print("✅ SLAM reset complete! New mapping session started.")
        try:
            rclpy.spin(controller)
        except KeyboardInterrupt:
            controller.stop_slam()
    else:
        print("❌ Failed to reset SLAM.")

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    if len(sys.argv) > 1:
        command = sys.argv[1]
        if command == 'on':
            main_on()
        elif command == 'off':
            main_off()
        elif command == 'status':
            main_status()
        elif command == 'reset':
            main_reset()
        else:
            print("Usage: python3 slam_controller.py [on|off|status|reset]")
    else:
        print("Usage: python3 slam_controller.py [on|off|status|reset]")