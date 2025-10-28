#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
AA10 Yaw Control Test GUI for ROS2
Tests aa10_car_yaw_control node with different control modes
"""

import sys
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import Int8, Int16, Float32
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import QApplication, QDialog, QButtonGroup
from PyQt5 import uic

# Load UI file
script_dir = os.path.dirname(os.path.abspath(__file__))
ui_file = os.path.join(script_dir, 'yaw_control_test.ui')
form_class = uic.loadUiType(ui_file)[0]


class YawControlTestNode(Node):
    """ROS2 Node for Yaw Control Testing"""

    def __init__(self):
        super().__init__('yaw_control_test_node')

        qos_profile = QoSProfile(depth=10)

        # Publishers - matching aa10_car_yaw_control topics
        self.mode_pub = self.create_publisher(
            Int8, '/car_control/steering_control_mode', qos_profile)

        self.target_angle_pub = self.create_publisher(
            Float32, '/car_control/target_angle', qos_profile)

        self.vision_xte_pub = self.create_publisher(
            Float32, '/xte/vision', qos_profile)

        self.maze_xte_pub = self.create_publisher(
            Float32, '/xte/maze', qos_profile)

        self.steer_input_pub = self.create_publisher(
            Int16, '/xte/steer', qos_profile)

        # Subscribers - for feedback
        self.imu_angle_sub = self.create_subscription(
            Float32, '/handsfree/imu_yaw_degree',
            self.imu_angle_callback, qos_profile)

        self.steering_output_sub = self.create_subscription(
            Int16, '/car_control/steering_angle',
            self.steering_output_callback, qos_profile)

        # State variables
        self.current_mode = -1
        self.current_imu_angle = 0.0
        self.current_steering_output = 0
        self.target_angle = 0.0

        self.get_logger().info('Yaw Control Test Node initialized')

    def imu_angle_callback(self, msg):
        """Receive current IMU angle"""
        self.current_imu_angle = msg.data

    def steering_output_callback(self, msg):
        """Receive steering output from yaw control"""
        self.current_steering_output = msg.data

    def publish_mode(self, mode):
        """Publish control mode"""
        msg = Int8()
        msg.data = mode
        self.mode_pub.publish(msg)
        self.current_mode = mode
        self.get_logger().info(f'Control mode set to: {mode}')

    def publish_target_angle(self, angle):
        """Publish target angle for IMU control"""
        msg = Float32()
        msg.data = float(angle)
        self.target_angle_pub.publish(msg)
        self.target_angle = angle
        self.get_logger().info(f'Target angle: {angle} deg')

    def publish_vision_xte(self, xte):
        """Publish vision cross-track error"""
        msg = Float32()
        msg.data = float(xte)
        self.vision_xte_pub.publish(msg)
        self.get_logger().info(f'Vision XTE: {xte}')

    def publish_maze_xte(self, xte):
        """Publish maze cross-track error"""
        msg = Float32()
        msg.data = float(xte)
        self.maze_xte_pub.publish(msg)
        self.get_logger().info(f'Maze XTE: {xte}')

    def publish_steer_input(self, steer):
        """Publish direct steer input"""
        msg = Int16()
        msg.data = int(steer)
        self.steer_input_pub.publish(msg)
        self.get_logger().info(f'Steer input: {steer}')


class YawControlTestGUI(QDialog, form_class):
    """PyQt5 GUI for Yaw Control Testing"""

    def __init__(self, ros_node):
        super().__init__()
        self.setupUi(self)

        self.ros_node = ros_node

        # Setup radio button group
        self.mode_button_group = QButtonGroup(self)
        self.mode_button_group.addButton(self.radioButton_imu_control, 0)
        self.mode_button_group.addButton(self.radioButton_lane_control, 1)
        self.mode_button_group.addButton(self.radioButton_maze_control, 2)
        self.mode_button_group.addButton(self.radioButton_steer_control, 3)
        self.mode_button_group.buttonClicked.connect(self.on_mode_changed)

        # Connect buttons
        self.pushButton_send_target_angle.clicked.connect(self.send_target_angle)
        self.pushButton_send_vision_xte.clicked.connect(self.send_vision_xte)
        self.pushButton_send_maze_xte.clicked.connect(self.send_maze_xte)
        self.pushButton_send_steer.clicked.connect(self.send_steer_input)

        # Timer for updating display
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_display)
        self.update_timer.start(100)  # 10Hz update

        # Mode names
        self.mode_names = {
            0: "IMU Control",
            1: "Lane/Vision Control",
            2: "Maze Control",
            3: "Steer Control"
        }

        self.get_logger_info('GUI initialized')

    def get_logger_info(self, msg):
        """Helper to log to console"""
        print(f"[GUI] {msg}")

    def on_mode_changed(self, button):
        """Handle mode selection"""
        mode = self.mode_button_group.id(button)
        self.ros_node.publish_mode(mode)

        mode_name = self.mode_names.get(mode, "Unknown")
        self.label_current_mode.setText(f"Current Mode: {mode_name} ({mode})")
        self.get_logger_info(f"Mode changed to: {mode_name}")

    def send_target_angle(self):
        """Send target angle for IMU control"""
        try:
            angle = float(self.lineEdit_target_angle.text())
            self.ros_node.publish_target_angle(angle)
        except ValueError:
            self.get_logger_info("Invalid target angle value")

    def send_vision_xte(self):
        """Send vision XTE"""
        try:
            xte = float(self.lineEdit_vision_xte.text())
            self.ros_node.publish_vision_xte(xte)
        except ValueError:
            self.get_logger_info("Invalid vision XTE value")

    def send_maze_xte(self):
        """Send maze XTE"""
        try:
            xte = float(self.lineEdit_maze_xte.text())
            self.ros_node.publish_maze_xte(xte)
        except ValueError:
            self.get_logger_info("Invalid maze XTE value")

    def send_steer_input(self):
        """Send direct steer input"""
        try:
            steer = int(self.lineEdit_steer_input.text())
            self.ros_node.publish_steer_input(steer)
        except ValueError:
            self.get_logger_info("Invalid steer input value")

    def update_display(self):
        """Update display with current values"""
        # Update ROS status
        if rclpy.ok():
            self.label_ros_status.setText("ROS2 Status: Connected")
            self.label_ros_status.setStyleSheet("color: green;")
        else:
            self.label_ros_status.setText("ROS2 Status: Disconnected")
            self.label_ros_status.setStyleSheet("color: red;")

        # Update IMU angle
        imu_angle = self.ros_node.current_imu_angle
        self.label_current_imu_angle.setText(f"Current IMU Angle: {imu_angle:.2f} deg")

        # Calculate and display IMU error
        if self.ros_node.current_mode == 0:  # IMU Control mode
            try:
                target = float(self.lineEdit_target_angle.text())
                error = target - imu_angle

                # Normalize error to -180 to 180
                if error > 180:
                    error -= 360
                elif error < -180:
                    error += 360

                self.label_imu_error.setText(f"Error: {error:.2f} deg")
            except:
                pass

        # Update steering output
        steering = self.ros_node.current_steering_output
        self.label_steering_output.setText(f"Final Steering Output: {steering} (Int16)")

        # Update mode-specific displays
        if self.ros_node.current_mode == 0:  # IMU
            self.label_imu_steering.setText(f"Steering Output: {steering} deg")
        elif self.ros_node.current_mode == 1:  # Vision
            self.label_vision_steering.setText(f"Steering Output: {steering} deg")
        elif self.ros_node.current_mode == 2:  # Maze
            self.label_maze_steering.setText(f"Steering Output: {steering} deg")
        elif self.ros_node.current_mode == 3:  # Steer
            self.label_steer_output.setText(f"Direct Steering: {steering}")

    def closeEvent(self, event):
        """Handle window close"""
        self.update_timer.stop()
        self.get_logger_info("GUI closing...")
        event.accept()


def main():
    """Main function"""
    # Initialize ROS2
    rclpy.init()

    # Create ROS2 node
    ros_node = YawControlTestNode()

    # Create Qt application
    app = QApplication(sys.argv)

    # Create GUI
    gui = YawControlTestGUI(ros_node)
    gui.show()

    # Create timer to spin ROS2 node
    ros_timer = QTimer()

    def safe_spin():
        try:
            if rclpy.ok():
                rclpy.spin_once(ros_node, timeout_sec=0.01)
        except Exception as e:
            print(f"ROS2 spin error: {e}")

    ros_timer.timeout.connect(safe_spin)
    ros_timer.start(10)  # 100Hz

    try:
        # Run Qt application
        exit_code = app.exec_()
    finally:
        # Cleanup
        try:
            ros_timer.stop()
            ros_node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"Cleanup error: {e}")

    return exit_code


if __name__ == '__main__':
    sys.exit(main())
