# -*- coding: utf-8 -*-

# ROS2 Race GUI for Mission Control
# Converted from ROS1 to ROS2

from __future__ import print_function

import math
import os, sys
import time

print(sys.version)
import warnings
warnings.filterwarnings('ignore')

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import String, Bool, Int8, Int16, Float32
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, Pose2D, PoseStamped

from PyQt5.QtCore import *
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import *
from PyQt5 import uic
from pyqtgraph import PlotWidget
from PyQt5.QtWidgets import QWidget, QTabWidget, QAction
from PyQt5.QtGui import *
from PyQt5.QtCore import QEvent
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QGraphicsScene

import matplotlib.pyplot as plt
import numpy as np
import math

import os
script_dir = os.path.dirname(os.path.abspath(__file__))
ui_file = os.path.join(script_dir, 'race_gui.ui')
form_class = uic.loadUiType(ui_file)[0]

class ROS2RaceGUINode(Node):
    def __init__(self):
        super().__init__('ros2_race_gui_node')
        
        # QoS profile
        qos_profile = QoSProfile(depth=10)
        
        # Publishers for ROS2 mission_control_node (matching exact topic names)
        self.race_start_pub = self.create_publisher(Bool, "/flag/race_run", qos_profile)
        self.yaw_control_mode_pub = self.create_publisher(Int8, "/Car_Control_Cmd/steering_control_mode", qos_profile)
        self.lane_control_flag_pub = self.create_publisher(Bool, "/flag/lane_control_set", qos_profile)
        self.car_odom_reset_pub = self.create_publisher(Bool, "/Car_Control_Cmd/reset_odom", qos_profile)
        self.pub_traffic_sign = self.create_publisher(String, '/traffic_light', qos_profile)
        self.target_angle_pub = self.create_publisher(Float32, "/target_angle", qos_profile)
        self.stop_line_position_pub = self.create_publisher(Float32, "/stop_line_position", qos_profile)
        self.start_mission_flag_pub = self.create_publisher(Int16, "/start_mission_flag", qos_profile)
        
        # Subscribers (add topics that mission_control publishes if any)
        # Note: mission_control_node currently doesn't publish mission status, 
        # but we can add these for future use
        self.car_odom_sub = self.create_subscription(
            Float32, "/car_odom", self.car_odom_callback, qos_profile)
        
        # Mission control state
        self.car_odometry = 0.0
        self.current_mission_flag = 0
        
        self.get_logger().info('ROS2 Race GUI Node initialized')

    def current_mission_flag_callback(self, msg):
        self.current_mission_flag = msg.data
        
    def car_odom_callback(self, msg):
        self.car_odometry = msg.data

    def publish_race_start(self, start):
        msg = Bool()
        msg.data = start
        self.race_start_pub.publish(msg)
        self.get_logger().info(f'Race {"started" if start else "stopped"}')

    def publish_yaw_control_mode(self, mode):
        msg = Int8()
        msg.data = mode
        self.yaw_control_mode_pub.publish(msg)
        self.get_logger().info(f'Yaw control mode: {mode}')

    def publish_lane_control_flag(self, enable):
        msg = Bool()
        msg.data = enable
        self.lane_control_flag_pub.publish(msg)
        self.get_logger().info(f'Lane control: {"enabled" if enable else "disabled"}')

    def publish_target_angle(self, angle):
        msg = Float32()
        msg.data = float(angle)
        self.target_angle_pub.publish(msg)
        self.get_logger().info(f'Target angle: {angle} degrees')

    def publish_odom_reset(self):
        msg = Bool()
        msg.data = True
        self.car_odom_reset_pub.publish(msg)
        self.get_logger().info('Odometry reset')

    def publish_traffic_sign(self, color):
        msg = String()
        msg.data = color
        self.pub_traffic_sign.publish(msg)
        self.get_logger().info(f'Traffic light: {color}')

    def publish_start_mission_flag(self, flag):
        msg = Int16()
        msg.data = flag
        self.start_mission_flag_pub.publish(msg)
        self.get_logger().info(f'Start mission flag: {flag}')

class WindowClass(QDialog, form_class):
    
    def __init__(self, ros2_node):
        super(QDialog, self).__init__()        
        self.setupUi(self)
        
        self.ros2_node = ros2_node
        
        # GUI connections
        self.pushButton_start.clicked.connect(self.toggle_start_stop)
        self.pushButton_reset_odom.clicked.connect(self.reset_odom)
        self.lineEdit_start_mission_flag.setText("0") 
        
        # Radio button group
        self.buttonGroup = QButtonGroup(self)
        self.buttonGroup.addButton(self.radioButton_no_test)
        self.buttonGroup.addButton(self.radioButton_lane_control)
        self.buttonGroup.addButton(self.radioButton_maze_control)
        self.buttonGroup.addButton(self.radioButton_imu_control)
        self.buttonGroup.addButton(self.radioButton_steer_control)
        
        self.buttonGroup.buttonClicked.connect(self.onRadioButtonGroupClicked)
        # Set no_test as default
        self.radioButton_no_test.setChecked(True)
        
        # Check boxes
        self.checkBox_lane_control.stateChanged.connect(self.onLanecontrolStateChange)
        self.checkBox_maze_control.stateChanged.connect(self.onMazecontrolStateChange)
        self.checkBox_rotary_car_find.stateChanged.connect(self.onRotarycarcheckStateChange)
         
        # Timer for updating labels
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_timer)
        self.timer.start(200)  # Update every 200 milliseconds
        
        # Traffic light buttons
        self.pushButton_red.clicked.connect(self.pub_traffic_sign_red)
        self.pushButton_green.clicked.connect(self.pub_traffic_sign_green)
        
        # Target angle input - add line edit for target angle if exists in UI
        self.target_angle = 0.0
        
        # Add target angle control if line edit exists
        if hasattr(self, 'lineEdit_target_angle'):
            self.lineEdit_target_angle.setText("90.0")
            self.lineEdit_target_angle.textChanged.connect(self.on_target_angle_changed)
        
    def onRadioButtonGroupClicked(self, button):
        if button == self.radioButton_lane_control:
            print("Lane control button selected.")
            self.ros2_node.publish_yaw_control_mode(1)  # LANE_CONTROL
        
        elif button == self.radioButton_imu_control:
            print("IMU control button selected.")			
            self.ros2_node.publish_yaw_control_mode(0)  # IMU_CONTROL
            # Send target angle when IMU control is selected
            try:
                angle = float(self.lineEdit_target_angle.text() if hasattr(self, 'lineEdit_target_angle') else "90.0")
                self.ros2_node.publish_target_angle(angle)
            except ValueError:
                self.ros2_node.publish_target_angle(90.0)  # Default angle
            
        elif button == self.radioButton_steer_control:
            print("Steer control button selected.")
            self.ros2_node.publish_yaw_control_mode(3)  # STEER_CONTROL
            
        elif button == self.radioButton_maze_control:
            print("Maze control button selected.")
            self.ros2_node.publish_yaw_control_mode(2)  # MAZE_CONTROL
        
        elif button == self.radioButton_no_test:
            print("No test mode")	
        else:
            print("Unknown button selected.")
                            
    def onLanecontrolStateChange(self, state):
        if state == 2:  # Checkbox checked
            print("Lane Control Activated")
            self.ros2_node.publish_lane_control_flag(True)
        else:  # Checkbox unchecked
            print("Lane Control Deactivated")
            self.ros2_node.publish_lane_control_flag(False)
        
    def onMazecontrolStateChange(self, state):
        if state == 2:  # Checkbox checked
            print("Maze Control Activated")
            # Add maze control publisher if needed
        else:  # Checkbox unchecked
            print("Maze Control Deactivated")	
            
    def onRotarycarcheckStateChange(self, state):
        if state == 2:  # Checkbox checked
            print("Rotary Car Find Activated")
            # Add rotary car find publisher if needed
        else:  # Checkbox unchecked
            print("Rotary Car Find Deactivated")	
        
    def pub_traffic_sign_green(self):
        print("Green light!")
        self.ros2_node.publish_traffic_sign("Green")
        
    def pub_traffic_sign_red(self):
        print("Red light!")		
        self.ros2_node.publish_traffic_sign("Red")
            
    def reset_odom(self):
        print("Reset Odom!")	
        self.ros2_node.publish_odom_reset()

    def on_target_angle_changed(self):
        try:
            angle = float(self.lineEdit_target_angle.text())
            self.target_angle = angle
            # Publish target angle if IMU control is active
            if self.radioButton_imu_control.isChecked():
                self.ros2_node.publish_target_angle(angle)
        except ValueError:
            pass  # Invalid input, ignore

    def update_timer(self):
        # Update labels with ROS2 data
        self.label_odom.setText("Odom : %6.3f" % self.ros2_node.car_odometry)
        if hasattr(self, 'label_current_mission_flag'):
            self.label_current_mission_flag.setText("Current mission :%2d" % self.ros2_node.current_mission_flag)
        
    def start_Function(self):
        self.ros2_node.publish_race_start(True)
        self.radioButton_no_test.setChecked(True)
        try:
            mission_flag_start = int(self.lineEdit_start_mission_flag.text())
        except ValueError:
            print("Invalid input. Please enter a valid integer.")
            mission_flag_start = 0
        
        print("mission_flag_start", mission_flag_start)
        self.ros2_node.publish_start_mission_flag(mission_flag_start)
        print("start")
    
    def stop_Function(self):
        self.ros2_node.publish_race_start(False)
        self.checkBox_lane_control.setChecked(False)
        self.checkBox_maze_control.setChecked(False)
        print("stop")
    
    def toggle_start_stop(self):
        if self.pushButton_start.text() == "Start":
            self.start_Function()
            self.pushButton_start.setText("Stop")
        else:
            self.stop_Function()
            self.pushButton_start.setText("Start")
            
    def closeEvent(self, event):
        # Stop the QTimer
        self.timer.stop()
        
        # Safely shutdown ROS2 node
        try:
            if self.ros2_node is not None:
                self.ros2_node.destroy_node()
        except Exception as e:
            print(f"Error destroying ROS2 node: {e}")
        
        event.accept()   

def main():
    # Initialize ROS2
    rclpy.init()
    
    # Create ROS2 node
    ros2_node = ROS2RaceGUINode()
    
    # Create Qt application
    app = QApplication(sys.argv)
    
    # Create GUI window
    window = WindowClass(ros2_node)
    window.show()
    
    # Create a timer to spin the ROS2 node
    ros_timer = QTimer()
    
    def safe_spin():
        try:
            if rclpy.ok():
                rclpy.spin_once(ros2_node, timeout_sec=0.01)
        except Exception as e:
            print(f"ROS2 spin error: {e}")
    
    ros_timer.timeout.connect(safe_spin)
    ros_timer.start(10)  # Spin every 10ms
    
    try:
        # Run the Qt application
        app.exec_()
    finally:
        # Cleanup
        try:
            ros_timer.stop()
            if ros2_node is not None:
                ros2_node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"Cleanup error: {e}")

if __name__ == "__main__":
    main()