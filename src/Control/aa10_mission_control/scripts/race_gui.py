# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'car_parking_gui.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!


from __future__ import print_function

import math
import os, sys
import rospy
import time

print(sys.version)
import warnings
warnings.filterwarnings('ignore')

from std_msgs.msg       import String
from std_msgs.msg       import Bool
from std_msgs.msg       import Int8
from std_msgs.msg       import Int16
from std_msgs.msg       import Float32
from sensor_msgs.msg    import NavSatFix
from nav_msgs.msg       import Odometry, OccupancyGrid, Odometry
from geometry_msgs.msg  import Point, Pose, Quaternion, Twist, Vector3, Pose2D, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from PyQt5.QtCore       import *
from PyQt5.QtCore       import Qt
from PyQt5.QtWidgets    import *
from PyQt5              import uic
from pyqtgraph          import PlotWidget
from PyQt5.QtWidgets    import QWidget, QTabWidget, QAction
from PyQt5.QtGui        import *
from PyQt5.QtCore 	    import QEvent
from PyQt5.QtGui 		import QImage, QPixmap
from PyQt5.QtWidgets 	import QGraphicsScene


import matplotlib.pyplot as plt

import numpy as np
import math

        
form_class = uic.loadUiType('race_gui.ui')[0]


class WindowClass(QDialog, form_class):
	
	car_odometery = 0.0
	
	def __init__(self):
		
		super(QDialog, self).__init__()        
		rospy.init_node('ROS_race_gui', anonymous=True)
		self.setupUi(self)
		
		# GUI connection
		
		self.pushButton_start.clicked.connect(self.toggle_start_stop)
		self.pushButton_reset_odom.clicked.connect(self.reset_odom)
		self.lineEdit_start_mission_flag.setText("0") 
		
		#radio button
		self.buttonGroup = QButtonGroup(self)
		self.buttonGroup.addButton(self.radioButton_no_test)
		self.buttonGroup.addButton(self.radioButton_lane_control)
		self.buttonGroup.addButton(self.radioButton_maze_control)
		self.buttonGroup.addButton(self.radioButton_imu_control)
		self.buttonGroup.addButton(self.radioButton_steer_control)
		
		self.buttonGroup.buttonClicked.connect(self.onRadioButtonGroupClicked)
		# no_test 라디오 버튼을 기본으로 선택
		self.radioButton_no_test.setChecked(True)
		
		# check box 
		
		self.checkBox_lane_control.stateChanged.connect(self.onLanecontrolStateChange)
		self.checkBox_maze_control.stateChanged.connect(self.onMazecontrolStateChange)
		self.checkBox_rotary_car_find.stateChanged.connect(self.onRotarycarcheckStateChange)
		 
		# Timer for updating labels
		self.timer = QTimer(self)
		self.timer.timeout.connect(self.update_timer)
		self.timer.start(200)  # Update every 200 milliseconds
		
		self.pose_data = None
		self.yaw_data  = None
		
		# Publisher for start_calibration topic	
		self.race_start_pub 				= rospy.Publisher("/flag/race_run", Bool, queue_size=10)
		
		# Publisher for maze escape run flag topic 
		
		self.maze_escape_control_enable_topic     = "/flag/maze_control_set"
		self.maze_control_run_flag_pub      = rospy.Publisher(self.maze_escape_control_enable_topic, Bool, queue_size=10)
		
		# Publisher for car odom reset
		self.car_odom_reset_topic           = "/Car_Control_Cmd/reset_odom";
		self.car_odom_reset_pub 			= rospy.Publisher(self.car_odom_reset_topic, Bool, queue_size=10)
		
		# Publisher for traffic light
		
		self.pub_traffic_sign = rospy.Publisher('/traffic_light', String, queue_size=1)	
		self.pushButton_red.clicked.connect(self.pub_traffic_sign_red)
		self.pushButton_green.clicked.connect(self.pub_traffic_sign_green)
				
		# Publisher for start mission flag
		self.start_mission_flag_topic       = "/mission_flag/start";
		self.start_mission_flag_pub 		= rospy.Publisher(self.start_mission_flag_topic, Int16, queue_size=10)
		
		# Publisher for yaw_control_mode
		self.yaw_control_mode_topic          = "/Car_Control_Cmd/steering_control_mode"
		self.yaw_control_mode_pub            = rospy.Publisher(self.yaw_control_mode_topic,Int8, queue_size=10)		
		
		# Publisher for lidar rotary car detect_mode
		self.lidar_detect_control_flag_topic      = "/flag/lidar_detect_set"
		self.lidar_detect_control_flag_pub            = rospy.Publisher(self.lidar_detect_control_flag_topic,Bool, queue_size=10)		
			
		
		# Subscribe for current mission_flag
		self.current_mission_flag_pose_sub   = rospy.Subscriber("/current_missiong_flag", Int16, self.current_mission_flag_callback)	
	
		# Subscribe for car_odom
		self.car_odom_topic                  = "/car_odom";
		self.car_odom_sub                    = rospy.Subscriber(self.car_odom_topic,Float32, self.car_odom_callback)		
		
		
	
	def onRadioButtonGroupClicked(self, button):
		if button == self.radioButton_lane_control:
			print("차선 제어(lane control) 버튼이 선택되었습니다.")
						# 차선 제어 관련 코드를 여기에 추가
			self.yaw_control_mode_pub.publish(Int8(data=1))
		
		elif button == self.radioButton_imu_control:
			print("IMU 제어 버튼이 선택되었습니다.")			
			# IMU 제어 관련 코드를 여기에 추가
			self.yaw_control_mode_pub.publish(Int8(data=0))
			
		elif button == self.radioButton_steer_control:
			print("조향 제어(steer control) 버튼이 선택되었습니다.")
			# 조향 제어 관련 코드를 여기에 추가
			self.yaw_control_mode_pub.publish(Int8(data=3))
			
		
		elif button == self.radioButton_maze_control:
			print("미로 제어(maze control) 버튼이 선택되었습니다.")
			# 미로 제어 관련 코드를 여기에 추가
			self.yaw_control_mode_pub.publish(Int8(data=2))
		
		
		elif button == self.radioButton_no_test:
			print("test mode가 아닙니다")	
		else:
			print("알 수 없는 버튼이 선택되었습니다.")
							
	
	def onLanecontrolStateChange(self, state):
		if state == 2:  # 체크박스가 선택되었을 때
			print("Lane Control Activated")
			# 여기에 차선 제어를 활성화하는 코드를 추가할 수 있습니다.
			
		else:  # 체크박스가 선택 해제되었을 때
			print("Lane Control Deactivated")
		
	def onMazecontrolStateChange(self, state):
		if state == 2:  # 체크박스가 선택되었을 때
			print("Maze Control Activated")
			# 여기에 미로 제어를 활성화하는 코드를 추가할 수 있습니다.
			self.maze_control_run_flag_pub.publish(Bool(data = True))
		else:  # 체크박스가 선택 해제되었을 때
			self.maze_control_run_flag_pub.publish(Bool(data = False))
			print("Maze Control Deactivated")	
			
	def onRotarycarcheckStateChange(self,state):
		if state == 2:  # 체크박스가 선택되었을 때
			print("Rotary Car Find Activated")
			# 여기에 미로 제어를 활성화하는 코드를 추가할 수 있습니다.
			self.lidar_detect_control_flag_pub .publish(Bool(data = True))
		else:  # 체크박스가 선택 해제되었을 때
			print("Rotary Car Find  Deactivated")	
			self.lidar_detect_control_flag_pub .publish(Bool(data = False))	
		
            
            
            
	def pub_traffic_sign_green(self):
		print("Green light!")
		self.pub_traffic_sign.publish(String(data="Green"))
		
	def pub_traffic_sign_red(self):
		print("Red light!")		
		self.pub_traffic_sign.publish(String(data="Red"))
			
	def reset_odom(self):
		print("Reset Odom!")	
		self.car_odom_reset_pub.publish(Bool(data=True))
		
	def car_odom_callback(self,car_odom_msg):
		self.car_odometery = car_odom_msg.data
		self.label_odom.setText("Odom : %6.3f" % car_odom_msg.data)
		None
			

	def yaw_control_mode_callback(self, yaw_control_mode):
		#self.
	
		None
		
	def current_mission_flag_callback(self, mission_flag):
		
		self.label_current_mission_flag.setText("Current mission :%2d" % mission_flag.data)
		#self.label_current_mission_flag.setText("Current mission: {}".format(mission_flag.data))
		
	def update_timer(self):
		None
		
	def yaw_pose_callback(self, data):
		if not hasattr(self, 'compass_widget_yaw'):
			return    
		yaw = data.data
		self.compass_widget_yaw.update_angle(math.degrees(yaw))
	
	def start_Function(self):
		self.race_start_pub.publish(Bool(data=True)) 
		self.radioButton_no_test.setChecked(True)
		try:
			mission_flag_start = int(self.lineEdit_start_mission_flag.text())
		except ValueError:
			print("Invalid input. Please enter a valid integer.")
			mission_flag_start = 0  # 또는 다른 기본값
		
		print("mission_flag_start", mission_flag_start)
		self.start_mission_flag_pub.publish(Int16(data=mission_flag_start))
		print("start")
	
	def stop_Function(self):
		self.race_start_pub.publish(Bool(data=False))
		
		self.checkBox_lane_control.setChecked(False)
		
		
		self.checkBox_maze_control.setChecked(False)
		self.maze_control_run_flag_pub.publish(Bool(data = False))
		print("stop")
	
	def toggle_start_stop(self):
		if self.pushButton_start.text() == "Start":
			self.start_Function()
			self.pushButton_start.setText("Stop")
		else:
			self.stop_Function()
			self.pushButton_start.setText("Start")
			
	
	def closeEvent(self, event):
		# Unsubscribe from ROS topics to prevent callbacks after the widget is deleted
		# Unsubscribe from ROS topics to prevent callbacks after the widget is deleted
		
		# Stop the QTimer
		self.timer.stop()
		
		# Destroy the compass widgets
	
		event.accept()   
      
if __name__ == "__main__":
	app = QApplication(sys.argv)
	myWindow = WindowClass()
	myWindow.show()
	app.exec_()
