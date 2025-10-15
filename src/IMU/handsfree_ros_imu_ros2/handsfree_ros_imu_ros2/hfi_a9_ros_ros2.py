#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import serial
import struct
import math
import platform
import serial.tools.list_ports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32
from transforms3d.euler import euler2quat


class HandsfreeImuNode(Node):
    def __init__(self):
        super().__init__('handsfree_imu_node')
        
        # Declare parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 921600)
        self.declare_parameter('gra_normalization', True)
        self.declare_parameter('frame_id', 'imu_link')

        # Get parameters
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baud').value
        self.gra_normalization = self.get_parameter('gra_normalization').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Subscribers
        self.sub_imu_angle_offset = self.create_subscription(Float32, 'imu_angle_offset', self.imu_angle_offset_callback, 10)

        # Publishers
        self.imu_pub = self.create_publisher(Imu, 'handsfree/imu', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'handsfree/mag', 10)
        self.imu_yaw_pub = self.create_publisher(Float32, 'handsfree/imu_yaw_degree', 10)
        self.imu_yaw_correction_pub = self.create_publisher(Float32, 'handsfree/imu_yaw_correction_degree', 10)
        
        # Variables
        self.imu_angle_offset = 0.0
        self.buff = {}
        self.key = 0
        self.angle_degree = [0.0, 0.0, 0.0]
        self.magnetometer = [0.0, 0.0, 0.0] 
        self.acceleration = [0.0, 0.0, 0.0]
        self.angularVelocity = [0.0, 0.0, 0.0]
        self.pub_flag = [True, True]
        self.data_right_count = 0
        
        self.imu_msg = Imu()
        self.mag_msg = MagneticField()
        self.yaw_msg = Float32()
        self.yaw_correction_msg = Float32()

        
        # Python version
        self.python_version = platform.python_version()[0]
        
        # Find USB devices
        self.find_ttyUSB()
        
        # Open serial port
        try:
            self.hf_imu = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=0.5)
            if self.hf_imu.isOpen():
                self.get_logger().info(f"IMU serial port opened successfully: {self.port}")
            else:
                self.hf_imu.open()
                self.get_logger().info(f"IMU serial port opened: {self.port}")
        except Exception as e:
            self.get_logger().error(f"IMU serial port opening failed: {e}")
            exit(0)
            
        # Create timer for reading serial data
        self.timer = self.create_timer(0.001, self.timer_callback)  # 1000Hz
        
    def imu_angle_offset_callback(self, msg):
        """Callback for IMU angle offset subscription"""
        self.imu_angle_offset = msg.data
        self.get_logger().info(f'Received IMU angle offset: {self.imu_angle_offset}')
        
    def find_ttyUSB(self):
        """Find ttyUSB* devices"""
        self.get_logger().info('Default IMU port is /dev/ttyUSB0')
        posts = [port.device for port in serial.tools.list_ports.comports() if 'USB' in port.device]
        self.get_logger().info(f'Found {len(posts)} USB devices: {posts}')
        
    def checkSum(self, list_data, check_data):
        """CRC check"""
        data = bytearray(list_data)
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for _ in range(8):
                if (crc & 1) != 0:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return hex(((crc & 0xff) << 8) + (crc >> 8)) == hex(check_data[0] << 8 | check_data[1])
    
    def hex_to_ieee(self, raw_data):
        """16 Convert base to ieee floating point"""
        ieee_data = []
        raw_data.reverse()
        for i in range(0, len(raw_data), 4):
            data2str = hex(raw_data[i] | 0xff00)[4:6] + hex(raw_data[i + 1] | 0xff00)[4:6] + \
                      hex(raw_data[i + 2] | 0xff00)[4:6] + hex(raw_data[i + 3] | 0xff00)[4:6]
            if self.python_version == '2':
                ieee_data.append(struct.unpack('>f', data2str.decode('hex'))[0])
            if self.python_version == '3':
                ieee_data.append(struct.unpack('>f', bytes.fromhex(data2str))[0])
        ieee_data.reverse()
        return ieee_data
    
    def handleSerialData(self, raw_data):
        """Process serial data"""
        if self.data_right_count > 200000:
            self.get_logger().error("The device transmits data error, exit")
            exit(0)
            
        if self.python_version == '2':
            self.buff[self.key] = ord(raw_data)
        if self.python_version == '3':
            self.buff[self.key] = raw_data
            
        self.key += 1
        
        # Debug first few bytes
        if self.key <= 3:
            self.get_logger().debug(f"Byte {self.key}: {hex(self.buff[self.key-1])}")
            
        if self.buff[0] != 0xaa:
            self.data_right_count += 1
            self.key = 0
            return
        if self.key < 3:
            return
        if self.buff[1] != 0x55:
            self.key = 0
            return
        if self.key < self.buff[2] + 5:
            return
            
        else:
            self.data_right_count = 0
            data_buff = list(self.buff.values())
            
            if self.buff[2] == 0x2c and self.pub_flag[0]:
                if self.checkSum(data_buff[2:47], data_buff[47:49]):
                    data = self.hex_to_ieee(data_buff[7:47])
                    self.angularVelocity = data[1:4]
                    self.acceleration = data[4:7]
                    self.magnetometer = data[7:10]
                else:
                    self.get_logger().debug('Validation failed for 0x2c')
                self.pub_flag[0] = False
            elif self.buff[2] == 0x14 and self.pub_flag[1]:
                if self.checkSum(data_buff[2:23], data_buff[23:25]):
                    data = self.hex_to_ieee(data_buff[7:23])
                    self.angle_degree = data[1:4]
                else:
                    self.get_logger().debug('Validation failed for 0x14')
                self.pub_flag[1] = False
            else:
                self.get_logger().debug(f"Data type {self.buff[2]} not supported or data error")
                self.buff = {}
                self.key = 0
                return
                
            self.buff = {}
            self.key = 0
            
            self.pub_flag[0] = self.pub_flag[1] = True
            
            # Get current time
            stamp = self.get_clock().now().to_msg()
            
            # IMU message
            self.imu_msg.header.stamp = stamp
            self.imu_msg.header.frame_id = self.frame_id

            # Magnetometer message
            self.mag_msg.header.stamp = stamp
            self.mag_msg.header.frame_id = self.frame_id
            
            # Convert to quaternion - match ROS1 order
            angle_radian = [self.angle_degree[i] * math.pi / 180 for i in range(3)]
            # ROS1 uses quaternion_from_euler which returns [x,y,z,w]
            # transforms3d.euler2quat returns [w,x,y,z] so we need to adjust
            qua = euler2quat(angle_radian[0], -angle_radian[1], -angle_radian[2])
            
            self.imu_msg.orientation.x = qua[1]  # qua[1] is x
            self.imu_msg.orientation.y = qua[2]  # qua[2] is y 
            self.imu_msg.orientation.z = qua[3]  # qua[3] is z
            self.imu_msg.orientation.w = qua[0]  # qua[0] is w
            
            self.imu_msg.angular_velocity.x = self.angularVelocity[0]
            self.imu_msg.angular_velocity.y = self.angularVelocity[1]
            self.imu_msg.angular_velocity.z = self.angularVelocity[2]
            
            acc_k = math.sqrt(self.acceleration[0] ** 2 + self.acceleration[1] ** 2 + self.acceleration[2] ** 2)
            if acc_k == 0:
                acc_k = 1
                
            if self.gra_normalization:
                self.imu_msg.linear_acceleration.x = self.acceleration[0] * -9.8 / acc_k
                self.imu_msg.linear_acceleration.y = self.acceleration[1] * -9.8 / acc_k
                self.imu_msg.linear_acceleration.z = self.acceleration[2] * -9.8 / acc_k
            else:
                self.imu_msg.linear_acceleration.x = self.acceleration[0] * -9.8
                self.imu_msg.linear_acceleration.y = self.acceleration[1] * -9.8
                self.imu_msg.linear_acceleration.z = self.acceleration[2] * -9.8
                
            self.mag_msg.magnetic_field.x = self.magnetometer[0]
            self.mag_msg.magnetic_field.y = self.magnetometer[1]
            self.mag_msg.magnetic_field.z = self.magnetometer[2]
            
            # Publish yaw degree
            self.yaw_msg.data = self.angle_degree[2]
            self.yaw_correction_msg.data = self.angle_degree[2] +  self.imu_angle_offset
            
            # Publish messages
            self.imu_pub.publish(self.imu_msg)
            self.mag_pub.publish(self.mag_msg)
            self.imu_yaw_pub.publish(self.yaw_msg)
            self.imu_yaw_correction_pub.publish(self.yaw_correction_msg)
            
            # Debug log - print once every 100 messages
            if not hasattr(self, 'msg_count'):
                self.msg_count = 0
            self.msg_count += 1
            if self.msg_count % 100 == 0:
                self.get_logger().info(f"Published {self.msg_count} IMU messages. Yaw: {self.angle_degree[2]:.2f}°")
            
    def timer_callback(self):
        """Timer callback to read serial data"""
        try:
            # Read one byte first
            byte = self.hf_imu.read(1)
            
            # Read remaining bytes if available
            buff_count = self.hf_imu.in_waiting
            
            if byte:
                self.handleSerialData(byte[0])
            
            if buff_count > 0:
                buff_data = self.hf_imu.read(buff_count)
                for i in range(buff_count):
                    self.handleSerialData(buff_data[i])
        except Exception as e:
            # Only log if it's a real error, not just no data
            if "device reports readiness" not in str(e):
                self.get_logger().debug(f"Serial read: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    imu_node = HandsfreeImuNode()
    
    try:
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        pass
    finally:
        imu_node.hf_imu.close()
        imu_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()