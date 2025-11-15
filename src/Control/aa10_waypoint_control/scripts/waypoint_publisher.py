#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import sys
import math


class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')

        # Publisher
        self.path_pub = self.create_publisher(Path, '/waypoint_path', 10)

        # Subscribers
        self.current_pose_sub = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.current_pose_callback,
            10
        )

        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10
        )

        # Optional: Subscribe to odometry as alternative to current_pose
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # State variables
        self.current_pose = None
        self.goal_pose = None
        self.waypoints = []
        self.path_generated = False

        # Parameters
        self.declare_parameter('num_intermediate_points', 10)
        self.declare_parameter('use_straight_line', True)
        self.num_intermediate_points = self.get_parameter('num_intermediate_points').value
        self.use_straight_line = self.get_parameter('use_straight_line').value

        self.get_logger().info('Waypoint Publisher Node Started')
        self.get_logger().info('Waiting for current_pose and goal_pose topics...')
        self.get_logger().info('Subscribe to: /current_pose and /goal_pose')

    def current_pose_callback(self, msg):
        """Callback for current pose"""
        self.current_pose = msg
        if self.goal_pose is not None and not self.path_generated:
            self.generate_path()

    def goal_pose_callback(self, msg):
        """Callback for goal pose"""
        self.goal_pose = msg
        self.path_generated = False
        self.get_logger().info(f'Received goal pose: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}')
        if self.current_pose is not None:
            self.generate_path()

    def odom_callback(self, msg):
        """Callback for odometry (alternative to current_pose)"""
        if self.current_pose is None:
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = msg.pose.pose
            self.current_pose = pose_stamped

    def generate_path(self):
        """Generate path from current pose to goal pose"""
        if self.current_pose is None or self.goal_pose is None:
            return

        self.waypoints.clear()

        # Extract positions
        x_start = self.current_pose.pose.position.x
        y_start = self.current_pose.pose.position.y
        x_goal = self.goal_pose.pose.position.x
        y_goal = self.goal_pose.pose.position.y

        self.get_logger().info(f'Generating path from ({x_start:.2f}, {y_start:.2f}) to ({x_goal:.2f}, {y_goal:.2f})')

        if self.use_straight_line:
            # Generate straight line path with intermediate points
            for i in range(self.num_intermediate_points + 1):
                t = i / self.num_intermediate_points
                x = x_start + t * (x_goal - x_start)
                y = y_start + t * (y_goal - y_start)

                # Calculate heading towards goal
                theta = math.atan2(y_goal - y, x_goal - x)

                self.waypoints.append((x, y, theta))
        else:
            # Just use start and goal
            theta_start = self.get_yaw_from_quaternion(self.current_pose.pose.orientation)
            theta_goal = self.get_yaw_from_quaternion(self.goal_pose.pose.orientation)
            self.waypoints.append((x_start, y_start, theta_start))
            self.waypoints.append((x_goal, y_goal, theta_goal))

        self.path_generated = True
        self.get_logger().info(f'Path generated with {len(self.waypoints)} waypoints')
        self.publish_path()

    def get_yaw_from_quaternion(self, quaternion):
        """Extract yaw angle from quaternion"""
        # quaternion = (x, y, z, w)
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def add_waypoint(self, x, y, theta=0.0):
        """Add a waypoint to the list"""
        self.waypoints.append((x, y, theta))
        self.get_logger().info(f'Added waypoint: x={x:.2f}, y={y:.2f}, theta={theta:.2f}')

    def clear_waypoints(self):
        """Clear all waypoints"""
        self.waypoints.clear()
        self.get_logger().info('Cleared all waypoints')

    def load_waypoints_from_file(self, filename):
        """Load waypoints from a file"""
        try:
            with open(filename, 'r') as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith('#'):
                        parts = line.split()
                        if len(parts) >= 2:
                            x = float(parts[0])
                            y = float(parts[1])
                            theta = float(parts[2]) if len(parts) > 2 else 0.0
                            self.add_waypoint(x, y, theta)
            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from {filename}')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {str(e)}')
            return False

    def publish_path(self):
        """Publish path message"""
        if not self.waypoints:
            return

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for x, y, theta in self.waypoints:
            pose = PoseStamped()
            pose.header.stamp = path_msg.header.stamp
            pose.header.frame_id = 'map'

            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # Convert theta to quaternion
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(theta / 2.0)
            pose.pose.orientation.w = math.cos(theta / 2.0)

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Published path with {len(self.waypoints)} waypoints', throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)

    node = WaypointPublisher()

    # Check if waypoint file is provided for manual waypoints
    if len(sys.argv) > 1:
        waypoint_file = sys.argv[1]
        node.get_logger().info(f'Loading waypoints from file: {waypoint_file}')
        if node.load_waypoints_from_file(waypoint_file):
            # Publish manually loaded waypoints
            node.path_generated = True
            node.publish_path()
    else:
        node.get_logger().info('No waypoint file provided')
        node.get_logger().info('Waiting for /current_pose and /goal_pose topics to generate path automatically')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
