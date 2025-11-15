#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
import subprocess
import sys

class SimpleResetService(Node):
    """
    Simple map reset service that uses subprocess to call SLAM Toolbox service
    """
    def __init__(self):
        super().__init__('simple_reset_service')
        
        # Create reset service (like Hector SLAM)
        self.reset_service = self.create_service(
            Empty, 
            '/reset_map', 
            self.reset_map_callback
        )
        
        self.get_logger().info("Simple Reset Service started")
        self.get_logger().info("Use: ros2 service call /reset_map std_srvs/srv/Empty")
        
    def reset_map_callback(self, request, response):
        """
        Reset the current map using subprocess call
        """
        self.get_logger().info("=== MAP RESET REQUESTED ===")
        
        try:
            # Use subprocess to call the SLAM Toolbox service
            result = subprocess.run([
                'ros2', 'service', 'call', 
                '/slam_toolbox/clear_changes', 
                'slam_toolbox/srv/Clear'
            ], 
            capture_output=True, 
            text=True, 
            timeout=10
            )
            
            if result.returncode == 0:
                self.get_logger().info("✅ Map reset successful!")
                self.get_logger().info("   - Current mapping data cleared")
                self.get_logger().info("   - Ready for new mapping session")
            else:
                self.get_logger().error("❌ Map reset failed!")
                self.get_logger().error(f"   Error: {result.stderr}")
                
        except subprocess.TimeoutExpired:
            self.get_logger().warn("⚠️  Service call timeout (but likely successful)")
        except Exception as e:
            self.get_logger().error(f"Map reset error: {str(e)}")
            
        return response

def main(args=None):
    rclpy.init(args=args)
    
    reset_service = SimpleResetService()
    
    try:
        rclpy.spin(reset_service)
    except KeyboardInterrupt:
        pass
    finally:
        reset_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()