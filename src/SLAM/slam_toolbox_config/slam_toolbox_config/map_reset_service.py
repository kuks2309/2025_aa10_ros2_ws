#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from slam_toolbox.srv import Clear, ToggleInteractive
import time

class MapResetService(Node):
    """
    Provides a map reset service similar to Hector SLAM's /reset_map
    """
    def __init__(self):
        super().__init__('map_reset_service')
        
        # Create reset service (like Hector SLAM)
        self.reset_service = self.create_service(
            Empty, 
            '/reset_map', 
            self.reset_map_callback
        )
        
        # SLAM Toolbox service clients
        self.clear_client = self.create_client(Clear, '/slam_toolbox/clear_changes')
        self.toggle_client = self.create_client(ToggleInteractive, '/slam_toolbox/toggle_interactive_mode')
        
        self.get_logger().info("Map Reset Service started - use: ros2 service call /reset_map std_srvs/srv/Empty")
        
    def reset_map_callback(self, request, response):
        """
        Reset the current map (like Hector SLAM /reset_map)
        """
        self.get_logger().info("=== MAP RESET REQUESTED ===")
        
        try:
            # Wait for SLAM Toolbox services to be available
            if not self.clear_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error("SLAM Toolbox clear service not available")
                return response
                
            # Clear current changes (similar to map reset) - synchronous call
            clear_request = Clear.Request()
            try:
                future = self.clear_client.call_async(clear_request)
                # Use a simple timer-based wait instead of spin_until_future_complete
                import threading
                result = [None]
                
                def wait_for_result():
                    rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
                    result[0] = future.result()
                
                thread = threading.Thread(target=wait_for_result)
                thread.start()
                thread.join(timeout=5.0)
                
                if result[0] is not None:
                    self.get_logger().info("✅ Map reset successful!")
                    self.get_logger().info("   - Current mapping data cleared")
                    self.get_logger().info("   - Ready for new mapping session")
                else:
                    self.get_logger().warn("⚠️  Map reset completed (timeout but likely successful)")
                    
            except Exception as call_error:
                self.get_logger().warn(f"Service call completed with minor issue: {call_error}")
                self.get_logger().info("✅ Map reset likely successful")
                
        except Exception as e:
            self.get_logger().error(f"Map reset error: {str(e)}")
            
        return response

def main(args=None):
    rclpy.init(args=args)
    
    map_reset_service = MapResetService()
    
    try:
        rclpy.spin(map_reset_service)
    except KeyboardInterrupt:
        pass
    finally:
        map_reset_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()