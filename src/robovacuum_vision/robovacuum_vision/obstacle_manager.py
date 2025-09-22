#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
import numpy as np

class ObstacleManager(Node):
    """
    Combines YOLO detections with navigation to avoid obstacles
    """
    def __init__(self):
        super().__init__('obstacle_manager')
        
        # Subscribe to YOLO detections
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/yolo/detections',
            self.detection_callback,
            10
        )
        
        # Subscribe to cmd_vel to monitor robot motion
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/robovacuum_controller/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publish obstacle points for costmap
        self.obstacle_pub = self.create_publisher(
            PointCloud2,
            '/obstacle_cloud',
            10
        )
        
        # Safety override publisher
        self.safety_cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel_safety',
            10
        )
        
        self.current_cmd = Twist()
        self.obstacles_detected = []
        
        self.get_logger().info('Obstacle Manager initialized!')

    def detection_callback(self, msg):
        """Process YOLO detections and identify obstacles"""
        self.obstacles_detected = []
        
        for detection in msg.detections:
            bbox = detection.bbox
            
            # Calculate distance estimate (simplified)
            # Assumes known camera parameters
            image_height = 480
            bbox_bottom = bbox.center.y + bbox.size_y / 2
            
            # Simple ground plane assumption
            # Objects touching bottom of image are closer
            distance_estimate = (image_height - bbox_bottom) / image_height * 3.0  # Max 3 meters
            
            # If object is close, mark as obstacle
            if distance_estimate < 1.0:  # Within 1 meter
                self.obstacles_detected.append({
                    'distance': distance_estimate,
                    'angle': (bbox.center.x - 320) / 320 * 30  # +/- 30 degrees FOV
                })
                
                self.get_logger().warn(
                    f'Obstacle detected at {distance_estimate:.2f}m, '
                    f'angle: {self.obstacles_detected[-1]["angle"]:.1f} degrees'
                )
        
        # Apply safety measures if needed
        self.apply_safety_override()

    def cmd_vel_callback(self, msg):
        """Monitor commanded velocity"""
        self.current_cmd = msg

    def apply_safety_override(self):
        """Override velocity commands if obstacles detected"""
        if not self.obstacles_detected:
            return
            
        # Check if any obstacle is in the path
        for obstacle in self.obstacles_detected:
            if abs(obstacle['angle']) < 15 and obstacle['distance'] < 0.5:
                # Emergency stop
                safety_cmd = Twist()
                safety_cmd.linear.x = 0.0
                safety_cmd.angular.z = 0.0
                self.safety_cmd_pub.publish(safety_cmd)
                self.get_logger().error('EMERGENCY STOP: Obstacle too close!')
                return
            elif abs(obstacle['angle']) < 30 and obstacle['distance'] < 1.0:
                # Slow down
                safety_cmd = Twist()
                safety_cmd.linear.x = self.current_cmd.linear.x * 0.3
                safety_cmd.angular.z = self.current_cmd.angular.z
                self.safety_cmd_pub.publish(safety_cmd)
                self.get_logger().warn('Slowing down due to obstacle')

def main(args=None):
    rclpy.init(args=args)
    manager = ObstacleManager()
    
    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        pass
    finally:
        manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()