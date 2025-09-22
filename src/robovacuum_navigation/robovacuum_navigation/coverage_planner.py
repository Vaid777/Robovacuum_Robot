#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid, Path
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np
from scipy.spatial import distance
import math

class CoveragePlanner(Node):
    """
    Simple zigzag coverage pattern for vacuum cleaning
    """
    def __init__(self):
        super().__init__('coverage_planner')
        
        # Subscribe to map
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        # Action client for navigation
        self.nav_client = ActionClient(
            self, 
            NavigateToPose, 
            'navigate_to_pose'
        )
        
        # Publishers for visualization
        self.path_pub = self.create_publisher(
            Path,
            '/coverage_path',
            10
        )
        
        self.map_data = None
        self.coverage_waypoints = []
        self.current_waypoint = 0
        self.robot_width = 0.35  # meters
        
        # Start coverage timer
        self.timer = self.create_timer(1.0, self.execute_coverage)
        
        self.get_logger().info('Coverage Planner initialized!')

    def map_callback(self, msg):
        """Process occupancy grid and plan coverage path"""
        self.map_data = msg
        
        # Only plan once when map is received
        if not self.coverage_waypoints:
            self.plan_coverage()

    def plan_coverage(self):
        """Generate simple zigzag coverage pattern"""
        if not self.map_data:
            return
            
        # Convert occupancy grid to numpy array
        width = self.map_data.info.width
        height = self.map_data.info.height
        resolution = self.map_data.info.resolution
        
        grid = np.array(self.map_data.data).reshape((height, width))
        
        # Find free space (0 = free, 100 = occupied, -1 = unknown)
        free_space = grid == 0
        
        # Simple boustrophedon (zigzag) pattern
        stripe_width = int(self.robot_width / resolution)
        
        waypoints = []
        direction = 1  # 1 = forward, -1 = backward
        
        for x in range(0, width, stripe_width):
            if direction == 1:
                # Move forward along y
                for y in range(0, height, stripe_width * 2):
                    if x < width and y < height and free_space[y, x]:
                        # Convert grid coordinates to world coordinates
                        world_x = x * resolution + self.map_data.info.origin.position.x
                        world_y = y * resolution + self.map_data.info.origin.position.y
                        waypoints.append((world_x, world_y))
            else:
                # Move backward along y
                for y in range(height-1, -1, -stripe_width * 2):
                    if x < width and y < height and free_space[y, x]:
                        world_x = x * resolution + self.map_data.info.origin.position.x
                        world_y = y * resolution + self.map_data.info.origin.position.y
                        waypoints.append((world_x, world_y))
            
            direction *= -1
        
        self.coverage_waypoints = waypoints
        self.get_logger().info(f'Generated {len(waypoints)} coverage waypoints')
        
        # Publish path for visualization
        self.publish_coverage_path()

    def publish_coverage_path(self):
        """Publish the coverage path for visualization in RViz"""
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        for wx, wy in self.coverage_waypoints:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        
        self.path_pub.publish(path)

    def execute_coverage(self):
        """Execute the coverage plan by sending navigation goals"""
        if not self.coverage_waypoints:
            return
            
        if self.current_waypoint >= len(self.coverage_waypoints):
            self.get_logger().info('Coverage complete!')
            return
        
        # Check if action server is available
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Navigation action server not available')
            return
        
        # Send next waypoint as goal
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        
        wx, wy = self.coverage_waypoints[self.current_waypoint]
        goal.pose.pose.position.x = wx
        goal.pose.pose.position.y = wy
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.w = 1.0
        
        # Send goal
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self.get_logger().info(f'Navigating to waypoint {self.current_waypoint}')
        
        # Get result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Reached waypoint {self.current_waypoint}')
            self.current_waypoint += 1
        else:
            self.get_logger().warn(f'Failed to reach waypoint {self.current_waypoint}')
            # Skip to next waypoint
            self.current_waypoint += 1

def main(args=None):
    rclpy.init(args=args)
    planner = CoveragePlanner()
    
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()