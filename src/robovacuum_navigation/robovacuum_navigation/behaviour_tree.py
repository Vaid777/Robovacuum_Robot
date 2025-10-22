#!/usr/bin/env python3
"""
Step 5: Behavior Tree Framework
Justifies: Bullet 5 (Complete robotics stack with behavior trees)

Implements:
- Behavior tree structure (composite, action, condition nodes)
- Decision-making logic for autonomous robots
- Hierarchical behavior organization
- Event-driven architecture
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
import enum
from collections import deque


class NodeStatus(enum.Enum):
    """Behavior tree node status."""
    SUCCESS = 1
    FAILURE = 2
    RUNNING = 3


class TreeNode:
    """Base behavior tree node."""
    
    def __init__(self, name):
        self.name = name
        self.status = NodeStatus.RUNNING
    
    def tick(self, context):
        """Execute node. Override in subclasses."""
        pass


class ConditionNode(TreeNode):
    """Condition node - evaluates to SUCCESS or FAILURE."""
    
    def __init__(self, name, condition_func):
        super().__init__(name)
        self.condition_func = condition_func
    
    def tick(self, context):
        if self.condition_func(context):
            self.status = NodeStatus.SUCCESS
        else:
            self.status = NodeStatus.FAILURE
        return self.status


class ActionNode(TreeNode):
    """Action node - performs an action."""
    
    def __init__(self, name, action_func):
        super().__init__(name)
        self.action_func = action_func
    
    def tick(self, context):
        self.status = self.action_func(context)
        return self.status


class SequenceNode(TreeNode):
    """Sequence node - all children must succeed."""
    
    def __init__(self, name, children):
        super().__init__(name)
        self.children = children
    
    def tick(self, context):
        for child in self.children:
            status = child.tick(context)
            if status != NodeStatus.SUCCESS:
                self.status = status
                return self.status
        
        self.status = NodeStatus.SUCCESS
        return self.status


class SelectorNode(TreeNode):
    """Selector node - first succeeding child wins."""
    
    def __init__(self, name, children):
        super().__init__(name)
        self.children = children
    
    def tick(self, context):
        for child in self.children:
            status = child.tick(context)
            if status == NodeStatus.SUCCESS:
                self.status = NodeStatus.SUCCESS
                return self.status
        
        self.status = NodeStatus.FAILURE
        return self.status


class BehaviorTreeNode(Node):
    """ROS2 node running behavior tree."""
    
    def __init__(self):
        super().__init__('behavior_tree_node')
        
        self.get_logger().info("="*60)
        self.get_logger().info("STEP 5: BEHAVIOR TREE")
        self.get_logger().info("="*60)
        self.get_logger().info("Justifies:")
        self.get_logger().info("  ✓ Bullet 5: Autonomous behavior framework")
        self.get_logger().info("  ✓ Bullet 5: Complete robotics stack")
        self.get_logger().info("="*60)
        
        # ============ CONTEXT DATA ============
        self.context = {
            'obstacle_detected': False,
            'at_goal': False,
            'battery_low': False,
            'coverage_complete': False,
            'is_cleaning': False
        }
        
        # ============ QOS ============
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # ============ SUBSCRIBERS ============
        self.obstacle_sub = self.create_subscription(
            Bool, 'sensors/obstacle_detected',
            self.obstacle_callback, sensor_qos
        )
        
        # ============ PUBLISHERS ============
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.behavior_state_pub = self.create_publisher(String, 'behavior/state', 10)
        
        # ============ BUILD BEHAVIOR TREE ============
        self.tree = self.build_tree()
        
        # ============ TIMER ============
        self.create_timer(0.1, self.tick_callback)
        self.behavior_history = deque(maxlen=100)
        
        self.get_logger().info(
            f"\nBehavior Tree Structure:\n"
            f"  Root: Autonomous Vacuum\n"
            f"    ├─ Sequence: If_Active\n"
            f"    │   ├─ Condition: Battery_OK\n"
            f"    │   └─ Selector: Main_Behavior\n"
            f"    │       ├─ Sequence: Avoid_Obstacle\n"
            f"    │       │   ├─ Condition: Obstacle_Detected\n"
            f"    │       │   └─ Action: Rotate_Around\n"
            f"    │       ├─ Sequence: Navigate_to_Goal\n"
            f"    │       │   ├─ Condition: Goal_Not_Reached\n"
            f"    │       │   └─ Action: Move_Forward\n"
            f"    │       └─ Action: Coverage_Cleaning\n"
        )
    
    def build_tree(self):
        """Build the behavior tree structure."""
        
        # ============ LEAF NODES (CONDITIONS) ============
        battery_ok = ConditionNode(
            "Battery_OK",
            lambda ctx: not ctx['battery_low']
        )
        
        obstacle_detected = ConditionNode(
            "Obstacle_Detected",
            lambda ctx: ctx['obstacle_detected']
        )
        
        goal_not_reached = ConditionNode(
            "Goal_Not_Reached",
            lambda ctx: not ctx['at_goal']
        )
        
        # ============ LEAF NODES (ACTIONS) ============
        def rotate_around(ctx):
            """Action: Rotate to avoid obstacle."""
            msg = Twist()
            msg.angular.z = 0.5  # Rotate
            self.cmd_vel_pub.publish(msg)
            return NodeStatus.SUCCESS
        
        def move_forward(ctx):
            """Action: Move towards goal."""
            msg = Twist()
            msg.linear.x = 0.2  # Move forward
            self.cmd_vel_pub.publish(msg)
            return NodeStatus.SUCCESS
        
        def coverage_cleaning(ctx):
            """Action: Execute coverage pattern."""
            msg = Twist()
            msg.linear.x = 0.1  # Slow cleaning speed
            self.cmd_vel_pub.publish(msg)
            ctx['is_cleaning'] = True
            return NodeStatus.RUNNING
        
        rotate_action = ActionNode("Rotate_Around", rotate_around)
        move_action = ActionNode("Move_Forward", move_forward)
        clean_action = ActionNode("Coverage_Cleaning", coverage_cleaning)
        
        # ============ COMPOSITE NODES ============
        # Obstacle avoidance sequence
        avoid_sequence = SequenceNode(
            "Avoid_Obstacle",
            [obstacle_detected, rotate_action]
        )
        
        # Navigation sequence
        navigate_sequence = SequenceNode(
            "Navigate_to_Goal",
            [goal_not_reached, move_action]
        )
        
        # Main behavior selector (priority: avoid > navigate > clean)
        main_selector = SelectorNode(
            "Main_Behavior",
            [avoid_sequence, navigate_sequence, clean_action]
        )
        
        # Active check sequence
        active_sequence = SequenceNode(
            "If_Active",
            [battery_ok, main_selector]
        )
        
        return active_sequence
    
    def obstacle_callback(self, msg: Bool):
        """Update obstacle status."""
        self.context['obstacle_detected'] = msg.data
    
    def tick_callback(self):
        """Execute behavior tree tick."""
        
        # Execute tree
        status = self.tree.tick(self.context)
        
        # Publish current behavior
        state_msg = String()
        state_msg.data = self.get_tree_status()
        self.behavior_state_pub.publish(state_msg)
        
        self.behavior_history.append(state_msg.data)
    
    def get_tree_status(self):
        """Get current tree execution status."""
        return f"Battery: {'OK' if not self.context['battery_low'] else 'LOW'}, " \
               f"Obstacle: {self.context['obstacle_detected']}, " \
               f"Cleaning: {self.context['is_cleaning']}"


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorTreeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()