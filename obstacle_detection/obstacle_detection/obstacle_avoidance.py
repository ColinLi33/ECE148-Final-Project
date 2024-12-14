import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import Twist
import numpy as np
import yaml
import os
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
from ament_index_python.packages import get_package_share_directory
from multiprocessing import Process, Queue

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        # Load configuration
        package_share_directory = get_package_share_directory('obstacle_detection')
        config_path = os.path.join(package_share_directory, 'config', 'obstacle_avoidance.yaml')

        with open(config_path, 'r') as file:
            self.config = yaml.safe_load(file)

        # Parameters from config
        self.viewing_angle = self.config.get('viewing_angle', 85.0)
        self.avoidance_enabled = self.config.get('avoidance', True)
        self.critical_distance = self.config.get('critical_distance', 3.0)
        self.warning_distance = self.config.get('warning_distance', 10.0)
        self.grouping_angle = self.config.get('grouping_angle',12.0)
        self.view_res = self.config.get('view_resolution',15)

        # Subscribers
        self.detection_sub = self.create_subscription(
            Float32MultiArray,
            '/object_detection',
            self.detection_callback,
            10
        )
        self.get_logger().info("Subscribed to /object_detection topic")

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.avoidance_state_pub = self.create_publisher(Bool, '/avoidance_state', 10)

        self.create_timer(5.0, self.subscription_check)

        # State variables
        self.is_avoiding = False
        self.obstacles = []

        self.get_logger().info("Obstacle Avoidance Node Initialized")

    def subscription_check(self):
        """Periodically log subscription status"""
        self.get_logger().warn("No obstacle detection messages received")

    def detection_callback(self, msg):
        self.destroy_timer(self.subscription_check)  # all is good.
        # self.get_logger().info(f"Received obstacle detection: {msg.data}")
        # Parse obstacle detections
        self.obstacles = []
        for i in range(0, len(msg.data), 3):
            x, y, distance = msg.data[i:i+3]
            self.obstacles.append((x, distance))

        # Check for avoidance conditions
        self.check_avoidance_state()

        # Perform obstacle avoidance if needed
        if self.is_avoiding:
            self.avoid_obj()

    def check_avoidance_state(self):
        # Determine if obstacles are within critical or warning distance
        avoidance_needed = any(
            obs[1] < self.critical_distance or obs[1] < self.warning_distance
            for obs in self.obstacles
        )

        # Update avoidance state
        if avoidance_needed and not self.is_avoiding:
            self.is_avoiding = True
            self.publish_avoidance_state(True)
            self.get_logger().info("Entering Obstacle Avoidance Mode")
        elif not avoidance_needed and self.is_avoiding:
            self.is_avoiding = False
            self.publish_avoidance_state(False)
            self.get_logger().info("Exiting Obstacle Avoidance Mode")

    def publish_avoidance_state(self, state):
        msg = Bool()
        msg.data = state
        self.avoidance_state_pub.publish(msg)

    def avoid_obj(self):
        """
        Strategically navigate around obstacles by finding the best gap.
        
        Strategy:
        1. Convert obstacles to horizontal angles
        2. Group nearby obstacles
        3. Find the largest gap closest to the center
        4. Steer towards the center of that gap
        5. Reduce throttle based on obstacle proximity
        """
        if not self.obstacles:
            return

        # Convert obstacles to horizontal angles and distances
        obstacle_data = []
        for x, distance in self.obstacles:
            # Calculate horizontal angle (approximately)
            horizontal_angle = np.degrees(np.arctan2(x, distance))
            obstacle_data.append((horizontal_angle, distance))
        
        # Sort obstacles by their horizontal angle
        obstacle_data.sort(key=lambda x: x[0])
        
        # Group obstacles that are close to each other (within self.grouping_angle degrees)
        grouped_obstacles = []
        group = [obstacle_data[0]]
        
        for i in range(1, len(obstacle_data)):
            if abs(obstacle_data[i][0] - obstacle_data[i-1][0]) <= self.grouping_angle:
                group.append(obstacle_data[i])
            else:
                grouped_obstacles.append(group)
                group = [obstacle_data[i]]
        
        grouped_obstacles.append(group)
        
        # Calculate gaps between grouped obstacles
        view_width = self.viewing_angle  # Use the viewing angle from the YAML configuration
        
        gaps = []

        # Add a virtual boundary at -view_width/2 and view_width/2
        grouped_obstacles = [[(-view_width / 2, None)]] + grouped_obstacles + [[(view_width / 2, None)]]

        for i in range(len(grouped_obstacles) - 1):
            right_edge = grouped_obstacles[i][-1][0]
            left_edge = grouped_obstacles[i+1][0][0]
            if left_edge - right_edge > 0:  # Positive gap
                gaps.append((right_edge, left_edge))
        
        if not gaps:
            # Entire view is blocked
            self.get_logger().warn("Entire view is blocked! Stopping robot.")
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            return

        # Select the gap closest to the center
        best_gap = min(gaps, key=lambda g: abs((g[0] + g[1]) / 2))
        
        # Calculate steering based on gap center
        gap_center = (best_gap[0] + best_gap[1]) / 2
        steering = (gap_center / view_width) + 0.5  # Normalize steering between 0 and 1
        steering = max(0, min(1, steering))  # Clamp between 0 and 1
        
        # Throttle reduction based on proximity
        min_distance = min(distance for _, distance in obstacle_data if distance)
        throttle = max(0.1, 0.5 - (0.4 * (self.critical_distance / min_distance)))  # Slow down near obstacles
        
        # Publish cmd_vel
        twist = Twist()
        twist.linear.x = throttle
        twist.angular.z = (steering - 0.5) * 2  # Convert normalized steering back to [-1, 1]
        self.cmd_vel_pub.publish(twist)
        
        # Text Visualization of the View
        self.view_res = 15  # Odd number for center
        view = ['-'] * self.view_res
        
        # Mark obstacles
        for obs in obstacle_data:
            # Convert angle to index
            idx = int((obs[0] / (view_width / 2) + 1) * (self.view_res - 1) / 2)
            if 0 <= idx < self.view_res:
                view[idx] = '#'
        
        # Mark gap center
        steer_idx = int((steering) * (self.view_res - 1))
        view[steer_idx] = '^'
        
        # Log the visualization
        view_str = ''.join(view)
        self.get_logger().info(f"Obstacle View: {view_str}\tSteering: {steering:.2f}, Throttle: {throttle:.3f}")


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


