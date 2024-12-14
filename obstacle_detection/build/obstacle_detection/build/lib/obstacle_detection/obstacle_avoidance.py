import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import Twist
import numpy as np
import yaml
import os
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
import matplotlib.animation as animation

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        
        # Load configuration
        config_path = os.path.join(os.path.dirname(__file__), 'config', 'obstacle_avoidance.yaml')
        with open(config_path, 'r') as file:
            self.config = yaml.safe_load(file)
        
        # Parameters from config
        self.visualization_enabled = self.config.get('visualization', False)
        self.avoidance_enabled = self.config.get('avoidance', True)
        self.critical_distance = self.config.get('critical_distance', 3.0)
        self.warning_distance = self.config.get('warning_distance', 10.0)
        
        # Subscribers
        self.detection_sub = self.create_subscription(
            Float32MultiArray, 
            '/object_detection', 
            self.detection_callback, 
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.avoidance_state_pub = self.create_publisher(Bool, '/avoidance_state', 10)
        
        # State variables
        self.is_avoiding = False
        self.obstacles = []
        
        # Visualization setup
        if self.visualization_enabled:
            self.setup_visualization()
        
        self.get_logger().info("Obstacle Avoidance Node Initialized")
    
    def detection_callback(self, msg):
        # Parse obstacle detections
        self.obstacles = []
        for i in range(0, len(msg.data), 3):
            x, y, distance = msg.data[i:i+3]
            self.obstacles.append((x, y, distance))
        
        # Check for avoidance conditions
        self.check_avoidance_state()
        
        # Visualize if enabled
        if self.visualization_enabled:
            self.update_visualization()
        
        # Perform obstacle avoidance if needed
        if self.is_avoiding:
            self.perform_rrt_avoidance()
    
    def check_avoidance_state(self):
        # Determine if obstacles are within critical or warning distance
        avoidance_needed = any(
            obs[2] < self.critical_distance or obs[2] < self.warning_distance 
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
    
    def perform_rrt_avoidance(self):
        """
        Simplified RRT path planning for obstacle avoidance
        This is a basic implementation and should be enhanced for real-world use
        """
        # Convert obstacles to a KD-Tree for efficient nearest neighbor search
        if not self.obstacles:
            return
        
        obstacle_points = np.array([(x, y) for x, y, _ in self.obstacles])
        obstacle_tree = cKDTree(obstacle_points)
        
        # Define goal (4m in front of the robot)
        goal = np.array([4.0, 0.0])
        
        # Basic avoidance logic
        # This is a simplified version and needs more sophisticated path planning
        closest_obstacle = obstacle_tree.query(goal)[1]
        
        # Determine steering and throttle based on obstacle position
        obstacle_x, obstacle_y = obstacle_points[closest_obstacle]
        
        # Calculate steering (simple proportional control)
        steering = max(0.0, min(1.0, abs(obstacle_x) / self.critical_distance))
        
        # Determine direction of steering based on obstacle position
        steering *= -1 if obstacle_x > 0 else 1
        
        # Throttle reduction based on proximity
        throttle = 0.03 * max(0, 1 - (len(self.obstacles) / 5))
        
        # Publish cmd_vel
        twist = Twist()
        twist.linear.x = throttle
        twist.angular.z = steering
        self.cmd_vel_pub.publish(twist)
    
    def setup_visualization(self):
        """Setup real-time top-down visualization of obstacles"""
        plt.ion()  # Interactive mode
        self.fig, self.ax = plt.subplots()
        self.scatter = self.ax.scatter([], [], c=[], cmap='viridis')
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(0, 5)
        self.ax.set_title('Top-Down Obstacle Detection')
        self.ax.set_xlabel('Lateral Distance (m)')
        self.ax.set_ylabel('Longitudinal Distance (m)')
    
    def update_visualization(self):
        """Update the real-time visualization"""
        if not self.visualization_enabled:
            return
        
        # Clear previous scatter plot
        self.scatter.remove()
        
        # Extract coordinates and distances
        x_coords = [obs[0] for obs in self.obstacles]
        y_coords = [obs[1] for obs in self.obstacles]
        distances = [obs[2] for obs in self.obstacles]
        
        # Create new scatter plot
        self.scatter = self.ax.scatter(
            x_coords, y_coords, 
            c=distances, 
            cmap='viridis', 
            vmin=0, 
            vmax=max(distances) if distances else 10
        )
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
