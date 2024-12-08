import numpy as np
import matplotlib.pyplot as plt
import math
import time
from dataclasses import dataclass
import os 
import csv

class SimulatedGPS:
    def __init__(self, start_lat, start_long, start_heading):
        self.true_location = {
            'lat': start_lat,
            'long': start_long,
            'heading': start_heading
        }
        self.current_location = self.true_location.copy()
        
        # GPS noise parameters (in degrees for lat/long)
        self.position_noise_std = 0.00003  # About 0.5m at equator
        self.heading_noise_std = 10.0  # degrees
        
        # Add initial noise
        self.add_noise()
    
    def add_noise(self):
        """Add random noise to GPS readings"""
        self.current_location = {
            'lat': self.true_location['lat'] + np.random.normal(0, self.position_noise_std),
            'long': self.true_location['long'] + np.random.normal(0, self.position_noise_std),
            'heading': self.true_location['heading'] + np.random.normal(0, self.heading_noise_std)
        }
        
        # Ensure heading stays within 0-360
        self.current_location['heading'] %= 360
    
    def update_position(self, speed, steering_position):
        """Update position based on speed and steering"""
        # Convert steering position [0,1] to angle [-180,180]
        steering_angle = (steering_position - 0.5) * 180
        
        # Update true heading
        self.true_location['heading'] += steering_angle * 0.1
        self.true_location['heading'] %= 360
        
        # Calculate new position (simplified model)
        heading_rad = math.radians(self.true_location['heading'])
        speed_factor = 0.00001  # Scale factor for simulation
        
        dlat = math.cos(heading_rad) * speed * speed_factor
        dlong = math.sin(heading_rad) * speed * speed_factor
        
        # Update true position
        self.true_location['lat'] += dlat
        self.true_location['long'] += dlong
        
        # Add noise to create the GPS reading
        self.add_noise()

class SimulatedMotorController:
    def __init__(self):
        self.current_speed = 0
        self.current_steering = 0.5

    def set_motor_speed(self, speed):
        self.current_speed = speed
    
    def set_servo_position(self, position):
        self.current_steering = position

class Robot:
    def __init__(self, start_lat, start_long, start_heading):
        self.gps = SimulatedGPS(start_lat, start_long, start_heading)
        self.motor_controller = SimulatedMotorController()
        self.path_history = [[start_lat, start_long]]
        self.true_path_history = [[start_lat, start_long]]
        self.noisy_path_history = [[start_lat, start_long]]
    
    def haversine_distance(self, point1, point2):
        """Calculate distance between two lat/long points in meters"""
        R = 6371000  # Earth's radius in meters
        lat1, lon1 = math.radians(point1[0]), math.radians(point1[1])
        lat2, lon2 = math.radians(point2[0]), math.radians(point2[1])
        
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c
    
    def calculate_bearing(self, point1, point2):
        """Calculate bearing between two lat/long points in degrees"""
        lat1, lon1 = math.radians(point1[0]), math.radians(point1[1])
        lat2, lon2 = math.radians(point2[0]), math.radians(point2[1])
        
        dlon = lon2 - lon1
        
        y = math.sin(dlon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        bearing = math.atan2(y, x)
        
        return math.degrees(bearing) % 360
    
    def initialize_direction(self, first_waypoint):
        self.gps.current_location['heading'] = self.calculate_bearing(
            [self.gps.current_location['lat'], self.gps.current_location['long']],
            first_waypoint
        )

    def find_closest_waypoint(self, current_location, waypoints):
        """
        Find the closest waypoint that is ahead in the path.
        Returns the index of the closest waypoint.
        """
        min_distance = float('inf')
        closest_idx = 0
        
        for i, waypoint in enumerate(waypoints):
            distance = self.haversine_distance(
                [current_location['lat'], current_location['long']], 
                waypoint
            )
            
            # Calculate bearing to the waypoint
            bearing_to_waypoint = self.calculate_bearing(
                [current_location['lat'], current_location['long']], 
                waypoint
            )
            
            # Calculate the absolute difference between current heading and bearing to waypoint
            heading_diff = abs(current_location['heading'] - bearing_to_waypoint)
            if heading_diff > 180:
                heading_diff = 360 - heading_diff
                
            # Only consider waypoints that are roughly ahead of us (within 90 degrees)
            if heading_diff < 90 and distance < min_distance:
                min_distance = distance
                closest_idx = i
                
        return closest_idx

    def follow_path(self, interpolated_path, tolerance=2.0):
        # Find the closest waypoint to start from
        start_idx = self.find_closest_waypoint(self.gps.current_location, interpolated_path)
        print(f"Starting from waypoint index: {start_idx}")
        
        self.initialize_direction(interpolated_path[start_idx])
        
        # Continue from the closest waypoint
        for waypoint in interpolated_path[start_idx:]:
            while True:
                # Calculate distance to the waypoint
                distance = self.haversine_distance(
                    [self.gps.current_location['lat'], self.gps.current_location['long']], 
                    waypoint
                )
                print("Distance", distance)
                
                # Check if waypoint is reached
                if distance < tolerance:
                    print(f"Waypoint {waypoint} reached!")
                    break

                # Calculate bearing to the waypoint
                target_bearing = self.calculate_bearing(
                    [self.gps.current_location['lat'], self.gps.current_location['long']], 
                    waypoint
                )

                # Calculate steering correction
                steering_correction = target_bearing - self.gps.current_location['heading']
                if steering_correction > 180:
                    steering_correction -= 360
                elif steering_correction < -180:
                    steering_correction += 360

                steering_position = 0.5 + (steering_correction / 180) * 0.5
                steering_position = max(0, min(1, steering_position))  # Clamp to [0, 1]

                # Set throttle and steering
                self.motor_controller.set_motor_speed(0.1)
                self.motor_controller.set_servo_position(steering_position)
                print("Steering", steering_position)
                
                # Update simulation
                self.gps.update_position(
                    self.motor_controller.current_speed,
                    self.motor_controller.current_steering
                )
                
                # Store both true and noisy positions
                self.true_path_history.append([
                    self.gps.true_location['lat'],
                    self.gps.true_location['long']
                ])
                self.noisy_path_history.append([
                    self.gps.current_location['lat'],
                    self.gps.current_location['long']
                ])

def plot_path(waypoints, true_path, noisy_path):
    """Plot the waypoints and robot's paths"""
    waypoints = np.array(waypoints)
    true_path = np.array(true_path)
    noisy_path = np.array(noisy_path)
    
    plt.figure(figsize=(10, 10))
    
    # Plot noisy GPS readings first (in background) with high transparency
    plt.plot(noisy_path[:, 1], noisy_path[:, 0], 'g.', 
             label='GPS Readings', alpha=0.2, markersize=2)
    
    # Plot waypoints last (on top) with larger markers
    plt.plot(waypoints[:, 1], waypoints[:, 0], 'ro-', 
             label='Waypoints', linewidth=1.5, 
             markersize=8, zorder=2)
    
    # Plot true path with thicker line
    plt.plot(true_path[:, 1], true_path[:, 0], 'b-', 
             label='True Path', linewidth=2, zorder=3)
    
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.legend()
    plt.grid(True)
    plt.show()

# Test the simulator
if __name__ == "__main__":
    # Read waypoints from CSV file
    waypoints = []
    try:
        with open('static/path.csv', 'r') as file:
            csv_reader = csv.reader(file)
            for row in csv_reader:
                # Assuming each row has lat,long format
                try:
                    lat = float(row[0])
                    lon = float(row[1])
                    waypoints.append([lat, lon])
                except (ValueError, IndexError) as e:
                    print(f"Skipping invalid row: {row}. Error: {e}")
    except FileNotFoundError:
        print("Error: static/path.csv not found")
        exit(1)

    if not waypoints:
        print("No valid waypoints found in path.csv")
        exit(1)

    print(f"Loaded {len(waypoints)} waypoints from path.csv")
    
    # Create robot at a position different from the first waypoint
    start_lat = waypoints[0][0] + 0.0000005  # Small offset from first waypoint
    start_long = waypoints[0][1] + 0.0000005
    start_heading = 45  # Starting heading in degrees
    
    robot = Robot(start_lat, start_long, start_heading)
    
    # Follow path
    robot.follow_path(waypoints, tolerance=2)  # Smaller tolerance for simulation scale
    
    # Plot results
    plot_path(waypoints, robot.true_path_history, robot.noisy_path_history)