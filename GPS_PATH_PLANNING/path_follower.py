import math
import numpy as np
from motor_controller import MotorController
import time

class PathFollower:
    def __init__(self, gps):
        self.gps = gps
        self.motor_controller = MotorController()
        # Control constants
        self.STEERING_GAIN = 0.5
        self.MAX_STEERING_CHANGE = 0.1
        self.HEADING_ERROR_THRESHOLD = 5
        self.BASE_SPEED = 0.05
        self.MIN_SPEED = 0.03
        self.LOOK_AHEAD_DISTANCE = 5

    def interpolate_path(self, path, num_points=1):
        if not path or len(path) < 2:
            raise ValueError("Path must have at least two points.")
        interpolated_path = []
        for i in range(len(path) - 1):
            start = path[i]
            end = path[i + 1]
            interpolated_path.append(start)
            x_diff = end[0] - start[0]
            y_diff = end[1] - start[1]
            for j in range(1, num_points + 1):
                t = j / (num_points + 1)
                x_interpolated = start[0] + t * x_diff
                y_interpolated = start[1] + t * y_diff
                interpolated_path.append([x_interpolated, y_interpolated])
        interpolated_path.append(path[-1])
        return interpolated_path

    def save_path(self, path):
        with open('./static/path.csv', 'w') as f:
            for node in path:
                f.write(f'{node[0]},{node[1]}\n')
        print("Path saved to path.csv")

    def haversine_distance(self, coord1, coord2):
        """Calculate the great-circle distance between two points on the Earth."""
        R = 6371000  # Radius of Earth in meters
        lat1, lon1 = math.radians(coord1[0]), math.radians(coord1[1])
        lat2, lon2 = math.radians(coord2[0]), math.radians(coord2[1])
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c

    def calculate_bearing(self, current, target):
        """Calculate the bearing from current location to target location."""
        lat1, lon1 = math.radians(current[0]), math.radians(current[1])
        lat2, lon2 = math.radians(target[0]), math.radians(target[1])
        dlon = lon2 - lon1
        x = math.sin(dlon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        return (math.degrees(math.atan2(x, y)) + 360) % 360

    def initialize_direction(self, first_waypoint):
        print("Initializing direction...")
        
        # Start moving forward briefly to compute initial heading
        self.motor_controller.set_servo_position(0.5)
        self.motor_controller.set_motor_speed_time(0.05, 3)
        
        # Get current GNSS data
        target_bearing = self.calculate_bearing(
            [self.gps.current_location['lat'], self.gps.current_location['long']], 
            first_waypoint
        )

        # Adjust steering to align with the target bearing
        steering_correction = target_bearing - self.gps.current_location['heading']
        if steering_correction > 180:
            steering_correction -= 360
        elif steering_correction < -180:
            steering_correction += 360

        # Normalize steering correction to [0, 1]
        steering_position = 0.5 + (steering_correction / 180) * 0.5
        steering_position = max(0, min(1, steering_position))

        print(f"Initial target bearing: {target_bearing}")
        print(f"Initial GNSS heading: {self.gps.current_location['heading']}")
        print(f"Setting initial steering to: {steering_position}")

        # Stop briefly before starting regular path following
        self.motor_controller.set_motor_speed(0)
        time.sleep(1)
        print("Initialization complete.")

    def find_closest_waypoint(self, current_location, waypoints):
        """Find the closest waypoint that is ahead in the path."""
        min_distance = float('inf')
        closest_idx = 0

        for i, waypoint in enumerate(waypoints):
            distance = self.haversine_distance(
                [current_location['lat'], current_location['long']], 
                waypoint
            )
            
            bearing_to_waypoint = self.calculate_bearing(
                [current_location['lat'], current_location['long']],
                waypoint
            )

            heading_diff = abs(current_location['heading'] - bearing_to_waypoint)
            if heading_diff > 180:
                heading_diff = 360 - heading_diff

            # Consider both distance and heading when selecting waypoint
            if (heading_diff < 90 and distance < min_distance and 
                distance > self.LOOK_AHEAD_DISTANCE):
                min_distance = distance
                closest_idx = i

        return closest_idx

    def follow_path(self, interpolated_path, tolerance=2.0):
        print("Following path...", interpolated_path)
        
        # Wait for valid heading data
        while self.gps.current_location['heading'] is None:
            print(self.gps.current_location)
            time.sleep(1)

        self.initialize_direction(interpolated_path[0])
        currIdx = 0
        previous_steering = 0.5

        while currIdx < len(interpolated_path):
            currIdx += self.find_closest_waypoint(
                self.gps.current_location, 
                interpolated_path[currIdx:]
            )
            waypoint = interpolated_path[currIdx]
            print(f"Starting from waypoint index: {currIdx}")

            # Calculate distance to the waypoint
            distance = self.haversine_distance(
                [self.gps.current_location['lat'], self.gps.current_location['long']],
                waypoint
            )
            print("Distance", distance)

            # Check if waypoint is reached
            if distance < tolerance:
                currIdx += 1
                print(f"Waypoint {waypoint} reached!")
                continue

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

            # Apply heading error threshold
            if abs(steering_correction) < self.HEADING_ERROR_THRESHOLD:
                steering_position = 0.5  # Keep going straight
            else:
                # Apply proportional control
                steering_position = 0.5 + (steering_correction / 180) * 0.5 * self.STEERING_GAIN

            # Limit steering rate
            steering_change = steering_position - previous_steering
            if abs(steering_change) > self.MAX_STEERING_CHANGE:
                steering_position = previous_steering + self.MAX_STEERING_CHANGE * np.sign(steering_change)

            steering_position = max(0, min(1, steering_position))
            previous_steering = steering_position

            # Calculate variable speed based on turn angle
            turn_factor = abs(steering_position - 0.5) * 2
            speed = self.BASE_SPEED - (self.BASE_SPEED - self.MIN_SPEED) * turn_factor

            # Set throttle and steering
            self.motor_controller.set_motor_speed(speed)
            self.motor_controller.set_servo_position(steering_position)
            time.sleep(0.2)

        # Stop the robot at the end
        self.motor_controller.set_motor_speed(0)
        self.motor_controller.set_servo_position(0.5)
        print("Path completed!")