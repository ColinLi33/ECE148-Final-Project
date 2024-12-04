import math
import numpy as np
from motor_controller import MotorController
import time
class PathFollower:
    def __init__(self, gps):
        self.gps = gps
        # self.motor_controller = MotorController()
        self.min_distance_threshold = 2.0
        self.default_speed = 0.1

    def interpolate_path(self, path, num_points_between=5):
        points = np.array(path)
        t = np.linspace(0, 1, len(points))
        fx = np.interp(np.linspace(0, 1, (len(points) - 1) * num_points_between + 1), t, points[:, 0])
        fy = np.interp(np.linspace(0, 1, (len(points) - 1) * num_points_between + 1), t, points[:, 1])
        interpolated_points = [[x, y] for x, y in zip(fx, fy)]
        return interpolated_points

    def save_path(self, path):
        with open('./static/path.csv', 'w') as f:
            origin = path[0]
            for node in path:
                latDiff, longDiff = self.lat_long_difference(origin, node)
                f.write(f'{latDiff},{longDiff},{0.5}\n')
        print("Path saved to path.csv")

    def lat_long_difference(self, origin, node):
        lat_to_meters = 111320
        long_to_meters = 40075000 * math.cos(math.radians(origin[0])) / 360

        latDiff = (node[0] - origin[0]) * lat_to_meters
        longDiff = (node[1] - origin[1]) * long_to_meters
        return latDiff, longDiff

    def compute_steering(self, current_heading, target_heading):
        """
        Compute the steering adjustment to align the robot's heading with the target heading.
        :param current_heading: float
        :param target_heading: float
        :return: float (steering value between 0.0 and 1.0)
        """
        error = (target_heading - current_heading + 360) % 360
        if error > 180:
            error -= 360
        
        # Normalize steering to range [0.0, 1.0]
        # Center is 0.5, left is < 0.5, right is > 0.5
        steering = 0.5 + (error / 180.0) * 0.5
        return max(0.0, min(1.0, steering))

    def follow_path(self, path):
        """
        Path-following logic to move through waypoints.
        :param path: List of waypoints [(lat, long), ...]
        """
        try:
            for waypoint in path:
                print(f"Following waypoint: {waypoint}")
                while True:
                    # Get current GPS location
                    current_lat = self.gps.current_location["lat"]
                    current_long = self.gps.current_location["long"]
                    current_heading = self.gps.current_location["heading"]
                    print("heading", current_heading)

                    # Calculate distance and heading to the next waypoint
                    latDiff, longDiff = self.lat_long_difference(
                        (current_lat, current_long), waypoint
                    )
                    distance = math.sqrt(latDiff**2 + longDiff**2)
                    target_heading = math.degrees(math.atan2(longDiff, latDiff))

                    # Compute steering angle
                    steering = self.compute_steering(current_heading, target_heading)
                    
                    # Apply controls
                    print("steering", steering)
                    # self.motor_controller.set_servo_position(steering)
                    # self.motor_controller.set_motor_speed(self.default_speed)

                    # Check if we've reached the waypoint
                    if distance < self.min_distance_threshold:
                        # self.motor_controller.set_motor_speed(0.0)
                        print(f"Reached waypoint: {waypoint}")
                        break
                    time.sleep(.1)

        except Exception as e:
            print(f"Error in path following: {e}")
            # self.motor_controller.set_motor_speed(0.0)  # Safety stop
            # self.motor_controller.set_servo_position(0.5)  # Center steering
            raise

        finally:
            exit()
            # Ensure the robot stops when done or if there's an error
            # self.motor_controller.set_motor_speed(0.0)
            # self.motor_controller.set_servo_position(0.5)