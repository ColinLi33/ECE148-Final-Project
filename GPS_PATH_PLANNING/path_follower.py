import math
import numpy as np
from motor_controller import MotorController


class PathFollower:
    def __init__(self, gps):
        self.gps = gps
        self.motor_controller = MotorController()

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
        steering = 0.5 + error / 90.0  # Normalize to range [0.0, 1.0]
        return max(0.0, min(1.0, steering))

    def follow_path(self, path):
        """
        Path-following logic to move through waypoints.
        :param path: List of waypoints [(lat, long), ...]
        """
        for waypoint in path:
            while True:
                # Calculate distance and heading to the next waypoint
                latDiff, longDiff = self.lat_long_difference(
                    (self.gps.current_location["lat"], self.gps.current_location["long"]), waypoint
                )
                distance = math.sqrt(latDiff**2 + longDiff**2)
                target_heading = math.degrees(math.atan2(longDiff, latDiff))

                # Adjust motor and servo
                steering = self.compute_steering(self.gps.current_location["heading"], target_heading)
                self.motor_controller.set_servo_position(steering)
                self.motor_controller.set_motor_speed(0.2)  # Moderate speed forward

                if distance < 2.0:  # Stop when close to waypoint
                    self.motor_controller.set_motor_speed(0.0)
                    break
