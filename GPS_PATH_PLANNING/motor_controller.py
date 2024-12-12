from pyvesc.VESC.messages import SetDutyCycle, SetServoPosition
import pyvesc
import serial
import math
import numpy as np
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
from threading import Lock

class MotorController(Node):
    def __init__(self, port="/dev/ttyACM0", baudrate=115200):
        # Initialize ROS2
        rclpy.init()
        super().__init__('motor_controller')
        
        # VESC setup
        self.vesc = None
        self.setup_vesc(port, baudrate)
        
        # ROS2 subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
            
        # Control flags and locks
        self.override_active = False
        self.lock = Lock()
        
        # Start ROS2 spin in separate thread
        self.spin_thread = threading.Thread(target=self._spin)
        self.spin_thread.daemon = True
        self.spin_thread.start()

    def setup_vesc(self, port, baudrate):
        try:
            self.vesc = serial.Serial(port, baudrate, timeout=0.1)
            print("VESC connected successfully.")
        except Exception as e:
            print(f"Failed to connect to VESC: {e}")

    def _spin(self):
        while True:
            rclpy.spin_once(self, timeout_sec=0.1)

    def cmd_vel_callback(self, msg):
        with self.lock:
            # Check if this is an override command
            if abs(msg.linear.x) < 0.01 and abs(msg.angular.z) < 0.01:
                self.override_active = True
                self._direct_motor_speed(0)
                self._direct_servo_position(0.5)
            else:
                # Convert cmd_vel to motor commands
                speed = msg.linear.x  # You might need to scale this appropriately
                steering = 0.5 + (msg.angular.z / 2)  # Convert angular.z to [0,1] range
                self.override_active = True
                self._direct_motor_speed(speed)
                self._direct_servo_position(steering)

    def _direct_motor_speed(self, speed):
        """Direct motor control without override check"""
        if self.vesc:
            try:
                message = pyvesc.encode(SetDutyCycle(speed))
                self.vesc.write(message)
            except Exception as e:
                print(f"Failed to set motor speed: {e}")

    def _direct_servo_position(self, position):
        """Direct servo control without override check"""
        if self.vesc:
            try:
                message = pyvesc.encode(SetServoPosition(position))
                self.vesc.write(message)
            except Exception as e:
                print(f"Failed to set servo position: {e}")

    def set_motor_speed(self, speed):
        """Public method that respects override"""
        with self.lock:
            if not self.override_active:
                self._direct_motor_speed(speed)

    def set_motor_speed_time(self, speed, duration):
        """Public method that respects override"""
        if self.vesc:
            try:
                startTime = time.time()
                while(time.time() - startTime < duration):
                    with self.lock:
                        if not self.override_active:
                            self._direct_motor_speed(speed)
                        time.sleep(0.1)
            except Exception as e:
                print(f"Failed to set motor speed for time: {e}")

    def set_servo_position(self, position):
        """Public method that respects override"""
        with self.lock:
            if not self.override_active:
                self._direct_servo_position(position)

    def cleanup(self):
        """Cleanup ROS2 node"""
        self.destroy_node()
        rclpy.shutdown()