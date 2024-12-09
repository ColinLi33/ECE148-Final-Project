from pyvesc.VESC.messages import SetDutyCycle, SetServoPosition
import pyvesc
import serial
import math
import numpy as np
import time

class MotorController:
    def __init__(self, port="/dev/ttyACM0", baudrate=115200):
        self.vesc = None
        self.setup_vesc(port, baudrate)

    def setup_vesc(self, port, baudrate):
        try:
            self.vesc = serial.Serial(port, baudrate, timeout=0.1)
            print("VESC connected successfully.")
        except Exception as e:
            print(f"Failed to connect to VESC: {e}")

    def set_motor_speed(self, speed):
        if self.vesc:
            try:
                message = pyvesc.encode(SetDutyCycle(speed))
                self.vesc.write(message)
            except Exception as e:
                print(f"Failed to set motor speed: {e}")

    def set_motor_speed_time(self, speed, duration):
        if self.vesc:
            try:
                startTime = time.time()
                while(time.time() - startTime < duration):
                    message = pyvesc.encode(SetDutyCycle(speed))
                    self.vesc.write(message)
                    time.sleep(0.1)
            except Exception as e:
                print(f"Failed to set motor speed for time: {e}")

    def set_servo_position(self, position):
        if self.vesc:
            try:
                # position should be between 0 and 1
                message = pyvesc.encode(SetServoPosition(position))
                self.vesc.write(message)
            except Exception as e:
                print(f"Failed to set servo position: {e}")