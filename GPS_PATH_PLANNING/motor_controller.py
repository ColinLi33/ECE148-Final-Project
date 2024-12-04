from pyvesc.VESC.messages import SetDutyCycle, SetServoPosition
import serial


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
                msg = SetDutyCycle(speed)
                self.vesc.write(msg.serialize())
            except Exception as e:
                print(f"Failed to set motor speed: {e}")

    def set_servo_position(self, position):
        if self.vesc:
            try:
                msg = SetServoPosition(position)
                self.vesc.write(msg.serialize())
            except Exception as e:
                print(f"Failed to set servo position: {e}")
