from pyvesc.VESC.messages import SetDutyCycle, SetServoPosition
import pyvesc
import serial
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
                # Convert speed (-1 to 1) to duty cycle (-100000 to 100000)
                duty_cycle = speed * 100000
                message = pyvesc.encode(SetDutyCycle(duty_cycle))
                self.vesc.write(message)
            except Exception as e:
                print(f"Failed to set motor speed: {e}")

    def set_servo_position(self, position):
        if self.vesc:
            try:
                # Convert position (0 to 1) to servo value (0 to 1)
                message = pyvesc.encode(SetServoPosition(position))
                self.vesc.write(message)
            except Exception as e:
                print(f"Failed to set servo position: {e}")

class VESCController:
    def __init__(self):
        self.motor = MotorController()
        self.current_speed = 0
        self.current_position = 0.5  # Center position for servo
        self.speed_increment = 0.05
        self.position_increment = 0.05
        self.running = True

    def print_status(self):
        print(f"\nCurrent Speed: {self.current_speed:.2f}")
        print(f"Current Servo Position: {self.current_position:.2f}")

    def handle_command(self, command):
        if command.lower() == 'w':  # Increase speed
            self.current_speed = min(1.0, self.current_speed + self.speed_increment)
            self.motor.set_motor_speed(self.current_speed)
            
        elif command.lower() == 's':  # Decrease speed
            self.current_speed = max(-1.0, self.current_speed - self.speed_increment)
            self.motor.set_motor_speed(self.current_speed)
            
        elif command.lower() == 'a':  # Turn left
            self.current_position = max(0.0, self.current_position - self.position_increment)
            self.motor.set_servo_position(self.current_position)
            
        elif command.lower() == 'd':  # Turn right
            self.current_position = min(1.0, self.current_position + self.position_increment)
            self.motor.set_servo_position(self.current_position)
            
        elif command.lower() == 'x':  # Stop
            self.current_speed = 0
            self.current_position = 0.5
            self.motor.set_motor_speed(self.current_speed)
            self.motor.set_servo_position(self.current_position)
            print("Emergency Stop! All controls reset.")
            
        elif command.lower() == 'q':  # Quit
            self.stop()
            self.running = False
            return
            
        else:
            print("Invalid command!")
            
        self.print_status()

    def stop(self):
        self.current_speed = 0
        self.current_position = 0.5
        self.motor.set_motor_speed(self.current_speed)
        self.motor.set_servo_position(self.current_position)

def main():
    controller = VESCController()
    
    print("""
    VESC Control Program (SSH Version)
    --------------------------------
    Commands:
    w: Increase Speed
    s: Decrease Speed
    a: Turn Left
    d: Turn Right
    x: Emergency Stop
    q: Quit Program
    
    Enter command: """)

    try:
        while controller.running:
            command = input("Command: ")
            controller.handle_command(command)
            
    except Exception as e:
        print(f"An error occurred: {e}")
    
    finally:
        controller.stop()
        print("Program terminated.")

if __name__ == "__main__":
    main()