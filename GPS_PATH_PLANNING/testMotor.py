import keyboard
import time
from motor_controller import MotorController  # Assuming the previous code is in motor_controller.py

class VESCController:
    def __init__(self):
        self.motor = MotorController()
        self.current_speed = 0
        self.current_position = 0.5  # Center position for servo
        self.speed_increment = 0.05
        self.position_increment = 0.05
        
    def increase_speed(self):
        self.current_speed = min(1.0, self.current_speed + self.speed_increment)
        self.motor.set_motor_speed(self.current_speed)
        print(f"Speed: {self.current_speed:.2f}")

    def decrease_speed(self):
        self.current_speed = max(-1.0, self.current_speed - self.speed_increment)
        self.motor.set_motor_speed(self.current_speed)
        print(f"Speed: {self.current_speed:.2f}")

    def turn_left(self):
        self.current_position = max(0.0, self.current_position - self.position_increment)
        self.motor.set_servo_position(self.current_position)
        print(f"Servo position: {self.current_position:.2f}")

    def turn_right(self):
        self.current_position = min(1.0, self.current_position + self.position_increment)
        self.motor.set_servo_position(self.current_position)
        print(f"Servo position: {self.current_position:.2f}")

    def stop(self):
        self.current_speed = 0
        self.current_position = 0.5
        self.motor.set_motor_speed(self.current_speed)
        self.motor.set_servo_position(self.current_position)
        print("Emergency Stop! All controls reset.")

def main():
    controller = VESCController()
    
    print("""
    VESC Control Program
    -------------------
    Controls:
    Up Arrow: Increase Speed
    Down Arrow: Decrease Speed
    Left Arrow: Turn Left
    Right Arrow: Turn Right
    Spacebar: Emergency Stop
    ESC: Exit Program
    """)

    try:
        keyboard.on_press_key("up", lambda _: controller.increase_speed())
        keyboard.on_press_key("down", lambda _: controller.decrease_speed())
        keyboard.on_press_key("left", lambda _: controller.turn_left())
        keyboard.on_press_key("right", lambda _: controller.turn_right())
        keyboard.on_press_key("space", lambda _: controller.stop())
        
        # Keep the program running until ESC is pressed
        keyboard.wait("esc")
        
    except Exception as e:
        print(f"An error occurred: {e}")
    
    finally:
        # Clean up
        controller.stop()
        print("Program terminated.")

if __name__ == "__main__":
    main()