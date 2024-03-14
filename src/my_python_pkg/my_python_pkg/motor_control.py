from pynput.keyboard import Key, Listener
import threading
import rclpy
from rclpy.node import Node
import serial
import time

# SEND MOTOR COMMANDS TO ARDUINO
class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.arduino_right = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.arduino_left = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
        self.speed = 128  # Default speed
        self.start_keyboard_listener()
        self.start_data_reading_thread()

    def start_keyboard_listener(self):
        listener_thread = threading.Thread(target=self.keyboard_event_listener)
        listener_thread.start()

    def keyboard_event_listener(self):
        with Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            listener.join()

    def on_press(self, key):
        try:
            if key == Key.up:
                self.speed = min(255, self.speed + 32)  # Increase speed
            elif key == Key.down:
                self.speed = max(70, self.speed - 32)  # Decrease speed
            elif key.char == 'w':
                self.send_command_to_arduinos("start_forward")
            elif key.char == 's':
                self.send_command_to_arduinos("start_backward")
            elif key.char == 'a':
                self.send_command_to_arduinos("turn_left")
            elif key.char == 'd':
                self.send_command_to_arduinos("turn_right")
            elif key.char == 'q':
                self.send_command_to_arduinos("slight_left")
            elif key.char == 'e':
                self.send_command_to_arduinos("slight_right")
        except AttributeError:
            pass

    def on_release(self, key):
        self.send_command_to_arduinos("stop")

    def send_command_to_arduinos(self, command):
        self.get_logger().info(f'Sending command to Arduinos: {command}, Speed: {self.speed}')
        if command in ["start_forward", "start_backward", "stop"]:
            command_with_speed = f'{command}:{self.speed}\n'
            self.arduino_right.write(command_with_speed.encode())
            self.arduino_left.write(command_with_speed.encode())
        elif command == "turn_left":
            self.arduino_right.write(f'start_forward:{self.speed}\n'.encode())
            self.arduino_left.write(f'start_backward:{self.speed}\n'.encode())
        elif command == "turn_right":
            self.arduino_left.write(f'start_forward:{self.speed}\n'.encode())
            self.arduino_right.write(f'start_backward:{self.speed}\n'.encode())
        elif command == "slight_left":
            self.arduino_right.write(f'start_forward:{int(self.speed * 0.5)}\n'.encode())  # Adjust ratio as needed
            self.arduino_left.write(f'start_forward:{self.speed}\n'.encode())
        elif command == "slight_right":
            self.arduino_left.write(f'start_forward:{int(self.speed * 0.5)}\n'.encode())  # Adjust ratio as needed
            self.arduino_right.write(f'start_forward:{self.speed}\n'.encode())

    def __del__(self):
        self.arduino_right.close()
        self.arduino_left.close()
        
    def read_arduino_data(self):
        while True:
            if self.arduino_right.in_waiting > 0:
                data_right = self.arduino_right.readline().decode().strip()
                self.print_position("Right Arduino:", data_right)
            if self.arduino_left.in_waiting > 0:
                data_left = self.arduino_left.readline().decode().strip()
                self.print_position("Left Arduino:", data_left)
            time.sleep(0.1)  # Adjust as needed to match Arduino sending rate

    def print_position(self, prefix, data):
        print(f"{prefix} Raw data: {data}")  # Print raw data for debugging
        try:
            pos_front, pos_back = map(int, data.split())
            print(f"{prefix} Position Front: {pos_front}, Position Back: {pos_back}")
        except ValueError:
            print(f"{prefix} Error parsing data: {data}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
        # Start a separate thread for reading data
        data_thread = threading.Thread(target=node.read_arduino_data)
        data_thread.start()

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
 
