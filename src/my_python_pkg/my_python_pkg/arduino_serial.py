import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading

# READ ARDUINO SERIAL DATA
class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.publisher_1 = self.create_publisher(String, 'encoder_left_data', 1)
        self.publisher_2 = self.create_publisher(String, 'encoder_right_data', 1)
        self.subscription = self.create_subscription(
            String,
            'commands',
            self.send_command_to_arduino,
            10)

        self.serial_port1 = serial.Serial('/dev/ttyACM2', 115200, timeout=0)  # Set timeout to 0 for non-blocking
        self.serial_port2 = serial.Serial('/dev/ttyACM3', 115200, timeout=0)  # Set timeout to 0 for non-blocking

        self.serial_thread1 = threading.Thread(target=self.read_from_arduino, args=(self.serial_port1, self.publisher_1))
        self.serial_thread2 = threading.Thread(target=self.read_from_arduino, args=(self.serial_port2, self.publisher_2))
        self.serial_thread1.start()
        self.serial_thread2.start()

        self.spin_thread = threading.Thread(target=self.spin_node)
        self.spin_thread.start()

    def read_from_arduino(self, serial_port, publisher):
        while rclpy.ok():
            if serial_port.in_waiting:
                data = serial_port.readline().decode('utf-8').rstrip()
                
                # Split the data based on whitespace and extract positions
                split_data = data.split()
                try:
                    if len(split_data) == 6:  # Expected format: ["Front", "pos:", <pos_front>, "Back", "pos:", <pos_back>]
                        pos_front = int(split_data[5])  # Convert to integer
                        pos_back = int(split_data[2])  # Convert to integer

                        # Publish to appropriate topics with formatted data
                        if publisher == self.publisher_1:
                            formatted_data = f"front: {pos_front}\nback: {pos_back}"
                            publisher.publish(String(data=f"left_motor:\n{formatted_data}"))  # Left motor
                        elif publisher == self.publisher_2:
                            formatted_data_negative = f"front: {-pos_front}\nback: {-pos_back}"
                            publisher.publish(String(data=f"right_motor:\n{formatted_data_negative}"))  # Right motor
                except (IndexError, ValueError) as e:
                    self.get_logger().warn(f"Invalid data format received: {data}, Error: {e}")


    def send_command_to_arduino(self, msg):
        arduino_id, command = msg.data.split(':', 1)
        if arduino_id == "1":
            self.serial_port1.write(command.encode('utf-8'))
        elif arduino_id == "2":
            self.serial_port2.write(command.encode('utf-8'))

    def spin_node(self):
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNode()
    serial_node.serial_thread1.join()
    serial_node.serial_thread2.join()
    serial_node.spin_thread.join()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
