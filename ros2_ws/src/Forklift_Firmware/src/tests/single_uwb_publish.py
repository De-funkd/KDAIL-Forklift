#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import re
import threading
import time

class UWBDistancePublisher(Node):
    def __init__(self):
        super().__init__('uwb_distance_publisher')

        # Create publisher for distance data
        self.distance_publisher = self.create_publisher(
            Float32,
            '/uwb_1_responder_data',
            10
        )

        # Serial port configuration
        self.serial_port = '/dev/ttyACM0'
        self.baud_rate = 115200
        self.serial_connection = None

        # Distance extraction pattern
        self.distance_pattern = re.compile(r'<inf> main: Distance: ([\d.]+)')

        # Initialize serial connection
        self.init_serial()

        # Start reading thread
        self.reading_thread = threading.Thread(target=self.read_serial_data, daemon=True)
        self.reading_thread.start()

        self.get_logger().info('UWB Distance Publisher initialized')
        self.get_logger().info(f'Publishing distance data to topic: /uwb_1_responder_data')
        self.get_logger().info(f'Reading from serial port: {self.serial_port} at {self.baud_rate} baud')

    def init_serial(self):
        """Initialize serial connection with error handling"""
        try:
            self.serial_connection = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0
            )
            self.get_logger().info(f'Serial connection established on {self.serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {self.serial_port}: {e}')
            self.get_logger().error('Please check if the device is connected and the port is correct')
            raise

    def read_serial_data(self):
        """Continuously read data from serial port and extract distance"""
        while rclpy.ok():
            try:
                if self.serial_connection and self.serial_connection.in_waiting > 0:
                    # Read line from serial port
                    line = self.serial_connection.readline().decode('utf-8', errors='ignore').strip()

                    if line:
                        # Extract distance from the line
                        distance = self.extract_distance(line)
                        if distance is not None:
                            self.publish_distance(distance)

                else:
                    # Small delay to prevent excessive CPU usage
                    time.sleep(0.001)

            except serial.SerialException as e:
                self.get_logger().error(f'Serial communication error: {e}')
                self.get_logger().warn('Attempting to reconnect...')
                self.reconnect_serial()
            except Exception as e:
                self.get_logger().error(f'Unexpected error in read_serial_data: {e}')
                time.sleep(0.1)

    def extract_distance(self, line):
        """Extract distance value from log line"""
        match = self.distance_pattern.search(line)
        if match:
            try:
                distance = float(match.group(1))
                return distance
            except ValueError as e:
                self.get_logger().warn(f'Failed to convert distance to float: {e}')
        return None

    def publish_distance(self, distance):
        """Publish distance value to ROS topic"""
        try:
            msg = Float32()
            msg.data = distance
            self.distance_publisher.publish(msg)
            self.get_logger().info(f'Published distance: {distance:.6f} meters')
        except Exception as e:
            self.get_logger().error(f'Failed to publish distance: {e}')

    def reconnect_serial(self):
        """Attempt to reconnect serial connection"""
        if self.serial_connection:
            try:
                self.serial_connection.close()
            except:
                pass

        time.sleep(2.0)  # Wait before reconnecting

        try:
            self.init_serial()
            self.get_logger().info('Serial connection reestablished')
        except Exception as e:
            self.get_logger().error(f'Failed to reconnect: {e}')
            time.sleep(5.0)  # Wait longer before next attempt

    def destroy_node(self):
        """Clean up resources when node is destroyed"""
        if self.serial_connection:
            self.serial_connection.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    try:
        uwb_publisher = UWBDistancePublisher()
        rclpy.spin(uwb_publisher)
    except KeyboardInterrupt:
        print('\nShutdown requested by user')
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'uwb_publisher' in locals():
            uwb_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
