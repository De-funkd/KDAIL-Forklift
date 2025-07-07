#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import serial
import re
import glob
import time
from typing import List

class UWBLogParser(Node):
    """
    Parses UWB logs from a serial port, handles anchor timeouts,
    and publishes consolidated distance and status messages.
    """
    def __init__(self):
        super().__init__('uwb_log_parser')

        # Publishers
        self.distance_publisher = self.create_publisher(Float32MultiArray, '/uwb_distances', 10)
        self.status_publisher = self.create_publisher(String, '/uwb_status', 10)

        # Configuration
        self.timeout_threshold = 2.0  # Seconds to mark an anchor as "Not received"
        self.num_anchors = 4

        # State variables
        self.ser = None
        self.distances = [0.0] * self.num_anchors
        self.last_received_time = [0.0] * self.num_anchors

        # Attempt to connect to the serial port
        if self.connect_to_port():
            self.get_logger().info(f'Successfully connected to UWB device on port {self.ser.port}')
            # Timer for reading data from serial (high frequency)
            self.create_timer(0.01, self.read_serial_data)
            # Timer for processing and publishing data (10 Hz)
            self.create_timer(0.1, self.process_and_publish)
        else:
            self.get_logger().error('Failed to connect to any UWB device. Please check connections.')

    def find_available_ports(self) -> List[str]:
        """Detects available serial ports, prioritizing /dev/ttyACM*."""
        ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
        if not ports:
            self.get_logger().warning('No /dev/ttyACM* or /dev/ttyUSB* ports found.')
        return ports

    def connect_to_port(self) -> bool:
        """Iterates through available ports and attempts to establish a serial connection."""
        for port in self.find_available_ports():
            try:
                self.get_logger().info(f'Attempting to connect to {port}...')
                self.ser = serial.Serial(port, 115200, timeout=1.0)
                # Wait for the connection to stabilize
                time.sleep(1.5)
                # A simple check to see if we are receiving data
                if self.ser.in_waiting > 0:
                    return True
                self.get_logger().warning(f'Connected to {port}, but no data received. Trying next port.')
                self.ser.close()

            except serial.SerialException as e:
                self.get_logger().warning(f'Failed to connect to {port}: {e}')
        return False

    def read_serial_data(self):
        """Reads lines from the serial port and sends them to the parser."""
        if not self.ser or not self.ser.is_open:
            return

        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    self.parse_line(line)
        except (serial.SerialException, IOError) as e:
            self.get_logger().error(f"Serial read error: {e}. Closing port.")
            if self.ser:
                self.ser.close()
            self.ser = None
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred while reading serial data: {e}")


    def parse_line(self, line: str):
        """Parses a single line of log data for distance information."""
        # Pattern: [timestamp] <inf> main: ID: X, Distance: Y.ZZZZZZ
        distance_match = re.search(r'main: ID: (\d+), Distance: ([\d.]+)', line)

        if distance_match:
            try:
                anchor_id = int(distance_match.group(1))
                distance = float(distance_match.group(2))

                if 1 <= anchor_id <= self.num_anchors:
                    idx = anchor_id - 1
                    self.distances[idx] = distance
                    self.last_received_time[idx] = time.time()
                    self.get_logger().debug(f"Parsed Anchor ID: {anchor_id}, Distance: {distance:.3f}m")
            except (ValueError, IndexError) as e:
                self.get_logger().warning(f"Error parsing distance data: {e} from line: {line}")

    def process_and_publish(self):
        """
        Checks for anchor timeouts, updates state, and publishes distance and status messages.
        This function is called by a 10 Hz timer.
        """
        current_time = time.time()
        status_info = []

        # Check for timeouts and build status message
        for i in range(self.num_anchors):
            if current_time - self.last_received_time[i] > self.timeout_threshold:
                # If timed out, reset distance to 0.0
                if self.distances[i] != 0.0:
                    self.get_logger().info(f"Anchor ID {i+1} timed out. Resetting distance to 0.0.")
                    self.distances[i] = 0.0
                status_info.append(f"ID{i+1}: Not received")
            else:
                status_info.append(f"ID{i+1}: {self.distances[i]:.3f}m")

        # Publish distances
        distance_msg = Float32MultiArray()
        distance_msg.data = self.distances
        self.distance_publisher.publish(distance_msg)

        # Publish status
        status_msg = String()
        status_msg.data = " | ".join(status_info)
        self.status_publisher.publish(status_msg)

        self.get_logger().debug(f"Published distances: {[f'{d:.3f}' for d in self.distances]}")

    def destroy_node(self):
        """Gracefully shuts down the node and closes the serial port."""
        self.get_logger().info("Shutting down UWB Log Parser.")
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = UWBLogParser()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().fatal(f"An unhandled exception occurred: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()