#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import re
import glob
from typing import List, Optional

class UWBNode(Node):
    def __init__(self):
        super().__init__('uwb_distance_node')

        # Create publisher for raw unfiltered distances
        self.raw_publisher_ = self.create_publisher(Float32MultiArray, '/raw_uwb_data', 10)

        # Serial connection
        self.ser = None

        # Current raw distances
        self.raw_distances = [0.0, 0.0, 0.0, 0.0]

        # Regex patterns for 4 anchors with JSON format
        self.distance_patterns = [
            re.compile(r'"Addr":"0x0001"[^}]*"D_cm"\s*:\s*(\d+)'),  # d1
            re.compile(r'"Addr":"0x0002"[^}]*"D_cm"\s*:\s*(\d+)'),  # d2
            re.compile(r'"Addr":"0x0003"[^}]*"D_cm"\s*:\s*(\d+)'),  # d3
            re.compile(r'"Addr":"0x0004"[^}]*"D_cm"\s*:\s*(\d+)')   # d4
        ]

        # Initialize serial connection
        if self.connect_to_port():
            self.get_logger().info('UWB Distance Node started successfully')
            # Create timer for reading serial data
            self.timer = self.create_timer(0.05, self.read_serial_data)  # 20Hz
        else:
            self.get_logger().error('Failed to connect to UWB device')

    def find_available_ports(self) -> List[str]:
        """Find all available ACM ports"""
        available_ports = []

        # Check ACM0 to ACM3
        for i in range(4):
            port = f'/dev/ttyACM{i}'
            try:
                test_ser = serial.Serial(port, 115200, timeout=0.1)
                test_ser.close()
                available_ports.append(port)
            except (serial.SerialException, FileNotFoundError):
                continue

        # Also check for USB ports as fallback
        usb_ports = glob.glob('/dev/ttyUSB*')
        for port in usb_ports:
            try:
                test_ser = serial.Serial(port, 115200, timeout=0.1)
                test_ser.close()
                available_ports.append(port)
            except serial.SerialException:
                continue

        return available_ports

    def connect_to_port(self) -> bool:
        """Try to connect to an available port"""
        available_ports = self.find_available_ports()

        if not available_ports:
            self.get_logger().error("No available ACM or USB ports found!")
            return False

        for port in available_ports:
            try:
                self.get_logger().info(f"Attempting to connect to {port}...")
                self.ser = serial.Serial(port, 115200, timeout=1.0, rtscts=True)
                self.get_logger().info(f"Successfully connected to {port}")
                return True
            except serial.SerialException as e:
                self.get_logger().warn(f"Failed to connect to {port}: {e}")
                continue

        self.get_logger().error("Failed to connect to any available port!")
        return False

    def parse_distances(self, data: str) -> List[Optional[float]]:
        """Extract d1, d2, d3, d4 from raw data"""
        distances = [None, None, None, None]

        for i, pattern in enumerate(self.distance_patterns):
            match = pattern.search(data)
            if match:
                try:
                    distance = float(match.group(1))
                    # Basic sanity check
                    if 5 <= distance <= 10000:  # 5cm to 100m range
                        distances[i] = distance
                except ValueError:
                    continue

        return distances

    def read_serial_data(self):
        """Read and process serial data"""
        if not self.ser or not self.ser.is_open:
            return

        try:
            if self.ser.in_waiting > 0:
                data = self.ser.read_all().decode('utf-8', errors='ignore')

                # Process data if it contains JSON format with Block and results
                if '"Block":' in data and '"results":' in data:
                    # Find complete JSON messages
                    messages = []
                    start_pos = 0

                    while True:
                        start = data.find('{"Block":', start_pos)
                        if start == -1:
                            break

                        brace_count = 0
                        end = start
                        in_string = False
                        escape_next = False

                        for i, char in enumerate(data[start:]):
                            pos = start + i
                            if escape_next:
                                escape_next = False
                                continue

                            if char == '\\':
                                escape_next = True
                                continue

                            if char == '"' and not escape_next:
                                in_string = not in_string
                                continue

                            if not in_string:
                                if char == '{':
                                    brace_count += 1
                                elif char == '}':
                                    brace_count -= 1
                                    if brace_count == 0:
                                        end = pos
                                        break

                        if brace_count == 0:
                            messages.append(data[start:end+1])
                            start_pos = end + 1
                        else:
                            break

                    # Process the last complete message
                    if messages:
                        message = messages[-1]
                        raw_distances = self.parse_distances(message)

                        # Update raw distances and publish them
                        raw_updated = False
                        for i, raw_dist in enumerate(raw_distances):
                            if raw_dist is not None:
                                self.raw_distances[i] = raw_dist
                                raw_updated = True

                        if raw_updated:
                            self.publish_raw_distances()

        except Exception as e:
            self.get_logger().error(f"Error reading serial data: {e}")

    def publish_raw_distances(self):
        """Publish raw unfiltered distances to ROS topic"""
        msg = Float32MultiArray()
        msg.data = [float(d) for d in self.raw_distances]
        self.raw_publisher_.publish(msg)

        # Log the published raw data
        self.get_logger().info(
            f"Published distances: d1={self.raw_distances[0]:.1f}, "
            f"d2={self.raw_distances[1]:.1f}, "
            f"d3={self.raw_distances[2]:.1f}, "
            f"d4={self.raw_distances[3]:.1f}"
        )

    def destroy_node(self):
        """Clean up resources"""
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    try:
        uwb_node = UWBNode()
        rclpy.spin(uwb_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'uwb_node' in locals():
            uwb_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
