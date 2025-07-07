#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import json
import glob
import time
from typing import List

ANCHOR_IDS = {
    "0x0001": 0,
    "0x0002": 1,
    "0x0003": 2,
    "0x0004": 3
}

class UWBRawPublisher(Node):
    def __init__(self):
        super().__init__('uwb_raw_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/raw_uwb_readings', 10)
        self.ser = None
        self.raw_distances = [0.0, 0.0, 0.0, 0.0]
        self.buffer = ""  # Buffer for incomplete JSON data

        if self.connect_to_port():
            self.get_logger().info('Connected to UWB device')
            self.timer = self.create_timer(0.05, self.read_serial_data)  # 20 Hz
        else:
            self.get_logger().error('Failed to connect to UWB device')

    def find_available_ports(self) -> List[str]:
        ports = []
        # Check ACM ports first (most common for UWB devices)
        for i in range(10):  # Check more ports
            port = f'/dev/ttyACM{i}'
            try:
                test_ser = serial.Serial(port, 115200, timeout=0.1)
                test_ser.close()
                ports.append(port)
                self.get_logger().info(f'Found available port: {port}')
            except:
                continue

        # Also check USB ports
        ports += glob.glob('/dev/ttyUSB*')

        if not ports:
            self.get_logger().warning('No serial ports found')

        return ports

    def connect_to_port(self) -> bool:
        ports = self.find_available_ports()

        for port in ports:
            try:
                self.get_logger().info(f'Trying to connect to {port}')
                self.ser = serial.Serial(port, 115200, timeout=1.0)

                # Test the connection by trying to read some data
                time.sleep(0.5)  # Give it time to settle
                if self.ser.in_waiting > 0:
                    test_data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                    if '"Block":' in test_data or '"results":' in test_data:
                        self.get_logger().info(f'Successfully connected to {port}')
                        return True

                # If no immediate data, still consider it connected
                self.get_logger().info(f'Connected to {port}, waiting for data...')
                return True

            except serial.SerialException as e:
                self.get_logger().warning(f'Failed to connect to {port}: {e}')
                continue

        return False

    def read_serial_data(self):
        if not self.ser or not self.ser.is_open:
            self.get_logger().warning('Serial port is not open')
            return

        try:
            if self.ser.in_waiting > 0:
                # Read available data
                new_data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                self.buffer += new_data

                # Process complete JSON objects
                self.process_buffer()

        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")

    def process_buffer(self):
        # Look for complete JSON objects in the buffer
        while True:
            start_idx = self.buffer.find('{"Block":')
            if start_idx == -1:
                break

            # Find the end of this JSON object
            brace_count = 0
            end_idx = -1

            for i in range(start_idx, len(self.buffer)):
                if self.buffer[i] == '{':
                    brace_count += 1
                elif self.buffer[i] == '}':
                    brace_count -= 1
                    if brace_count == 0:
                        end_idx = i + 1
                        break

            if end_idx == -1:
                # Incomplete JSON, keep in buffer
                self.buffer = self.buffer[start_idx:]
                break

            # Extract and parse the JSON object
            json_str = self.buffer[start_idx:end_idx]
            self.buffer = self.buffer[end_idx:]

            try:
                json_data = json.loads(json_str)
                self.parse_and_publish(json_data)
            except json.JSONDecodeError as e:
                self.get_logger().warning(f"JSON decode error: {e}")
                self.get_logger().debug(f"Problematic JSON: {json_str[:100]}...")

            # Keep buffer size reasonable
            if len(self.buffer) > 10000:
                self.buffer = self.buffer[-1000:]

    def parse_and_publish(self, json_data):
        distances = [0.0, 0.0, 0.0, 0.0]
        valid_readings = 0

        for result in json_data.get("results", []):
            addr = result.get("Addr")
            status = result.get("Status")
            dist = result.get("D_cm")

            if status == "Ok" and addr in ANCHOR_IDS and dist is not None:
                idx = ANCHOR_IDS[addr]
                distances[idx] = float(dist)
                valid_readings += 1

        # Only publish if we have at least one valid reading
        if valid_readings > 0:
            self.raw_distances = distances
            msg = Float32MultiArray()
            msg.data = self.raw_distances
            self.publisher_.publish(msg)

            block_num = json_data.get("Block", "unknown")
            self.get_logger().info(
                f"Block {block_num}: d1={distances[0]:.1f}cm, d2={distances[1]:.1f}cm, "
                f"d3={distances[2]:.1f}cm, d4={distances[3]:.1f}cm ({valid_readings} valid)"
            )
        else:
            self.get_logger().debug("No valid readings in this block")

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = UWBRawPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
