#!/usr/bin/env python3
"""
Distance Reader Script for Ranging System
Reads serial data from ACM0 and extracts distance measurements for responders 2, 3, 4, 5
"""

import serial
import re
import time
from datetime import datetime
import sys

class DistanceReader:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200, timeout=1):
        """
        Initialize the distance reader
        
        Args:
            port: Serial port (default: /dev/ttyACM0)
            baudrate: Baud rate (default: 115200)
            timeout: Read timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn = None
        
        # Store latest distances for each responder
        self.distances = {
            'd1': None,
            'd2': None, 
            'd3': None,
            'd4': None
        }
        
        # Regular expressions for parsing
        self.distance_pattern = re.compile(r'Distance to responder (\d+): ([\d.]+) m')
        self.no_response_pattern = re.compile(r'Did not receive response from responder (\d+)')
        
    def connect(self):
        """Connect to serial port"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            print(f"Connected to {self.port} at {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            print(f"Failed to connect to {self.port}: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from serial port"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("Disconnected from serial port")
    
    def parse_line(self, line):
        """
        Parse a line of serial data and extract distance information
        
        Args:
            line: String line from serial data
            
        Returns:
            tuple: (responder_id, distance) or (responder_id, None) if no response
        """
        # Check for successful distance measurement
        distance_match = self.distance_pattern.search(line)
        if distance_match:
            responder_id = int(distance_match.group(1))
            distance = float(distance_match.group(2))
            return responder_id, distance
        
        # Check for failed response
        no_response_match = self.no_response_pattern.search(line)
        if no_response_match:
            responder_id = int(no_response_match.group(1))
            return responder_id, None
        
        return None, None
    
    def update_distances(self, responder_id, distance):
        """Update the distance for a specific responder"""
        if responder_id in [2, 3, 4, 5]:
            # Map responder IDs 2,3,4,5 to display labels d1,d2,d3,d4
            key = f'd{responder_id - 1}'
            self.distances[key] = distance
    
    def display_distances(self):
        """Display current distances in a formatted way"""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        
        print(f"\r[{timestamp}] ", end="")
        for responder in ['d1', 'd2', 'd3', 'd4']:
            distance = self.distances[responder]
            if distance is not None:
                print(f"{responder}:{distance:>6.2f}m ", end="")
            else:
                print(f"{responder}:  ----  ", end="")
        print("", end="", flush=True)
    
    def run(self):
        """Main loop to read serial data and display distances"""
        if not self.connect():
            return
        
        print("Reading distance measurements...")
        print("Press Ctrl+C to stop\n")
        
        # Print header
        print("Format: d1=Responder2, d2=Responder3, d3=Responder4, d4=Responder5")
        print("-" * 60)
        
        try:
            while True:
                if self.serial_conn.in_waiting > 0:
                    try:
                        # Read line from serial port
                        line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                        
                        if line:
                            # Parse the line for distance information
                            responder_id, distance = self.parse_line(line)
                            
                            if responder_id is not None:
                                # Update distances and display
                                self.update_distances(responder_id, distance)
                                self.display_distances()
                                
                                # Also print the raw successful measurements
                                if distance is not None:
                                    print(f"\n✓ Responder {responder_id}: {distance:.2f}m")
                                else:
                                    print(f"\n✗ Responder {responder_id}: No response")
                    
                    except UnicodeDecodeError:
                        # Skip lines that can't be decoded
                        pass
                
                time.sleep(0.01)  # Small delay to prevent excessive CPU usage
                
        except KeyboardInterrupt:
            print("\n\nStopping distance reader...")
        
        except Exception as e:
            print(f"\nError: {e}")
        
        finally:
            self.disconnect()

def main():
    """Main function"""
    # Parse command line arguments for port and baudrate if needed
    port = '/dev/ttyACM0'
    baudrate = 115200
    
    if len(sys.argv) > 1:
        port = sys.argv[1]
    if len(sys.argv) > 2:
        baudrate = int(sys.argv[2])
    
    print(f"Distance Reader for Ranging System")
    print(f"Port: {port}, Baudrate: {baudrate}")
    print("=" * 50)
    
    # Create and run the distance reader
    reader = DistanceReader(port=port, baudrate=baudrate)
    reader.run()

if __name__ == "__main__":
    main()
