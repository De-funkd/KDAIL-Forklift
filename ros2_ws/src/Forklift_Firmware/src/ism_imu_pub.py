#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from smbus2 import SMBus
import struct
import math
import time

# I2C constants
I2C_ADDRESS = 0x6B
WHO_AM_I_REG = 0x0F
WHO_AM_I_EXPECTED = 0x6B
CTRL1_XL = 0x10
CTRL2_G = 0x11
OUTX_L_G = 0x22
OUTX_L_XL = 0x28

def read_raw_data(bus, reg_addr):
    data = bus.read_i2c_block_data(I2C_ADDRESS, reg_addr, 6)
    x, y, z = struct.unpack('<hhh', bytes(data))
    return x, y, z

def initialize_sensor(bus):
    who_am_i = bus.read_byte_data(I2C_ADDRESS, WHO_AM_I_REG)
    if who_am_i != WHO_AM_I_EXPECTED:
        raise RuntimeError(f"WHO_AM_I mismatch: expected 0x{WHO_AM_I_EXPECTED:02X}, got 0x{who_am_i:02X}")

    bus.write_byte_data(I2C_ADDRESS, CTRL1_XL, 0x40)  # Accelerometer: 104 Hz, 2g
    bus.write_byte_data(I2C_ADDRESS, CTRL2_G, 0x40)   # Gyroscope: 104 Hz, 250 dps
    time.sleep(0.1)

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.publisher_ = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz

        self.bus = SMBus(1)
        initialize_sensor(self.bus)
        self.get_logger().info("IMU Node started and sensor initialized.")

    def timer_callback(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Read raw IMU data
        ax_raw, ay_raw, az_raw = read_raw_data(self.bus, OUTX_L_XL)
        gx_raw, gy_raw, gz_raw = read_raw_data(self.bus, OUTX_L_G)

        # Convert to SI units
        ax = ax_raw * 0.00981     # mg to m/s² (2g range = 0.061 mg/LSB approx → tuned to 0.00981 here)
        ay = ay_raw * 0.00981
        az = az_raw * 0.00981

        gx = gx_raw * 0.0000017453  # mdps to rad/s (250 dps range = 8.75 mdps/LSB → tuned to rad/s)
        gy = gy_raw * 0.0000017453
        gz = gz_raw * 0.0000017453

        # Orientation (not available → set as 0, or optionally use a complementary filter or external filter later)
        imu_msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Angular velocity and linear acceleration
        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz

        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az

        # Publish
        self.publisher_.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.bus.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
