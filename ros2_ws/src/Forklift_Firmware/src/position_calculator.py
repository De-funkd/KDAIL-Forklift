#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
import numpy as np
from typing import Optional, Tuple

class UWBPositionCalculator(Node):
    def __init__(self):
        super().__init__('uwb_position_calculator')

        # Default anchor positions (in cm, now with x, y, z)
        self.anchor_positions = {
            1: (0.0, 0.0, 0.0),      # Anchor 1 at origin
            2: (700.0, 0.0, 0.0),    # Anchor 2 at (Xmax, 0, 0)
            3: (700.0, 700.0, 0.0),  # Anchor 3 at (Xmax, Ymax, 0)
            4: (0.0, 700.0, 0.0)     # Anchor 4 at (0, Ymax, 0)
        }

        # Subscribe to UWB raw data
        self.raw_subscriber = self.create_subscription(
            Float32MultiArray,
            '/raw_uwb_data',
            self.raw_callback,
            10
        )

        # Subscribe to anchor positions
        self.anchor_subscriber = self.create_subscription(
            Float32MultiArray,
            '/anchor_positions',
            self.anchor_callback,
            10
        )

        # Publisher for position
        self.position_publisher = self.create_publisher(Point, '/uwb_xy_coords', 10)
        self.get_logger().info('UWB Position Calculator Node started')
        self.get_logger().info(f'Anchor positions: {self.anchor_positions}')

    def anchor_callback(self, msg):
        """Callback for updated anchor positions"""
        if len(msg.data) == 12:  # Expecting x, y, z for 4 anchors
            for i in range(4):
                x = msg.data[i * 3]
                y = msg.data[i * 3 + 1]
                z = msg.data[i * 3 + 2]
                self.anchor_positions[i + 1] = (x, y, z)
            self.get_logger().info(f'Updated anchor positions: {self.anchor_positions}')

    def raw_callback(self, msg):
        """Callback for raw UWB distances"""
        if len(msg.data) >= 4:
            distances = list(msg.data[:4])
            position = self.calculate_position(distances)
            if position:
                self.publish_position(position)

    def trilaterate_least_squares(self, distances) -> Optional[Tuple[float, float, float]]:
        """Calculate position using least squares trilateration with all 4 anchors in 3D"""
        anchors = np.array(list(self.anchor_positions.values()))
        dists = np.array(distances)
        x0, y0, z0 = 350.0, 350.0, 0.0  # Initial guess at center
        x, y, z = x0, y0, z0

        for _ in range(10):  # Reduced iterations for faster processing
            calculated_dists = np.sqrt((anchors[:, 0] - x)**2 + (anchors[:, 1] - y)**2 + (anchors[:, 2] - z)**2)
            residuals = calculated_dists - dists
            J = np.zeros((4, 3))

            for i in range(4):
                if calculated_dists[i] > 0:
                    J[i, 0] = (x - anchors[i, 0]) / calculated_dists[i]
                    J[i, 1] = (y - anchors[i, 1]) / calculated_dists[i]
                    J[i, 2] = (z - anchors[i, 2]) / calculated_dists[i]

            try:
                delta = np.linalg.lstsq(J, -residuals, rcond=None)[0]
                x += delta[0]
                y += delta[1]
                z += delta[2]
                if np.linalg.norm(delta) < 0.01:
                    break
            except np.linalg.LinAlgError:
                return None

        return (x, y, z)

    def calculate_position(self, distances):
        """Calculate position from distances"""
        position = self.trilaterate_least_squares(distances)
        return position

    def publish_position(self, position):
        """Publish position to ROS topic"""
        msg = Point()
        msg.x = position[0]
        msg.y = position[1]
        msg.z = position[2]
        self.position_publisher.publish(msg)
        self.get_logger().info(f'Published position: x={position[0]:.2f}, y={position[1]:.2f}, z={position[2]:.2f}')

def main(args=None):
    rclpy.init(args=args)
    try:
        position_calculator = UWBPositionCalculator()
        rclpy.spin(position_calculator)
    except KeyboardInterrupt:
        pass
    finally:
        if 'position_calculator' in locals():
            position_calculator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
