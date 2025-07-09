#!/usr/bin/env python3
# This script subscribes to UWB distance measurements from multiple anchors,
# performs trilateration using simple trigonometry with pairs of adjacent anchors,
# calculates multiple position estimates, averages them, and publishes the result.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped
import numpy as np
import math

# Anchor coordinates [x, y, z] in meters
ANCHORS = [
    [0.0, 0.0, 2.0],      # Anchor 1
    [5.0, 0.0, 2.0],      # Anchor 2
    [5.0, 5.0, 2.0],      # Anchor 3
    [0.0, 5.0, 2.0]       # Anchor 4
]

# Fixed tag height in meters
TAG_HEIGHT = 1.0

class UWBTrilaterationNode(Node):
    def __init__(self):
        super().__init__('uwb_trilateration_node')
        self.anchors = np.array(ANCHORS)
        self.tag_z = TAG_HEIGHT
        self.num_anchors = len(ANCHORS)

        # Publisher for individual pairwise calculations
        self.pairwise_pub = self.create_publisher(PointStamped, '/uwb_pairwise_position', 10)

        # Publisher for final averaged position
        self.position_pub = self.create_publisher(PointStamped, '/uwb_tag_position', 10)

        # Subscriber for anchor distances
        self.distance_sub = self.create_subscription(
            Float32MultiArray,
            '/uwb_distances',
            self.distance_callback,
            10
        )

        self.get_logger().info('UWB Trilateration Node initialized with pairwise trigonometry.')
        self.get_logger().info(f'Anchor positions: {self.anchors.tolist()}')
        self.get_logger().info(f'Tag fixed height: {self.tag_z} m')

    def calculate_position_from_pair(self, anchor1, anchor2, d1, d2):
        """
        Calculate 3D position using two anchors and their distances using trigonometry.
        Returns (x, y, z) coordinates or None if calculation fails.
        """
        try:
            # Calculate 2D projected distances (removing z-component)
            dz1 = self.tag_z - anchor1[2]
            dz2 = self.tag_z - anchor2[2]

            # Check if distances are valid
            if d1**2 - dz1**2 < 0 or d2**2 - dz2**2 < 0:
                return None

            r1 = math.sqrt(d1**2 - dz1**2)  # 2D distance from anchor1
            r2 = math.sqrt(d2**2 - dz2**2)  # 2D distance from anchor2

            # Distance between anchors (2D)
            dx = anchor2[0] - anchor1[0]
            dy = anchor2[1] - anchor1[1]
            d = math.sqrt(dx**2 + dy**2)

            # Check if solution is possible
            if d > r1 + r2 or d < abs(r1 - r2):
                return None

            # Use law of cosines to find angle
            cos_angle = (r1**2 + d**2 - r2**2) / (2 * r1 * d)

            # Clamp to valid range for acos
            cos_angle = max(-1, min(1, cos_angle))
            angle = math.acos(cos_angle)

            # Calculate angle of line between anchors
            base_angle = math.atan2(dy, dx)

            # Two possible positions (we'll use both and average later if needed)
            angle1 = base_angle + angle
            angle2 = base_angle - angle

            # Calculate positions
            x1 = anchor1[0] + r1 * math.cos(angle1)
            y1 = anchor1[1] + r1 * math.sin(angle1)

            x2 = anchor1[0] + r1 * math.cos(angle2)
            y2 = anchor1[1] + r1 * math.sin(angle2)

            # For simplicity, return the first solution
            # In practice, you might want to use additional constraints to choose
            return (x1, y1, self.tag_z)

        except Exception as e:
            self.get_logger().warn(f'Error in pairwise calculation: {str(e)}')
            return None

    def distance_callback(self, msg):
        distances = np.array(msg.data)

        if len(distances) != self.num_anchors:
            self.get_logger().warn(f'Expected {self.num_anchors} distances, got {len(distances)}')
            return

        # Filter valid distances
        if not all(d > 0 for d in distances):
            self.get_logger().warn('Some distances are invalid (<=0)')
            return

        # Calculate positions using pairs of adjacent anchors
        pairwise_positions = []
        valid_calculations = 0

        # Define pairs: (1,2), (2,3), (3,4), (4,1)
        pairs = [(0, 1), (1, 2), (2, 3), (3, 0)]

        for i, (idx1, idx2) in enumerate(pairs):
            anchor1 = self.anchors[idx1]
            anchor2 = self.anchors[idx2]
            d1 = distances[idx1]
            d2 = distances[idx2]

            position = self.calculate_position_from_pair(anchor1, anchor2, d1, d2)

            if position is not None:
                pairwise_positions.append(position)
                valid_calculations += 1

                # Publish individual pairwise calculation
                pairwise_msg = PointStamped()
                pairwise_msg.header.stamp = self.get_clock().now().to_msg()
                pairwise_msg.header.frame_id = 'map'
                pairwise_msg.point.x = float(position[0])
                pairwise_msg.point.y = float(position[1])
                pairwise_msg.point.z = float(position[2])
                self.pairwise_pub.publish(pairwise_msg)

                self.get_logger().info(f'Pair {idx1+1}-{idx2+1}: ({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f})')
            else:
                self.get_logger().warn(f'Failed to calculate position for pair {idx1+1}-{idx2+1}')

        # Calculate average position if we have valid calculations
        if valid_calculations > 0:
            avg_x = sum(pos[0] for pos in pairwise_positions) / len(pairwise_positions)
            avg_y = sum(pos[1] for pos in pairwise_positions) / len(pairwise_positions)
            avg_z = self.tag_z  # Z is fixed

            # Publish averaged position
            final_msg = PointStamped()
            final_msg.header.stamp = self.get_clock().now().to_msg()
            final_msg.header.frame_id = 'map'
            final_msg.point.x = float(avg_x)
            final_msg.point.y = float(avg_y)
            final_msg.point.z = float(avg_z)
            self.position_pub.publish(final_msg)

            self.get_logger().info(f'AVERAGED position ({valid_calculations} calculations): ({avg_x:.2f}, {avg_y:.2f}, {avg_z:.2f})')
        else:
            self.get_logger().error('No valid position calculations from any pair')

def main(args=None):
    rclpy.init(args=args)
    node = UWBTrilaterationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
