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

# Updated anchor coordinates based on your description
ANCHORS = [
    [0.0, 0.0, 0.2],        # Anchor 0
    [0.0, 2.085, 0.2],      # Anchor 1
    [2.02, 2.085, 0.2],     # Anchor 2
    [2.02, 0.0, 0.2]        # Anchor 3
]

# Fixed tag height in meters
TAG_HEIGHT = 0.14


class UWBTrilaterationNode(Node):
    def __init__(self):
        super().__init__('uwb_trilateration_node')
        self.anchors = np.array(ANCHORS)
        self.tag_z = TAG_HEIGHT
        self.num_anchors = len(ANCHORS)

        # Publisher for individual pairwise calculations
        self.pairwise_pub = self.create_publisher(
            PointStamped, '/uwb_pairwise_position', 10)

        # Publisher for final averaged position
        self.position_pub = self.create_publisher(
            PointStamped, '/uwb_tag_position', 10)

        # Subscriber for anchor distances
        self.distance_sub = self.create_subscription(
            Float32MultiArray,
            '/uwb_distances',
            self.distance_callback,
            10
        )

        self.get_logger().info(
            'UWB Trilateration Node initialized with improved pairwise trigonometry.')
        self.get_logger().info(f'Anchor positions: {self.anchors.tolist()}')
        self.get_logger().info(f'Tag fixed height: {self.tag_z} m')

    def calculate_position_from_pair(self, anchor1, anchor2, d1, d2):
        """
        Calculate 3D position using two anchors and their distances using trigonometry.
        Returns both possible solutions and selects the more reasonable one.
        """
        try:
            # Calculate 2D projected distances (removing z-component)
            dz1 = self.tag_z - anchor1[2]
            dz2 = self.tag_z - anchor2[2]

            # Check if distances are valid
            if d1**2 - dz1**2 < 0 or d2**2 - dz2**2 < 0:
                self.get_logger().warn(
                    f'Invalid distance: d1={d1:.3f}, dz1={dz1:.3f}, d2={d2:.3f}, dz2={dz2:.3f}')
                return None

            r1 = math.sqrt(d1**2 - dz1**2)  # 2D distance from anchor1
            r2 = math.sqrt(d2**2 - dz2**2)  # 2D distance from anchor2

            # Distance between anchors (2D)
            dx = anchor2[0] - anchor1[0]
            dy = anchor2[1] - anchor1[1]
            d = math.sqrt(dx**2 + dy**2)

            # Check if solution is geometrically possible
            if d > r1 + r2:
                self.get_logger().warn(
                    f'Circles don\'t intersect: d={d:.3f} > r1+r2={r1+r2:.3f}')
                return None

            if d < abs(r1 - r2):
                self.get_logger().warn(
                    f'One circle inside another: d={d:.3f} < |r1-r2|={abs(r1-r2):.3f}')
                return None

            # Special case: anchors are vertically aligned (same x-coordinate)
            if abs(dx) < 1e-6:
                # Vertical line between anchors
                if d == 0:  # Same anchor position
                    return None

                # Calculate x-coordinate of intersections
                a = (r1**2 - r2**2 + d**2) / (2 * d)
                h_squared = r1**2 - a**2

                if h_squared < 0:
                    return None

                h = math.sqrt(h_squared)

                # Intersection point on the line between anchors
                intersection_y = anchor1[1] + a * (dy / d)

                # Two possible x-coordinates
                x1 = anchor1[0] + h
                x2 = anchor1[0] - h

                # Choose the solution within the rectangle bounds
                if 0 <= x1 <= 2.02:
                    return (x1, intersection_y, self.tag_z)
                elif 0 <= x2 <= 2.02:
                    return (x2, intersection_y, self.tag_z)
                else:
                    # Return the closer one to the rectangle center
                    center_x = 1.01
                    if abs(x1 - center_x) < abs(x2 - center_x):
                        return (x1, intersection_y, self.tag_z)
                    else:
                        return (x2, intersection_y, self.tag_z)

            # Special case: anchors are horizontally aligned (same y-coordinate)
            elif abs(dy) < 1e-6:
                # Horizontal line between anchors
                if d == 0:  # Same anchor position
                    return None

                # Calculate y-coordinate of intersections
                a = (r1**2 - r2**2 + d**2) / (2 * d)
                h_squared = r1**2 - a**2

                if h_squared < 0:
                    return None

                h = math.sqrt(h_squared)

                # Intersection point on the line between anchors
                intersection_x = anchor1[0] + a * (dx / d)

                # Two possible y-coordinates
                y1 = anchor1[1] + h
                y2 = anchor1[1] - h

                # Choose the solution within the rectangle bounds
                if 0 <= y1 <= 2.085:
                    return (intersection_x, y1, self.tag_z)
                elif 0 <= y2 <= 2.085:
                    return (intersection_x, y2, self.tag_z)
                else:
                    # Return the closer one to the rectangle center
                    center_y = 1.0425
                    if abs(y1 - center_y) < abs(y2 - center_y):
                        return (intersection_x, y1, self.tag_z)
                    else:
                        return (intersection_x, y2, self.tag_z)

            # General case: use law of cosines
            else:
                cos_angle = (r1**2 + d**2 - r2**2) / (2 * r1 * d)

                # Clamp to valid range for acos
                cos_angle = max(-1, min(1, cos_angle))
                angle = math.acos(cos_angle)

                # Calculate angle of line between anchors
                base_angle = math.atan2(dy, dx)

                # Two possible positions
                angle1 = base_angle + angle
                angle2 = base_angle - angle

                # Calculate positions
                x1 = anchor1[0] + r1 * math.cos(angle1)
                y1 = anchor1[1] + r1 * math.sin(angle1)

                x2 = anchor1[0] + r1 * math.cos(angle2)
                y2 = anchor1[1] + r1 * math.sin(angle2)

                # Choose the solution that's more likely to be within the rectangle
                # or closer to the rectangle center
                center_x, center_y = 1.01, 1.0425

                dist1_to_center = math.sqrt(
                    (x1 - center_x)**2 + (y1 - center_y)**2)
                dist2_to_center = math.sqrt(
                    (x2 - center_x)**2 + (y2 - center_y)**2)

                # Check if solutions are within reasonable bounds
                in_bounds1 = (-0.5 <= x1 <= 2.5) and (-0.5 <= y1 <= 2.6)
                in_bounds2 = (-0.5 <= x2 <= 2.5) and (-0.5 <= y2 <= 2.6)

                if in_bounds1 and in_bounds2:
                    # Both in bounds, choose closer to center
                    if dist1_to_center < dist2_to_center:
                        return (x1, y1, self.tag_z)
                    else:
                        return (x2, y2, self.tag_z)
                elif in_bounds1:
                    return (x1, y1, self.tag_z)
                elif in_bounds2:
                    return (x2, y2, self.tag_z)
                else:
                    # Neither in bounds, choose closer to center
                    if dist1_to_center < dist2_to_center:
                        return (x1, y1, self.tag_z)
                    else:
                        return (x2, y2, self.tag_z)

        except Exception as e:
            self.get_logger().warn(f'Error in pairwise calculation: {str(e)}')
            return None

    def distance_callback(self, msg):
        distances = np.array(msg.data)

        if len(distances) != self.num_anchors:
            self.get_logger().warn(
                f'Expected {self.num_anchors} distances, got {len(distances)}')
            return

        # Filter valid distances
        if not all(d > 0 for d in distances):
            self.get_logger().warn('Some distances are invalid (<=0)')
            return

        # Calculate positions using pairs of adjacent anchors
        pairwise_positions = []
        valid_calculations = 0

        # Define pairs: (0,1), (1,2), (2,3), (3,0) - adjacent pairs only
        pairs = [(0, 1), (1, 2), (2, 3), (3, 0)]
        # Equal weights for all adjacent pairs
        pair_weights = [1.0, 1.0, 1.0, 1.0]

        for i, (idx1, idx2) in enumerate(pairs):
            anchor1 = self.anchors[idx1]
            anchor2 = self.anchors[idx2]
            d1 = distances[idx1]
            d2 = distances[idx2]

            position = self.calculate_position_from_pair(
                anchor1, anchor2, d1, d2)

            if position is not None:
                # Equal weight for all adjacent pairs
                weight = pair_weights[i]
                weighted_position = (position[0], position[1], position[2])
                pairwise_positions.append(weighted_position)
                valid_calculations += 1

                # Publish individual pairwise calculation
                pairwise_msg = PointStamped()
                pairwise_msg.header.stamp = self.get_clock().now().to_msg()
                pairwise_msg.header.frame_id = 'map'
                pairwise_msg.point.x = float(position[0])
                pairwise_msg.point.y = float(position[1])
                pairwise_msg.point.z = float(position[2])
                self.pairwise_pub.publish(pairwise_msg)

                self.get_logger().info(
                    f'Pair {idx1}-{idx2}: ({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f})')
            else:
                self.get_logger().warn(
                    f'Failed to calculate position for pair {idx1}-{idx2}')

        # Calculate weighted average position if we have valid calculations
        if valid_calculations > 0:
            avg_x = sum(pos[0]
                        for pos in pairwise_positions) / valid_calculations
            avg_y = sum(pos[1]
                        for pos in pairwise_positions) / valid_calculations
            avg_z = self.tag_z  # Z is fixed

            # Publish averaged position
            final_msg = PointStamped()
            final_msg.header.stamp = self.get_clock().now().to_msg()
            final_msg.header.frame_id = 'map'
            final_msg.point.x = float(avg_x)
            final_msg.point.y = float(avg_y)
            final_msg.point.z = float(avg_z)
            self.position_pub.publish(final_msg)

            self.get_logger().info(
                f'AVERAGED position ({valid_calculations} calculations): ({avg_x:.2f}, {avg_y:.2f}, {avg_z:.2f})')
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
