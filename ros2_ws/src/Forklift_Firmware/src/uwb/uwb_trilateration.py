#!/usr/bin/env python3
# This script subscribes to UWB distance measurements from multiple anchors,
# performs trilateration using a least-squares algorithm to calculate the
# 2D position of a tag at a fixed height, and publishes the calculated
# position as a PointStamped message.
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped
import numpy as np

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

        # Publisher for tag coordinates
        self.position_pub = self.create_publisher(PointStamped, '/uwb_tag_position', 10)
        # Subscriber for anchor distances
        self.distance_sub = self.create_subscription(
            Float32MultiArray,
            '/uwb_distances',
            self.distance_callback,
            10
        )

        self.get_logger().info('UWB Trilateration Node initialized.')
        self.get_logger().info(f'Anchor positions: {self.anchors.tolist()}')
        self.get_logger().info(f'Tag fixed height: {self.tag_z} m')

    def distance_callback(self, msg):
        distances = np.array(msg.data)
        if len(distances) != self.num_anchors:
            self.get_logger().warn(f'Expected {self.num_anchors} distances, got {len(distances)}')
            return

        # Filter valid distances
        valid_idx = distances > 0
        if np.sum(valid_idx) < 3:
            self.get_logger().warn(f'Only {np.sum(valid_idx)} valid distances; need at least 3')
            return

        valid_anchors = self.anchors[valid_idx]
        valid_distances = distances[valid_idx]

        # Compute 2D projected distances
        dz = self.tag_z - valid_anchors[:, 2]
        r_squared = valid_distances**2 - dz**2
        if np.any(r_squared < 0):
            self.get_logger().warn('Negative 2D distance squared; invalid measurements')
            return

        try:
            # Set up linear system
            A = []
            b = []
            ref_anchor = valid_anchors[0]
            ref_r_sq = r_squared[0]
            for i in range(1, len(valid_anchors)):
                anchor = valid_anchors[i]
                r_sq = r_squared[i]
                A.append([2 * (ref_anchor[0] - anchor[0]), 2 * (ref_anchor[1] - anchor[1])])
                b.append(ref_r_sq - r_sq + anchor[0]**2 + anchor[1]**2 - ref_anchor[0]**2 - ref_anchor[1]**2)

            A = np.array(A)
            b = np.array(b)

            # Solve using least squares
            xy, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
            x, y = xy

            # Publish position
            point_msg = PointStamped()
            point_msg.header.stamp = self.get_clock().now().to_msg()
            point_msg.header.frame_id = 'map'
            point_msg.point.x = float(x)
            point_msg.point.y = float(y)
            point_msg.point.z = self.tag_z
            self.position_pub.publish(point_msg)
            self.get_logger().info(f'Published tag position: ({x:.2f}, {y:.2f}, {self.tag_z:.2f})')

        except np.linalg.LinAlgError:
            self.get_logger().error('Failed to solve: anchors may be collinear')
        except Exception as e:
            self.get_logger().error(f'Error in computation: {str(e)}')

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
