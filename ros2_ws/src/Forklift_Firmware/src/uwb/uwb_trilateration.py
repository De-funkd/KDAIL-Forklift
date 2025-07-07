#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped
import numpy as np
from typing import List

# Hardcoded coordinates of the anchors [x, y, z]
# Users should replace these with their actual anchor coordinates.
ANCHOR_COORDS: List[List[float]] = [
    [0.0, 0.0, 2.10],    # Anchor 1 (ID 1)
    [4.486, 0.0, 2.10],    # Anchor 2 (ID 2)
    [3.942, 5.07, 2.10],    # Anchor 3 (ID 3)
    [0.0, 5.07, 2.10]     # Anchor 4 (ID 4)
]

# Hardcoded height of the UWB tag.
# Users should replace this with the actual height of the tag.
TAG_Z: float = 1.0

class UWBTrilaterationNode(Node):
    """
    Calculates the (x, y) coordinates of a UWB tag using trilateration
    based on distances from fixed anchors.
    """
    def __init__(self):
        super().__init__('uwb_trilateration_node')

        # Anchor and tag configuration
        self.anchors = np.array(ANCHOR_COORDS)
        self.tag_z = TAG_Z
        self.num_anchors = len(ANCHOR_COORDS)

        # Publishers
        self.tag_coord_publisher = self.create_publisher(PointStamped, '/uwb_coordinates', 10)
        self.anchor_coord_publisher = self.create_publisher(Float32MultiArray, '/anchor_coordinates', 10)

        # Subscriber
        self.distance_subscription = self.create_subscription(
            Float32MultiArray,
            '/uwb_distances',
            self.distance_callback,
            10)

        # Timer to periodically publish anchor coordinates
        self.anchor_publish_timer = self.create_timer(1.0, self.publish_anchor_coords)

        self.get_logger().info('UWB Trilateration Node started.')
        self.get_logger().info(f'Anchors at: {self.anchors.tolist()}')
        self.get_logger().info(f'Tag Z height: {self.tag_z}')


    def publish_anchor_coords(self):
        """Publishes the hardcoded anchor coordinates."""
        msg = Float32MultiArray()
        msg.data = self.anchors.flatten().tolist()
        self.anchor_coord_publisher.publish(msg)

    def distance_callback(self, msg: Float32MultiArray):
        """
        Callback for receiving UWB distances and performing trilateration.
        """
        distances = np.array(msg.data)

        # Filter out anchors with no signal (distance <= 0)
        valid_indices = np.where(distances > 0)[0]

        if len(valid_indices) < 3:
            self.get_logger().warn(
                f"Need at least 3 valid anchor distances for trilateration, but got {len(valid_indices)}. Skipping."
            )
            return

        valid_anchors = self.anchors[valid_indices]
        valid_distances = distances[valid_indices]

        # Use the first valid anchor as the reference point
        ref_anchor = valid_anchors[0]
        ref_dist = valid_distances[0]

        # Calculate the squared projected 2D distance for the reference anchor
        ref_dist_2d_sq = ref_dist**2 - (self.tag_z - ref_anchor[2])**2
        if ref_dist_2d_sq < 0:
            self.get_logger().warn(f"Measurement error: squared 2D distance for reference anchor {valid_indices[0]+1} is negative. Skipping.")
            return

        # Set up the system of linear equations (A * x = B)
        A = []
        B = []
        for i in range(1, len(valid_anchors)):
            anchor_i = valid_anchors[i]
            dist_i = valid_distances[i]

            dist_2d_sq_i = dist_i**2 - (self.tag_z - anchor_i[2])**2
            if dist_2d_sq_i < 0:
                self.get_logger().warn(f"Measurement error: squared 2D distance for anchor {valid_indices[i]+1} is negative. Skipping this anchor.")
                continue

            A.append([2 * (ref_anchor[0] - anchor_i[0]), 2 * (ref_anchor[1] - anchor_i[1])])
            B.append(ref_dist_2d_sq - dist_2d_sq_i - (ref_anchor[0]**2 - anchor_i[0]**2) - (ref_anchor[1]**2 - anchor_i[1]**2))

        if len(A) < 2:
            self.get_logger().warn("Not enough valid equations to solve for (x, y). Need at least 2 (i.e., 3 anchors).")
            return

        try:
            # Use least squares to solve for the tag's (x, y) position
            A_matrix = np.array(A)
            B_vector = np.array(B)
            result, _, _, _ = np.linalg.lstsq(A_matrix, B_vector, rcond=None)
            tag_x, tag_y = result[0], result[1]

            # Publish the calculated coordinates
            point_msg = PointStamped()
            point_msg.header.stamp = self.get_clock().now().to_msg()
            point_msg.header.frame_id = 'uwb_map_frame'  # Or a more appropriate frame_id
            point_msg.point.x = tag_x
            point_msg.point.y = tag_y
            point_msg.point.z = self.tag_z
            self.tag_coord_publisher.publish(point_msg)
            self.get_logger().debug(f"Calculated Tag Position: ({tag_x:.3f}, {tag_y:.3f}, {self.tag_z:.3f})")

        except np.linalg.LinAlgError:
            self.get_logger().error("Linear algebra error: Could not solve the system of equations. Anchors might be collinear.")
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred during calculation: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = UWBTrilaterationNode()
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
