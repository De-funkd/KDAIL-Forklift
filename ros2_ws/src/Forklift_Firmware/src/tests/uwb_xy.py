#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import PointStamped
import numpy as np
from typing import List, Tuple, Optional
import math
import time

class UWBPositionCalculator(Node):
    def __init__(self):
        super().__init__('uwb_position_calculator')

        # =============================================================================
        # CONFIGURATION - UPDATE THESE VALUES WITH YOUR ANCHOR POSITIONS
        # =============================================================================

        # Anchor positions (x, y, z) in meters
        # TODO: Replace with your actual anchor coordinates
        self.anchor_positions = [
            [0.0, 0.0, 2.10],    # Anchor 1 (ID 1)
            [4.486, 0.0, 2.10],    # Anchor 2 (ID 2)
            [3.942, 5.07, 2.10],    # Anchor 3 (ID 3)
            [0.0, 5.07, 2.10]     # Anchor 4 (ID 4)
        ]

        # UWB tag height (z-coordinate) in meters
        # TODO: Replace with your actual UWB height
        self.uwb_height = 1.0  # meters

        # =============================================================================
        # END CONFIGURATION
        # =============================================================================

        # Convert to numpy array for easier computation
        self.anchors = np.array(self.anchor_positions)

        # Subscriber for UWB distances
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/uwb_distances',
            self.distance_callback,
            10
        )

        # Subscriber for UWB status
        self.status_subscription = self.create_subscription(
            String,
            '/uwb_status',
            self.status_callback,
            10
        )

        # Publishers
        self.position_publisher = self.create_publisher(
            PointStamped,
            '/uwb_position',
            10
        )

        self.position_debug_publisher = self.create_publisher(
            String,
            '/uwb_position_debug',
            10
        )

        # Store last valid data
        self.last_distances = [0.0, 0.0, 0.0, 0.0]
        self.last_position = [0.0, 0.0]
        self.last_valid_anchors = []
        self.current_status = "Initializing"

        # Parameters for filtering and validation
        self.max_distance_change = 10.0  # meters - max change between readings
        self.min_distance = 0.01  # meters - minimum valid distance
        self.max_distance = 300.0  # meters - maximum valid distance
        self.min_valid_anchors = 3  # minimum number of valid distances needed

        # Smoothing parameters
        self.position_smoothing_alpha = 0.7  # 0 = no smoothing, 1 = no history
        self.distance_smoothing_alpha = 0.8

        self.get_logger().info('UWB Position Calculator started')
        self.get_logger().info(f'Anchor positions: {self.anchor_positions}')
        self.get_logger().info(f'UWB height: {self.uwb_height}m')

    def status_callback(self, msg: String):
        """Callback for UWB status updates"""
        self.current_status = msg.data
        self.get_logger().debug(f"Status: {self.current_status}")

    def distance_callback(self, msg: Float32MultiArray):
        """Callback function for UWB distance readings"""
        distances_m = msg.data  # Already in meters from the log parser

        # Apply smoothing to distances
        smoothed_distances = self.smooth_distances(distances_m)

        # Validate distances
        valid_info = self.validate_distances(smoothed_distances)

        if valid_info['is_valid']:
            # Calculate position using trilateration
            position = self.calculate_position(smoothed_distances, valid_info['valid_indices'])

            if position is not None:
                # Apply position smoothing
                smoothed_position = self.smooth_position(position)

                # Publish the calculated position
                self.publish_position(smoothed_position)

                # Publish debug information
                self.publish_debug_info(smoothed_distances, smoothed_position, valid_info)

                # Log the result
                self.get_logger().info(
                    f"Position: x={smoothed_position[0]:.3f}m, y={smoothed_position[1]:.3f}m "
                    f"| Valid anchors: {len(valid_info['valid_indices'])}/4 "
                    f"| Distances: [{', '.join([f'{d:.3f}m' for d in smoothed_distances])}]"
                )
            else:
                self.get_logger().warn("Failed to calculate position - trilateration error")
                self.publish_debug_info(smoothed_distances, None, valid_info)
        else:
            self.get_logger().warn(f"Invalid distance readings - {valid_info['reason']}")
            self.publish_debug_info(smoothed_distances, None, valid_info)

    def smooth_distances(self, new_distances: List[float]) -> List[float]:
        """Apply exponential smoothing to distance measurements"""
        smoothed = []
        for i, new_dist in enumerate(new_distances):
            if new_dist > 0:  # Only smooth valid distances
                if self.last_distances[i] > 0:
                    smoothed_dist = (self.distance_smoothing_alpha * new_dist +
                                   (1 - self.distance_smoothing_alpha) * self.last_distances[i])
                else:
                    smoothed_dist = new_dist
                smoothed.append(smoothed_dist)
                self.last_distances[i] = smoothed_dist
            else:
                smoothed.append(new_dist)

        return smoothed

    def smooth_position(self, new_position: Tuple[float, float]) -> Tuple[float, float]:
        """Apply exponential smoothing to position estimates"""
        if self.last_position[0] == 0.0 and self.last_position[1] == 0.0:
            # First valid position
            smoothed_position = new_position
        else:
            smoothed_x = (self.position_smoothing_alpha * new_position[0] +
                         (1 - self.position_smoothing_alpha) * self.last_position[0])
            smoothed_y = (self.position_smoothing_alpha * new_position[1] +
                         (1 - self.position_smoothing_alpha) * self.last_position[1])
            smoothed_position = (smoothed_x, smoothed_y)

        self.last_position = smoothed_position
        return smoothed_position

    def validate_distances(self, distances: List[float]) -> dict:
        """Validate distance measurements and return detailed info"""
        valid_indices = []
        invalid_reasons = []

        for i, dist in enumerate(distances):
            if dist <= 0:
                invalid_reasons.append(f"ID{i+1}: No data")
                continue

            # Check if distance is within valid range
            if not (self.min_distance <= dist <= self.max_distance):
                invalid_reasons.append(f"ID{i+1}: Out of range ({dist:.3f}m)")
                continue

            # Check if change from last reading is reasonable
            if (self.last_distances[i] > 0 and
                abs(dist - self.last_distances[i]) > self.max_distance_change):
                invalid_reasons.append(f"ID{i+1}: Large change ({dist:.3f}m)")
                continue

            valid_indices.append(i)

        is_valid = len(valid_indices) >= self.min_valid_anchors
        reason = f"Valid: {len(valid_indices)}/4 anchors"
        if not is_valid:
            reason += f" (need {self.min_valid_anchors})"
        if invalid_reasons:
            reason += f" | Issues: {', '.join(invalid_reasons)}"

        return {
            'is_valid': is_valid,
            'valid_indices': valid_indices,
            'reason': reason
        }

    def calculate_position(self, distances: List[float], valid_indices: List[int]) -> Optional[Tuple[float, float]]:
        """Calculate UWB position using trilateration with improved robustness"""
        try:
            # Extract valid measurements
            valid_distances = [distances[i] for i in valid_indices]
            valid_anchors = [self.anchors[i] for i in valid_indices]

            valid_distances = np.array(valid_distances)
            valid_anchors = np.array(valid_anchors)

            # Calculate 2D distances (project to XY plane)
            adjusted_distances = []
            for i, (anchor, dist) in enumerate(zip(valid_anchors, valid_distances)):
                z_diff = self.uwb_height - anchor[2]
                projected_dist_sq = dist**2 - z_diff**2

                if projected_dist_sq <= 0:
                    self.get_logger().warn(f"Anchor {valid_indices[i]+1}: 3D distance too small for height difference")
                    projected_dist_sq = 0.01  # 10cm minimum 2D distance

                adjusted_distances.append(math.sqrt(projected_dist_sq))

            adjusted_distances = np.array(adjusted_distances)

            # Use weighted least squares trilateration
            if len(valid_anchors) >= 3:
                position = self.trilaterate_weighted(valid_anchors[:, :2], adjusted_distances)

                if position is not None and self.is_position_reasonable(position):
                    return tuple(position)
                else:
                    self.get_logger().warn("Trilateration result unreasonable")
                    return None
            else:
                self.get_logger().warn("Not enough valid anchors for trilateration")
                return None

        except Exception as e:
            self.get_logger().error(f"Error in position calculation: {e}")
            return None

    def trilaterate_weighted(self, anchor_positions: np.ndarray, distances: np.ndarray) -> Optional[np.ndarray]:
        """Weighted least squares trilateration"""
        try:
            # Use first anchor as reference
            ref_pos = anchor_positions[0]
            ref_dist = distances[0]

            # Build overdetermined system
            A = []
            b = []
            weights = []

            for i in range(1, len(anchor_positions)):
                pos = anchor_positions[i]
                dist = distances[i]

                # Linear system coefficients
                A.append([2 * (ref_pos[0] - pos[0]), 2 * (ref_pos[1] - pos[1])])
                b.append(ref_dist**2 - dist**2 + pos[0]**2 + pos[1]**2 - ref_pos[0]**2 - ref_pos[1]**2)

                # Weight inversely proportional to distance (closer anchors are more reliable)
                weights.append(1.0 / (dist + 0.1))

            A = np.array(A)
            b = np.array(b)
            W = np.diag(weights)

            # Weighted least squares: (A^T W A) x = A^T W b
            try:
                AtWA = A.T @ W @ A
                AtWb = A.T @ W @ b
                position = np.linalg.solve(AtWA, AtWb)
                return position
            except np.linalg.LinAlgError:
                # Fallback to regular least squares
                position, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
                return position

        except Exception as e:
            self.get_logger().error(f"Trilateration error: {e}")
            return None

    def is_position_reasonable(self, position: np.ndarray) -> bool:
        """Check if calculated position is reasonable"""
        x, y = position

        # Check for NaN or Inf
        if np.isnan(x) or np.isnan(y) or np.isinf(x) or np.isinf(y):
            return False

        # Check if position is within a reasonable area (adjust based on your setup)
        if abs(x) > 20 or abs(y) > 20:  # 20m from origin
            return False

        return True

    def publish_position(self, position: Tuple[float, float]):
        """Publish the calculated position with timestamp"""
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "uwb_base"
        msg.point.x = float(position[0])
        msg.point.y = float(position[1])
        msg.point.z = float(self.uwb_height)

        self.position_publisher.publish(msg)

    def publish_debug_info(self, distances: List[float], position: Optional[Tuple[float, float]], valid_info: dict):
        """Publish debug information"""
        debug_msg = String()

        timestamp = time.strftime("%H:%M:%S", time.localtime())
        debug_info = f"[{timestamp}] UWB Debug Info:\n"
        debug_info += f"  Status: {self.current_status}\n"
        debug_info += f"  Distances: [ID1: {distances[0]:.3f}m, ID2: {distances[1]:.3f}m, ID3: {distances[2]:.3f}m, ID4: {distances[3]:.3f}m]\n"
        debug_info += f"  Validation: {valid_info['reason']}\n"

        if position:
            debug_info += f"  Position: x={position[0]:.3f}m, y={position[1]:.3f}m, z={self.uwb_height:.3f}m\n"
            debug_info += f"  Valid anchors: {[f'ID{i+1}' for i in valid_info['valid_indices']]}\n"
        else:
            debug_info += "  Position: FAILED\n"

        debug_msg.data = debug_info
        self.position_debug_publisher.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)

    try:
        node = UWBPositionCalculator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down UWB Position Calculator...")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
