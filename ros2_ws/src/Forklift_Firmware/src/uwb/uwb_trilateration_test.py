#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import PointStamped
import numpy as np
import math
import json

class UWBPositionCalculator(Node):
    def __init__(self):
        super().__init__('uwb_position_calculator')

        # Get anchor heights and tag height from user
        self.get_setup_parameters()

        # Subscribe to UWB distances
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/uwb_distances',
            self.distance_callback,
            10
        )

        # Publisher for calculated coordinates
        self.coordinates_publisher = self.create_publisher(
            PointStamped,
            '/uwb_coordinates',
            10
        )

        # Publisher for anchor coordinates and heights
        self.anchor_info_publisher = self.create_publisher(
            String,
            '/anchor_coordinates_heights',
            10
        )

        # Publish anchor info once at startup
        self.publish_anchor_info()

        # Anchor positions (d1, d2, d3, d4) in centimeters
        self.anchor_positions = {
            0: np.array([0.0, 0.0]),         # d1 at (0,0)
            1: np.array([0.0,1488.8]),     # d2 at (10000,0)
            2: np.array([435.8, 1500.0]),  # d3 at (10000,6000)
            3: np.array([0.0, 445.0])       # d4 at (0,6000)
        }

        # Define specific anchor pairs for calculation (only adjacent pairs)
        self.anchor_pairs = [
            (0, 1),  # d1, d2
            (1, 2),  # d2, d3
            (2, 3),  # d3, d4
            (3, 0)   # d4, d1
        ]

        self.get_logger().info('UWB Position Calculator initialized for RTLS system')
        self.get_logger().info('System will process UWB distances (in cm) at 5Hz and publish coordinates')
        self.get_logger().info(f'Anchor positions: {self.anchor_positions}')
        self.get_logger().info(f'Anchor heights: {self.anchor_heights}')
        self.get_logger().info(f'Tag height: {self.tag_height}cm')
        self.get_logger().info(f'Anchor pairs for calculation: {self.anchor_pairs}')
        self.get_logger().info('Publishing to /uwb_coordinates and /anchor_coordinates_heights')

    def get_setup_parameters(self):
        """Get anchor heights and tag height from user input in centimeters"""
        print("\n=== UWB Position Calculator Setup ===")

        # Get anchor heights
        print("Please enter the heights of the four anchors in centimeters:")
        self.anchor_heights = {}
        anchor_names = ['d1 (0,0)', 'd2 (10000,0)', 'd3 (10000,6000)', 'd4 (0,6000)']

        for i, name in enumerate(anchor_names):
            while True:
                try:
                    height = float(input(f"Enter height for anchor {name}: "))
                    self.anchor_heights[i] = height
                    break
                except ValueError:
                    print("Please enter a valid number")

        # Get tag height
        while True:
            try:
                self.tag_height = float(input("Enter tag height in centimeters: "))
                break
            except ValueError:
                print("Please enter a valid number")

        print(f"\nAnchor heights set: {self.anchor_heights}")
        print(f"Tag height set: {self.tag_height}cm")
        print("RTLS system ready - will process distances at 5Hz")
        print("Starting UWB position calculation...\n")

    def publish_anchor_info(self):
        """Publish anchor coordinates and heights information"""
        anchor_info = {
            "anchor_coordinates": {
                "d1": {"x": 0.0, "y": 0.0, "height": self.anchor_heights[0]},
                "d2": {"x": 10000.0, "y": 0.0, "height": self.anchor_heights[1]},
                "d3": {"x": 10000.0, "y": 6000.0, "height": self.anchor_heights[2]},
                "d4": {"x": 0.0, "y": 6000.0, "height": self.anchor_heights[3]}
            },
            "tag_height": self.tag_height,
            "field_dimensions": {"length": 10000.0, "width": 6000.0},
            "coordinate_frame": "uwb_coordinate_frame"
        }

        # Convert to JSON string and publish
        anchor_msg = String()
        anchor_msg.data = json.dumps(anchor_info, indent=2)
        self.anchor_info_publisher.publish(anchor_msg)

        self.get_logger().info('Published anchor coordinates and heights to /anchor_coordinates_heights')

    def distance_callback(self, msg):
        """Process incoming UWB distance measurements (in cm) at 5Hz"""
        start_time = self.get_clock().now()

        distances = msg.data

        if len(distances) < 2:
            self.get_logger().warn('RTLS: Need at least 2 distance measurements for positioning')
            return

        # Convert 3D distances to 2D planar distances using Pythagorean theorem
        planar_distances = {}

        for i in range(min(4, len(distances))):  # Process up to 4 distances (d1, d2, d3, d4)
            if distances[i] > 0:  # Valid distance measurement
                height_diff = abs(self.anchor_heights[i] - self.tag_height)

                # Calculate planar distance: sqrt(3D_distance² - height_diff²)
                if distances[i] > height_diff:
                    planar_distance = math.sqrt(distances[i]**2 - height_diff**2)
                    planar_distances[i] = planar_distance
                    self.get_logger().debug(f'RTLS: Anchor d{i+1}: 3D={distances[i]:.2f}cm, 2D={planar_distance:.2f}cm')
                else:
                    self.get_logger().debug(f'RTLS: Distance {distances[i]}cm from anchor d{i+1} is less than height difference {height_diff}cm')

        if len(planar_distances) < 2:
            self.get_logger().debug('RTLS: Need at least 2 valid planar distances')
            return

        # Calculate position using specific anchor pairs
        positions = []
        successful_pairs = []

        for anchor1, anchor2 in self.anchor_pairs:
            if anchor1 in planar_distances and anchor2 in planar_distances:
                pos = self.calculate_position_coordinate_geometry(
                    anchor1, anchor2,
                    planar_distances[anchor1], planar_distances[anchor2]
                )
                if pos is not None:
                    positions.append(pos)
                    successful_pairs.append((anchor1, anchor2))
                    pair_name = f"(d{anchor1+1},d{anchor2+1})"
                    self.get_logger().debug(f'RTLS: Position from {pair_name}: ({pos[0]:.2f}, {pos[1]:.2f})')

        if not positions:
            self.get_logger().debug('RTLS: Could not calculate position from any anchor pair')
            return

        # Average all calculated positions
        avg_position = np.mean(positions, axis=0)

        # Create PointStamped message with timestamp and frame_id
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = "uwb_coordinate_frame"
        point_msg.point.x = float(avg_position[0])
        point_msg.point.y = float(avg_position[1])
        point_msg.point.z = float(self.tag_height)  # Set z to tag height

        self.coordinates_publisher.publish(point_msg)

        # Calculate processing time
        end_time = self.get_clock().now()
        processing_time = (end_time - start_time).nanoseconds / 1e6  # Convert to milliseconds

        # Log results (reduce frequency for cleaner output)
        if len(successful_pairs) > 0:
            timestamp = point_msg.header.stamp.sec + point_msg.header.stamp.nanosec * 1e-9
            self.get_logger().info(f'RTLS [{timestamp:.3f}s]: Position ({point_msg.point.x:.2f}, {point_msg.point.y:.2f}) | '
                                 f'Pairs: {len(positions)} | Processing: {processing_time:.1f}ms')

    def calculate_position_coordinate_geometry(self, anchor1_id, anchor2_id, dist1, dist2):
        """
        Calculate tag position using basic coordinate geometry
        Using the distance formula: (x-x1)² + (y-y1)² = d1²
        """
        # Get anchor positions
        x1, y1 = self.anchor_positions[anchor1_id]
        x2, y2 = self.anchor_positions[anchor2_id]

        self.get_logger().debug(f'RTLS: Calculating position for anchors d{anchor1_id+1}({x1},{y1}) and d{anchor2_id+1}({x2},{y2})')
        self.get_logger().debug(f'RTLS: Distances: d{anchor1_id+1}={dist1:.2f}cm, d{anchor2_id+1}={dist2:.2f}cm')

        # Distance equations:
        # (x - x1)² + (y - y1)² = d1²  ... (1)
        # (x - x2)² + (y - y2)² = d2²  ... (2)

        # Expand both equations:
        # x² - 2*x1*x + x1² + y² - 2*y1*y + y1² = d1²
        # x² - 2*x2*x + x2² + y² - 2*y2*y + y2² = d2²

        # Subtract equation (2) from equation (1):
        # 2*x*(x2-x1) + 2*y*(y2-y1) = d1² - d2² + x2² - x1² + y2² - y1²

        A = 2 * (x2 - x1)
        B = 2 * (y2 - y1)
        C = dist1**2 - dist2**2 + x2**2 - x1**2 + y2**2 - y1**2

        # Linear equation: A*x + B*y = C

        # Check if anchors are at the same position
        if abs(A) < 1e-10 and abs(B) < 1e-10:
            self.get_logger().debug(f'RTLS: Anchors d{anchor1_id+1} and d{anchor2_id+1} are at the same position')
            return None

        # Special case: A ≈ 0 (anchors have same x-coordinate)
        if abs(A) < 1e-10:
            y = C / B
            # Substitute back into first circle equation to get x
            discriminant = dist1**2 - (y - y1)**2
            if discriminant < 0:
                self.get_logger().debug(f'RTLS: No real solution for anchors d{anchor1_id+1} and d{anchor2_id+1}')
                return None

            x1_sol = x1 + math.sqrt(discriminant)
            x2_sol = x1 - math.sqrt(discriminant)

            # Choose solution within field bounds
            if 0 <= x1_sol <= 10000:
                x = x1_sol
            elif 0 <= x2_sol <= 10000:
                x = x2_sol
            else:
                # Take the one closer to field bounds
                x = x1_sol if abs(x1_sol - 5000) < abs(x2_sol - 5000) else x2_sol

        # Special case: B ≈ 0 (anchors have same y-coordinate)
        elif abs(B) < 1e-10:
            x = C / A
            # Substitute back into first circle equation to get y
            discriminant = dist1**2 - (x - x1)**2
            if discriminant < 0:
                self.get_logger().debug(f'RTLS: No real solution for anchors d{anchor1_id+1} and d{anchor2_id+1}')
                return None

            y1_sol = y1 + math.sqrt(discriminant)
            y2_sol = y1 - math.sqrt(discriminant)

            # Choose solution within field bounds
            if 0 <= y1_sol <= 6000:
                y = y1_sol
            elif 0 <= y2_sol <= 6000:
                y = y2_sol
            else:
                # Take the one closer to field bounds
                y = y1_sol if abs(y1_sol - 3000) < abs(y2_sol - 3000) else y2_sol

        # General case: both A and B are non-zero
        else:
            # Express x in terms of y: x = (C - B*y) / A
            # Substitute into first circle equation:
            # ((C - B*y) / A - x1)² + (y - y1)² = d1²

            # Let D = C - A*x1, so x = (D - B*y) / A
            D = C - A * x1

            # Substitute: ((D - B*y) / A)² + (y - y1)² = d1²
            # (D - B*y)² / A² + (y - y1)² = d1²
            # (D - B*y)² + A²*(y - y1)² = A²*d1²

            # Expand: D² - 2*D*B*y + B²*y² + A²*y² - 2*A²*y1*y + A²*y1² = A²*d1²
            # (B² + A²)*y² - (2*D*B + 2*A²*y1)*y + (D² + A²*y1² - A²*d1²) = 0

            a_coeff = B**2 + A**2
            b_coeff = -(2*D*B + 2*A**2*y1)
            c_coeff = D**2 + A**2*y1**2 - A**2*dist1**2

            # Solve quadratic equation
            discriminant = b_coeff**2 - 4*a_coeff*c_coeff

            if discriminant < 0:
                self.get_logger().debug(f'RTLS: No real solution for anchors d{anchor1_id+1} and d{anchor2_id+1}')
                return None

            # Two possible y values
            y1_sol = (-b_coeff + math.sqrt(discriminant)) / (2*a_coeff)
            y2_sol = (-b_coeff - math.sqrt(discriminant)) / (2*a_coeff)

            # Calculate corresponding x values
            x1_sol = (C - B*y1_sol) / A
            x2_sol = (C - B*y2_sol) / A

            # Choose the solution that's within reasonable bounds
            sol1_valid = 0 <= x1_sol <= 10000 and 0 <= y1_sol <= 6000
            sol2_valid = 0 <= x2_sol <= 10000 and 0 <= y2_sol <= 6000

            if sol1_valid and sol2_valid:
                # Both solutions valid, choose the one closer to field center
                center_x, center_y = 5000, 3000
                dist1_to_center = math.sqrt((x1_sol - center_x)**2 + (y1_sol - center_y)**2)
                dist2_to_center = math.sqrt((x2_sol - center_x)**2 + (y2_sol - center_y)**2)

                if dist1_to_center <= dist2_to_center:
                    x, y = x1_sol, y1_sol
                else:
                    x, y = x2_sol, y2_sol
            elif sol1_valid:
                x, y = x1_sol, y1_sol
            elif sol2_valid:
                x, y = x2_sol, y2_sol
            else:
                # Neither solution is perfectly in bounds, choose the better one
                self.get_logger().debug(f'RTLS: Solutions out of bounds for anchors d{anchor1_id+1} and d{anchor2_id+1}')
                # Choose the one that's closer to being in bounds
                score1 = self.calculate_bound_score(x1_sol, y1_sol)
                score2 = self.calculate_bound_score(x2_sol, y2_sol)

                if score1 <= score2:
                    x, y = x1_sol, y1_sol
                else:
                    x, y = x2_sol, y2_sol

        return np.array([x, y])

    def calculate_bound_score(self, x, y):
        """Calculate how far a point is from being within bounds (lower is better)"""
        x_penalty = max(0, -x) + max(0, x - 10000)  # Penalty for being outside [0, 10000]
        y_penalty = max(0, -y) + max(0, y - 6000)   # Penalty for being outside [0, 6000]
        return x_penalty + y_penalty

def main(args=None):
    rclpy.init(args=args)

    try:
        uwb_calculator = UWBPositionCalculator()

        # Log system startup
        uwb_calculator.get_logger().info('=== RTLS SYSTEM STARTED ===')
        uwb_calculator.get_logger().info('Ready to process UWB distances (in cm) at 5Hz')
        uwb_calculator.get_logger().info('Publishing coordinates to /uwb_coordinates')
        uwb_calculator.get_logger().info('Publishing anchor info to /anchor_coordinates_heights')
        uwb_calculator.get_logger().info('=============================')

        rclpy.spin(uwb_calculator)

    except KeyboardInterrupt:
        uwb_calculator.get_logger().info('RTLS system shutdown requested')
    except Exception as e:
        uwb_calculator.get_logger().error(f'RTLS system error: {e}')
    finally:
        if 'uwb_calculator' in locals():
            uwb_calculator.destroy_node()
        rclpy.shutdown()
        print('RTLS system stopped')

if __name__ == '__main__':
    main()
