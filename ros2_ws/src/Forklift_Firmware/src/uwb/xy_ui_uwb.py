#!/usr/bin/env python3
"""
UWB Position Visualizer using Pygame
Subscribes to UWB trilateration results and displays them on a real-time map
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import pygame
import math
import threading
import time
from collections import deque

# Anchor coordinates [x, y, z] in meters (same as in trilateration script)
# Example for 100m x 100m area - modify as needed
#ANCHORS = [
#    [0, 0, 0.2],           # Anchor 1 - bottom-left
#    [0, 100, 0.2],         # Anchor 2 - top-left
#    [100, 100, 0.2],       # Anchor 3 - top-right
#    [100, 0, 0.2]          # Anchor 4 - bottom-right
#]

# Original small area example:
# ANCHORS = [
#      [1.18, 1.0, 0.2],      # Anchor 1
#      [1.18, 3.085, 0.2],    # Anchor 2
#      [3.2, 3.085, 0.2],     # Anchor 3
#      [3.2, 1.0, 0.2]        # Anchor 4
#  ]

ANCHORS = [
    [14.661, 0.4860, 1.60],      # Anchor 1
    [13.063, 61.099, 1.60],      # Anchor 2
    [84.384, 62.372, 1.60],      # Anchor 3
    [86.745, 4.310, 1.60]       # Anchor 4
]

# Display settings
WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 800
MAP_WIDTH = 700
MAP_HEIGHT = 500
MAP_MARGIN = 50

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
GRAY = (128, 128, 128)
LIGHT_GRAY = (200, 200, 200)
DARK_GRAY = (64, 64, 64)
ORANGE = (255, 165, 0)
PURPLE = (128, 0, 128)

class UWBVisualizerNode(Node):
    def __init__(self):
        super().__init__('uwb_visualizer_node')

        # Current position
        self.current_position = None
        self.pairwise_positions = []
        self.position_history = deque(maxlen=500)  # Store last 500 positions
        self.last_update_time = time.time()

        # Subscribers
        self.position_sub = self.create_subscription(
            PointStamped,
            '/uwb_tag_position',
            self.position_callback,
            10
        )

        self.pairwise_sub = self.create_subscription(
            PointStamped,
            '/uwb_pairwise_position',
            self.pairwise_callback,
            10
        )

        self.get_logger().info('UWB Visualizer Node initialized')

        # Threading lock for thread-safe access
        self.lock = threading.Lock()

    def position_callback(self, msg):
        with self.lock:
            self.current_position = {
                'x': msg.point.x,
                'y': msg.point.y,
                'z': msg.point.z,
                'timestamp': msg.header.stamp
            }

            # Add to history
            self.position_history.append((msg.point.x, msg.point.y))
            self.last_update_time = time.time()

            # Clear pairwise positions (they'll be repopulated for next calculation)
            self.pairwise_positions = []

    def pairwise_callback(self, msg):
        with self.lock:
            self.pairwise_positions.append({
                'x': msg.point.x,
                'y': msg.point.y,
                'z': msg.point.z
            })

class PygameVisualizer:
    def __init__(self, ros_node):
        self.ros_node = ros_node
        pygame.init()
        pygame.font.init()

        # Create display
        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption('UWB Position Visualizer')

        # Fonts
        self.font_large = pygame.font.Font(None, 36)
        self.font_medium = pygame.font.Font(None, 24)
        self.font_small = pygame.font.Font(None, 18)

        # Calculate map bounds
        self.anchors = ANCHORS
        self.min_x = min(anchor[0] for anchor in self.anchors) - 0.5
        self.max_x = max(anchor[0] for anchor in self.anchors) + 0.5
        self.min_y = min(anchor[1] for anchor in self.anchors) - 0.5
        self.max_y = max(anchor[1] for anchor in self.anchors) + 0.5

        # Map positioning
        self.map_x = MAP_MARGIN
        self.map_y = MAP_MARGIN

        # Clock for controlling frame rate
        self.clock = pygame.time.Clock()

    def world_to_screen(self, world_x, world_y):
        """Convert world coordinates to screen coordinates"""
        # Normalize to 0-1 range
        norm_x = (world_x - self.min_x) / (self.max_x - self.min_x)
        norm_y = (world_y - self.min_y) / (self.max_y - self.min_y)

        # Convert to screen coordinates (flip Y axis)
        screen_x = self.map_x + norm_x * MAP_WIDTH
        screen_y = self.map_y + (1 - norm_y) * MAP_HEIGHT

        return int(screen_x), int(screen_y)

    def draw_map(self):
        """Draw the map with anchors and grid"""
        # Draw map border
        pygame.draw.rect(self.screen, BLACK,
                        (self.map_x, self.map_y, MAP_WIDTH, MAP_HEIGHT), 2)

        # Draw grid
        grid_color = LIGHT_GRAY
        for i in range(1, 10):
            # Vertical lines
            x = self.map_x + (i / 10) * MAP_WIDTH
            pygame.draw.line(self.screen, grid_color, (x, self.map_y), (x, self.map_y + MAP_HEIGHT), 1)

            # Horizontal lines
            y = self.map_y + (i / 10) * MAP_HEIGHT
            pygame.draw.line(self.screen, grid_color, (self.map_x, y), (self.map_x + MAP_WIDTH, y), 1)

        # Draw walls (lines between anchors)
        wall_color = DARK_GRAY
        wall_thickness = 3

        # Connect anchors in order: A1->A2->A3->A4->A1 (rectangular boundary)
        anchor_connections = [(0, 1), (1, 2), (2, 3), (3, 0)]

        for start_idx, end_idx in anchor_connections:
            start_pos = self.world_to_screen(self.anchors[start_idx][0], self.anchors[start_idx][1])
            end_pos = self.world_to_screen(self.anchors[end_idx][0], self.anchors[end_idx][1])
            pygame.draw.line(self.screen, wall_color, start_pos, end_pos, wall_thickness)

        # Draw anchors on top of walls
        for i, anchor in enumerate(self.anchors):
            screen_x, screen_y = self.world_to_screen(anchor[0], anchor[1])
            pygame.draw.circle(self.screen, RED, (screen_x, screen_y), 10)
            pygame.draw.circle(self.screen, WHITE, (screen_x, screen_y), 10, 2)

            # Label anchors
            label = self.font_small.render(f'A{i+1}', True, BLACK)
            self.screen.blit(label, (screen_x - 10, screen_y - 25))

        # Draw coordinate labels
        coord_font = self.font_small

        # X-axis labels
        for i in range(0, 11, 2):
            world_x = self.min_x + (i / 10) * (self.max_x - self.min_x)
            screen_x = self.map_x + (i / 10) * MAP_WIDTH
            label = coord_font.render(f'{world_x:.1f}', True, DARK_GRAY)
            self.screen.blit(label, (screen_x - 10, self.map_y + MAP_HEIGHT + 10))

        # Y-axis labels
        for i in range(0, 11, 2):
            world_y = self.min_y + (i / 10) * (self.max_y - self.min_y)
            screen_y = self.map_y + ((10 - i) / 10) * MAP_HEIGHT
            label = coord_font.render(f'{world_y:.1f}', True, DARK_GRAY)
            self.screen.blit(label, (self.map_x - 40, screen_y - 8))

        # Axis labels
        x_label = self.font_medium.render('X (meters)', True, BLACK)
        y_label = self.font_medium.render('Y (meters)', True, BLACK)

        self.screen.blit(x_label, (self.map_x + MAP_WIDTH//2 - 40, self.map_y + MAP_HEIGHT + 40))

        # Rotate Y label
        y_label_rot = pygame.transform.rotate(y_label, 90)
        self.screen.blit(y_label_rot, (self.map_x - 80, self.map_y + MAP_HEIGHT//2 - 40))

    def draw_positions(self):
        """Draw current and historical positions"""
        with self.ros_node.lock:
            # Draw position history (trail)
            if len(self.ros_node.position_history) > 1:
                history_points = []
                for pos in self.ros_node.position_history:
                    screen_x, screen_y = self.world_to_screen(pos[0], pos[1])
                    history_points.append((screen_x, screen_y))

                # Draw trail
                for i in range(1, len(history_points)):
                    alpha = int(255 * (i / len(history_points)) * 0.5)  # Fade effect
                    color = (*BLUE[:3], alpha)
                    pygame.draw.line(self.screen, BLUE, history_points[i-1], history_points[i], 2)

            # Draw pairwise positions
            for i, pos in enumerate(self.ros_node.pairwise_positions):
                screen_x, screen_y = self.world_to_screen(pos['x'], pos['y'])
                pygame.draw.circle(self.screen, ORANGE, (screen_x, screen_y), 6)
                pygame.draw.circle(self.screen, BLACK, (screen_x, screen_y), 6, 1)

                # Label
                label = self.font_small.render(f'P{i+1}', True, BLACK)
                self.screen.blit(label, (screen_x + 10, screen_y - 10))

            # Draw current position
            if self.ros_node.current_position:
                pos = self.ros_node.current_position
                screen_x, screen_y = self.world_to_screen(pos['x'], pos['y'])

                # Draw larger circle for current position
                pygame.draw.circle(self.screen, GREEN, (screen_x, screen_y), 12)
                pygame.draw.circle(self.screen, BLACK, (screen_x, screen_y), 12, 2)

                # Draw crosshair
                pygame.draw.line(self.screen, BLACK, (screen_x-8, screen_y), (screen_x+8, screen_y), 2)
                pygame.draw.line(self.screen, BLACK, (screen_x, screen_y-8), (screen_x, screen_y+8), 2)

    def draw_info_panel(self):
        """Draw information panel with coordinates and status"""
        panel_x = MAP_WIDTH + MAP_MARGIN * 2
        panel_y = MAP_MARGIN
        panel_width = WINDOW_WIDTH - panel_x - MAP_MARGIN
        panel_height = MAP_HEIGHT

        # Panel background
        pygame.draw.rect(self.screen, LIGHT_GRAY, (panel_x, panel_y, panel_width, panel_height))
        pygame.draw.rect(self.screen, BLACK, (panel_x, panel_y, panel_width, panel_height), 2)

        # Title
        title = self.font_large.render('UWB Position Data', True, BLACK)
        self.screen.blit(title, (panel_x + 10, panel_y + 10))

        y_offset = 60

        with self.ros_node.lock:
            # Current position
            if self.ros_node.current_position:
                pos = self.ros_node.current_position

                # Position coordinates
                coord_title = self.font_medium.render('Current Position:', True, BLACK)
                self.screen.blit(coord_title, (panel_x + 10, panel_y + y_offset))
                y_offset += 30

                x_text = self.font_medium.render(f'X: {pos["x"]:.3f} m', True, DARK_GRAY)
                y_text = self.font_medium.render(f'Y: {pos["y"]:.3f} m', True, DARK_GRAY)
                z_text = self.font_medium.render(f'Z: {pos["z"]:.3f} m', True, DARK_GRAY)

                self.screen.blit(x_text, (panel_x + 20, panel_y + y_offset))
                self.screen.blit(y_text, (panel_x + 20, panel_y + y_offset + 25))
                self.screen.blit(z_text, (panel_x + 20, panel_y + y_offset + 50))

                y_offset += 90

                # Status
                time_since_update = time.time() - self.ros_node.last_update_time
                if time_since_update < 1.0:
                    status_color = GREEN
                    status_text = "ACTIVE"
                elif time_since_update < 5.0:
                    status_color = ORANGE
                    status_text = "DELAYED"
                else:
                    status_color = RED
                    status_text = "INACTIVE"

                status_title = self.font_medium.render('Status:', True, BLACK)
                status_value = self.font_medium.render(status_text, True, status_color)
                self.screen.blit(status_title, (panel_x + 10, panel_y + y_offset))
                self.screen.blit(status_value, (panel_x + 80, panel_y + y_offset))

                y_offset += 40

                # Update frequency
                update_text = self.font_small.render(f'Last update: {time_since_update:.1f}s ago', True, DARK_GRAY)
                self.screen.blit(update_text, (panel_x + 10, panel_y + y_offset))

                y_offset += 25

                # Trail length info
                trail_text = self.font_small.render(f'Trail length: {len(self.ros_node.position_history)} points', True, DARK_GRAY)
                self.screen.blit(trail_text, (panel_x + 10, panel_y + y_offset))

                y_offset += 35

                # Pairwise calculations
                if self.ros_node.pairwise_positions:
                    pair_title = self.font_medium.render('Pairwise Calculations:', True, BLACK)
                    self.screen.blit(pair_title, (panel_x + 10, panel_y + y_offset))
                    y_offset += 30

                    for i, pair_pos in enumerate(self.ros_node.pairwise_positions):
                        pair_text = self.font_small.render(
                            f'P{i+1}: ({pair_pos["x"]:.2f}, {pair_pos["y"]:.2f})',
                            True, DARK_GRAY
                        )
                        self.screen.blit(pair_text, (panel_x + 20, panel_y + y_offset))
                        y_offset += 20

            else:
                # No position data
                no_data_text = self.font_medium.render('No position data received', True, RED)
                self.screen.blit(no_data_text, (panel_x + 10, panel_y + y_offset))

        # Legend
        legend_y = panel_y + panel_height - 140
        legend_title = self.font_medium.render('Legend:', True, BLACK)
        self.screen.blit(legend_title, (panel_x + 10, legend_y))

        legend_items = [
            (RED, 'Anchors'),
            (DARK_GRAY, 'Walls'),
            (GREEN, 'Current Position'),
            (ORANGE, 'Pairwise Calculations'),
            (BLUE, 'Position Trail')
        ]

        for i, (color, label) in enumerate(legend_items):
            y_pos = legend_y + 25 + i * 20
            pygame.draw.circle(self.screen, color, (panel_x + 20, y_pos + 8), 6)
            label_text = self.font_small.render(label, True, BLACK)
            self.screen.blit(label_text, (panel_x + 35, y_pos))

    def run(self):
        """Main visualization loop"""
        running = True

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_c:
                        # Clear position history
                        with self.ros_node.lock:
                            self.ros_node.position_history.clear()

            # Clear screen
            self.screen.fill(WHITE)

            # Draw components
            self.draw_map()
            self.draw_positions()
            self.draw_info_panel()

            # Add instructions
            instructions = [
                "Press 'C' to clear position trail",
                "Trail length: 500 points max",
                "Close window to exit"
            ]

            for i, instruction in enumerate(instructions):
                text = self.font_small.render(instruction, True, DARK_GRAY)
                self.screen.blit(text, (10, WINDOW_HEIGHT - 40 + i * 20))

            # Update display
            pygame.display.flip()
            self.clock.tick(30)  # 30 FPS

        pygame.quit()

def main(args=None):
    rclpy.init(args=args)

    # Create ROS node
    ros_node = UWBVisualizerNode()

    # Create visualizer
    visualizer = PygameVisualizer(ros_node)

    # Run ROS node in a separate thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,))
    ros_thread.daemon = True
    ros_thread.start()

    try:
        # Run visualization
        visualizer.run()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
