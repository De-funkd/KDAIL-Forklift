#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import pygame
import sys
import threading
import time
import math
from typing import List, Tuple, Optional
from collections import deque

class UWBVisualizer(Node):
    def __init__(self):
        super().__init__('uwb_visualizer')

        # =============================================================================
        # CONFIGURATION - UPDATE THESE VALUES TO MATCH YOUR SETUP
        # =============================================================================

        # Anchor positions (x, y, z) in meters - should match your calculator script
        self.anchor_positions = [
            [0.0, 0.0, 2.0],    # Anchor 1 (0x0001)
            [5.0, 0.0, 2.0],    # Anchor 2 (0x0002)
            [5.0, 5.0, 2.0],    # Anchor 3 (0x0003)
            [0.0, 5.0, 2.0]     # Anchor 4 (0x0004)
        ]

        # Visualization parameters
        self.world_width = 6.0   # meters - width of the area to visualize
        self.world_height = 6.0  # meters - height of the area to visualize
        self.world_origin_x = -0.5  # meters - x offset for world origin
        self.world_origin_y = -0.5  # meters - y offset for world origin

        # =============================================================================
        # END CONFIGURATION
        # =============================================================================

        # Subscribe to position topic
        self.subscription = self.create_subscription(
            Point,
            '/uwb_position',
            self.position_callback,
            10
        )

        # Current position
        self.current_position = None
        self.position_lock = threading.Lock()

        # Position history for trail
        self.position_history = deque(maxlen=1000)  # Keep last 1000 positions

        # Statistics
        self.total_positions = 0
        self.start_time = time.time()

        self.get_logger().info('UWB Visualizer started - waiting for position data...')

    def position_callback(self, msg: Point):
        """Callback for position updates"""
        with self.position_lock:
            self.current_position = (msg.x, msg.y)
            self.position_history.append((msg.x, msg.y))
            self.total_positions += 1

            # Log occasionally
            if self.total_positions % 10 == 0:
                self.get_logger().info(f"Received {self.total_positions} positions")

class PygameVisualizer:
    def __init__(self, uwb_node: UWBVisualizer):
        self.uwb_node = uwb_node

        # Pygame setup
        pygame.init()

        # Screen settings
        self.screen_width = 1000
        self.screen_height = 800
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height))
        pygame.display.set_caption("UWB Position Visualizer")

        # Colors
        self.colors = {
            'background': (20, 20, 30),
            'grid': (60, 60, 80),
            'anchor': (255, 100, 100),
            'anchor_text': (255, 255, 255),
            'current_position': (100, 255, 100),
            'trail': (50, 150, 255),
            'trail_fade': (30, 90, 150),
            'text': (255, 255, 255),
            'info_bg': (40, 40, 50),
            'border': (100, 100, 120)
        }

        # Fonts
        self.font_large = pygame.font.Font(None, 24)
        self.font_medium = pygame.font.Font(None, 18)
        self.font_small = pygame.font.Font(None, 14)

        # Visualization area (leave space for info panel)
        self.viz_margin = 50
        self.info_panel_width = 250
        self.viz_width = self.screen_width - self.info_panel_width - 2 * self.viz_margin
        self.viz_height = self.screen_height - 2 * self.viz_margin
        self.viz_x = self.viz_margin
        self.viz_y = self.viz_margin

        # World to screen conversion
        self.world_to_screen_scale_x = self.viz_width / self.uwb_node.world_width
        self.world_to_screen_scale_y = self.viz_height / self.uwb_node.world_height

        # Use the smaller scale to maintain aspect ratio
        self.world_to_screen_scale = min(self.world_to_screen_scale_x, self.world_to_screen_scale_y)

        # Clock for FPS
        self.clock = pygame.time.Clock()
        self.running = True

        # Animation
        self.trail_fade_time = 5.0  # seconds

    def world_to_screen(self, world_x: float, world_y: float) -> Tuple[int, int]:
        """Convert world coordinates to screen coordinates"""
        # Adjust for world origin
        adjusted_x = world_x - self.uwb_node.world_origin_x
        adjusted_y = world_y - self.uwb_node.world_origin_y

        # Convert to screen coordinates (flip Y axis)
        screen_x = int(self.viz_x + adjusted_x * self.world_to_screen_scale)
        screen_y = int(self.viz_y + self.viz_height - adjusted_y * self.world_to_screen_scale)

        return screen_x, screen_y

    def draw_grid(self):
        """Draw background grid"""
        # Draw grid lines every 0.5 meters
        grid_spacing = 0.5

        # Vertical lines
        x = 0
        while x <= self.uwb_node.world_width:
            world_x = self.uwb_node.world_origin_x + x
            screen_x, _ = self.world_to_screen(world_x, 0)
            if self.viz_x <= screen_x <= self.viz_x + self.viz_width:
                pygame.draw.line(self.screen, self.colors['grid'],
                               (screen_x, self.viz_y),
                               (screen_x, self.viz_y + self.viz_height), 1)
            x += grid_spacing

        # Horizontal lines
        y = 0
        while y <= self.uwb_node.world_height:
            world_y = self.uwb_node.world_origin_y + y
            _, screen_y = self.world_to_screen(0, world_y)
            if self.viz_y <= screen_y <= self.viz_y + self.viz_height:
                pygame.draw.line(self.screen, self.colors['grid'],
                               (self.viz_x, screen_y),
                               (self.viz_x + self.viz_width, screen_y), 1)
            y += grid_spacing

    def draw_anchors(self):
        """Draw anchor positions"""
        for i, (x, y, z) in enumerate(self.uwb_node.anchor_positions):
            screen_x, screen_y = self.world_to_screen(x, y)

            # Draw anchor as a larger circle
            pygame.draw.circle(self.screen, self.colors['anchor'],
                             (screen_x, screen_y), 12)
            pygame.draw.circle(self.screen, self.colors['anchor_text'],
                             (screen_x, screen_y), 12, 2)

            # Draw anchor label
            label = f"A{i+1}"
            text_surface = self.font_medium.render(label, True, self.colors['anchor_text'])
            text_rect = text_surface.get_rect(center=(screen_x, screen_y))
            self.screen.blit(text_surface, text_rect)

            # Draw anchor coordinates
            coord_text = f"({x:.1f}, {y:.1f})"
            coord_surface = self.font_small.render(coord_text, True, self.colors['anchor_text'])
            coord_rect = coord_surface.get_rect(center=(screen_x, screen_y + 20))
            self.screen.blit(coord_surface, coord_rect)

    def draw_trail(self):
        """Draw position trail with fading effect"""
        if len(self.uwb_node.position_history) < 2:
            return

        current_time = time.time()
        positions = list(self.uwb_node.position_history)

        # Draw trail segments
        for i in range(len(positions) - 1):
            # Calculate fade based on position in history
            fade_factor = (i + 1) / len(positions)

            # Interpolate color
            r = int(self.colors['trail_fade'][0] +
                   (self.colors['trail'][0] - self.colors['trail_fade'][0]) * fade_factor)
            g = int(self.colors['trail_fade'][1] +
                   (self.colors['trail'][1] - self.colors['trail_fade'][1]) * fade_factor)
            b = int(self.colors['trail_fade'][2] +
                   (self.colors['trail'][2] - self.colors['trail_fade'][2]) * fade_factor)

            color = (r, g, b)

            # Draw line segment
            start_pos = self.world_to_screen(positions[i][0], positions[i][1])
            end_pos = self.world_to_screen(positions[i + 1][0], positions[i + 1][1])

            # Draw thicker line for recent positions
            thickness = max(1, int(fade_factor * 3))
            pygame.draw.line(self.screen, color, start_pos, end_pos, thickness)

        # Draw position dots
        for i, (x, y) in enumerate(positions[::5]):  # Every 5th position
            fade_factor = (i * 5 + 1) / len(positions)
            alpha = int(100 + 155 * fade_factor)

            screen_x, screen_y = self.world_to_screen(x, y)

            # Create surface for alpha blending
            dot_surface = pygame.Surface((6, 6), pygame.SRCALPHA)
            dot_color = (*self.colors['trail'], alpha)
            pygame.draw.circle(dot_surface, dot_color, (3, 3), 3)
            self.screen.blit(dot_surface, (screen_x - 3, screen_y - 3))

    def draw_current_position(self):
        """Draw current UWB position"""
        with self.uwb_node.position_lock:
            if self.uwb_node.current_position is None:
                return

            x, y = self.uwb_node.current_position

        screen_x, screen_y = self.world_to_screen(x, y)

        # Draw pulsing circle for current position
        pulse = abs(math.sin(time.time() * 4)) * 0.3 + 0.7
        radius = int(8 * pulse)

        pygame.draw.circle(self.screen, self.colors['current_position'],
                         (screen_x, screen_y), radius + 2)
        pygame.draw.circle(self.screen, self.colors['background'],
                         (screen_x, screen_y), radius, 2)

        # Draw crosshair
        pygame.draw.line(self.screen, self.colors['current_position'],
                        (screen_x - 12, screen_y), (screen_x + 12, screen_y), 2)
        pygame.draw.line(self.screen, self.colors['current_position'],
                        (screen_x, screen_y - 12), (screen_x, screen_y + 12), 2)

        # Draw coordinates
        coord_text = f"({x:.2f}, {y:.2f})"
        coord_surface = self.font_medium.render(coord_text, True, self.colors['current_position'])
        coord_rect = coord_surface.get_rect(center=(screen_x, screen_y + 25))
        self.screen.blit(coord_surface, coord_rect)

    def draw_info_panel(self):
        """Draw information panel"""
        panel_x = self.viz_x + self.viz_width + 10
        panel_y = self.viz_y
        panel_width = self.info_panel_width - 20
        panel_height = self.viz_height

        # Draw panel background
        pygame.draw.rect(self.screen, self.colors['info_bg'],
                        (panel_x, panel_y, panel_width, panel_height))
        pygame.draw.rect(self.screen, self.colors['border'],
                        (panel_x, panel_y, panel_width, panel_height), 2)

        # Panel content
        y_offset = panel_y + 20
        line_height = 25

        # Title
        title_surface = self.font_large.render("UWB Tracker", True, self.colors['text'])
        self.screen.blit(title_surface, (panel_x + 10, y_offset))
        y_offset += line_height + 10

        # Current position
        with self.uwb_node.position_lock:
            if self.uwb_node.current_position:
                x, y = self.uwb_node.current_position
                pos_text = f"Position: ({x:.3f}, {y:.3f})"
            else:
                pos_text = "Position: No data"

        pos_surface = self.font_medium.render(pos_text, True, self.colors['text'])
        self.screen.blit(pos_surface, (panel_x + 10, y_offset))
        y_offset += line_height

        # Statistics
        stats_text = [
            f"Total points: {self.uwb_node.total_positions}",
            f"Trail length: {len(self.uwb_node.position_history)}",
            f"Uptime: {time.time() - self.uwb_node.start_time:.1f}s"
        ]

        for stat in stats_text:
            stat_surface = self.font_small.render(stat, True, self.colors['text'])
            self.screen.blit(stat_surface, (panel_x + 10, y_offset))
            y_offset += line_height

        y_offset += 20

        # Anchor information
        anchor_title = self.font_medium.render("Anchors:", True, self.colors['text'])
        self.screen.blit(anchor_title, (panel_x + 10, y_offset))
        y_offset += line_height

        for i, (x, y, z) in enumerate(self.uwb_node.anchor_positions):
            anchor_text = f"A{i+1}: ({x:.1f}, {y:.1f}, {z:.1f})"
            anchor_surface = self.font_small.render(anchor_text, True, self.colors['anchor'])
            self.screen.blit(anchor_surface, (panel_x + 15, y_offset))
            y_offset += line_height - 5

        # Controls
        y_offset += 20
        controls_title = self.font_medium.render("Controls:", True, self.colors['text'])
        self.screen.blit(controls_title, (panel_x + 10, y_offset))
        y_offset += line_height

        controls_text = [
            "R - Clear trail",
            "ESC - Exit",
            "Space - Pause/Resume"
        ]

        for control in controls_text:
            control_surface = self.font_small.render(control, True, self.colors['text'])
            self.screen.blit(control_surface, (panel_x + 15, y_offset))
            y_offset += line_height - 5

    def draw_border(self):
        """Draw border around visualization area"""
        pygame.draw.rect(self.screen, self.colors['border'],
                        (self.viz_x - 2, self.viz_y - 2,
                         self.viz_width + 4, self.viz_height + 4), 2)

    def handle_events(self):
        """Handle pygame events"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_r:
                    # Clear trail
                    self.uwb_node.position_history.clear()
                    self.uwb_node.get_logger().info("Trail cleared")
                elif event.key == pygame.K_SPACE:
                    # Toggle pause (could be implemented)
                    pass

    def run(self):
        """Main visualization loop"""
        while self.running:
            self.handle_events()

            # Clear screen
            self.screen.fill(self.colors['background'])

            # Draw visualization elements
            self.draw_grid()
            self.draw_anchors()
            self.draw_trail()
            self.draw_current_position()
            self.draw_info_panel()
            self.draw_border()

            # Update display
            pygame.display.flip()
            self.clock.tick(30)  # 30 FPS

        pygame.quit()

def main(args=None):
    rclpy.init(args=args)

    try:
        # Create ROS node
        uwb_node = UWBVisualizer()

        # Start ROS spinning in a separate thread
        ros_thread = threading.Thread(target=lambda: rclpy.spin(uwb_node), daemon=True)
        ros_thread.start()

        # Create and run pygame visualizer
        visualizer = PygameVisualizer(uwb_node)

        print("\n" + "="*60)
        print("UWB Position Visualizer")
        print("="*60)
        print("Controls:")
        print("  R     - Clear trail")
        print("  ESC   - Exit")
        print("  Space - Pause/Resume")
        print("="*60)
        print("Subscribing to: /uwb_position")
        print("="*60 + "\n")

        visualizer.run()

    except KeyboardInterrupt:
        print("\nShutting down visualizer...")
    finally:
        if 'uwb_node' in locals():
            uwb_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
