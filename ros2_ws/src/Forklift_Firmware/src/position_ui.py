#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
import pygame
import sys
import math
import json
import os

class EnhancedUWBVisualizer(Node):
    def __init__(self):
        super().__init__('enhanced_uwb_visualizer')

        # Initialize pygame
        pygame.init()

        # Constants
        self.WINDOW_SIZE = (1200, 900)
        self.VISUALIZATION_AREA = (800, 800)
        self.CONTROL_PANEL_WIDTH = 400
        self.MARGIN = 50
        self.SCALE = (self.VISUALIZATION_AREA[0] - 2 * self.MARGIN) / 1000

        # Colors
        self.WHITE = (255, 255, 255)
        self.BLACK = (0, 0, 0)
        self.RED = (255, 0, 0)
        self.GREEN = (0, 255, 0)
        self.BLUE = (0, 0, 255)
        self.GRAY = (128, 128, 128)
        self.LIGHT_GRAY = (200, 200, 200)
        self.DARK_GRAY = (64, 64, 64)
        self.YELLOW = (255, 255, 0)
        self.ORANGE = (255, 165, 0)

        # Setup display
        self.screen = pygame.display.set_mode(self.WINDOW_SIZE)
        pygame.display.set_caption('Enhanced UWB Position Visualizer (3D)')

        # Fonts
        self.font = pygame.font.Font(None, 24)
        self.small_font = pygame.font.Font(None, 18)
        self.large_font = pygame.font.Font(None, 32)

        # Anchor positions (in cm, now with x, y, z)
        self.config_file = 'anchor_config.json'
        self.load_anchor_positions()

        # Current tag position and distances
        self.current_position = (0, 0, 0)
        self.current_distances = [0.0, 0.0, 0.0, 0.0]

        # UI state
        self.selected_anchor = None
        self.editing_anchor = None
        self.edit_fields = {'x': '', 'y': '', 'z': ''}
        self.active_field = None
        self.show_distances = True
        self.show_circles = True

        # UI elements
        self.input_boxes = {}
        self.buttons = {}
        self.create_ui_elements()

        # Subscribe to position and distance topics
        self.position_subscription = self.create_subscription(
            Point,
            '/uwb_xy_coords',
            self.position_callback,
            10
        )

        self.distance_subscription = self.create_subscription(
            Float32MultiArray,
            '/raw_uwb_data',
            self.distance_callback,
            10
        )

        # Publisher for anchor positions
        self.anchor_publisher = self.create_publisher(
            Float32MultiArray,
            '/anchor_positions',
            10
        )

        # Publish initial anchor positions
        self.publish_anchor_positions()

        # Timer for pygame events
        self.create_timer(0.05, self.update)  # 20 Hz

        self.get_logger().info('Enhanced UWB Visualizer (3D) started')

    def create_ui_elements(self):
        """Create UI input boxes and buttons"""
        panel_x = self.VISUALIZATION_AREA[0] + 10

        # Create input boxes for each anchor (x, y, z)
        for i in range(1, 5):
            y_pos = 50 + (i-1) * 150
            self.input_boxes[f'anchor_{i}_x'] = pygame.Rect(panel_x + 80, y_pos, 80, 30)
            self.input_boxes[f'anchor_{i}_y'] = pygame.Rect(panel_x + 80, y_pos + 40, 80, 30)
            self.input_boxes[f'anchor_{i}_z'] = pygame.Rect(panel_x + 80, y_pos + 80, 80, 30)

        # Create buttons with adjusted positions to avoid overlap
        self.buttons['save'] = pygame.Rect(panel_x, 600, 100, 40)
        self.buttons['reset'] = pygame.Rect(panel_x + 110, 600, 100, 40)
        self.buttons['toggle_distances'] = pygame.Rect(panel_x, 650, 150, 40)
        self.buttons['toggle_circles'] = pygame.Rect(panel_x, 700, 150, 40)

    def load_anchor_positions(self):
        """Load anchor positions from config file"""
        self.anchor_positions = {
            1: (0.0, 0.0, 0.0),
            2: (1000.0, 0.0, 0.0),
            3: (1000.0, 1000.0, 0.0),
            4: (0.0, 1000.0, 0.0)
        }

        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r') as f:
                    data = json.load(f)
                # Convert string keys back to integers and handle 2D or 3D
                for k, v in data.items():
                    k = int(k)
                    if len(v) == 2:  # Handle legacy 2D coordinates
                        self.anchor_positions[k] = (float(v[0]), float(v[1]), 0.0)
                    elif len(v) == 3:  # Handle 3D coordinates
                        self.anchor_positions[k] = (float(v[0]), float(v[1]), float(v[2]))
                self.get_logger().info(f'Loaded anchor positions: {self.anchor_positions}')
            except Exception as e:
                self.get_logger().warn(f'Failed to load anchor positions: {e}')
                # Save default positions to fix config file
                self.save_anchor_positions()

    def save_anchor_positions(self):
        """Save anchor positions to config file and publish"""
        try:
            with open(self.config_file, 'w') as f:
                json.dump(self.anchor_positions, f, indent=2)
            self.get_logger().info('Anchor positions saved')
            self.publish_anchor_positions()
        except Exception as e:
            self.get_logger().error(f'Failed to save anchor positions: {e}')

    def publish_anchor_positions(self):
        """Publish anchor positions to ROS topic"""
        msg = Float32MultiArray()
        # Flatten anchor positions: [x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4]
        data = []
        for i in range(1, 5):
            x, y, z = self.anchor_positions[i]
            data.extend([x, y, z])
        msg.data = data
        self.anchor_publisher.publish(msg)
        self.get_logger().info('Published anchor positions')

    def position_callback(self, msg):
        """Callback for position updates"""
        self.current_position = (msg.x, msg.y, msg.z)

    def distance_callback(self, msg):
        """Callback for distance updates"""
        if len(msg.data) >= 4:
            self.current_distances = list(msg.data[:4])

    def cm_to_screen(self, x, y, z):
        """Convert from cm coordinates to screen coordinates (project 3D to 2D)"""
        screen_x = self.MARGIN + x * self.SCALE
        screen_y = self.VISUALIZATION_AREA[1] - (self.MARGIN + y * self.SCALE)
        return (screen_x, screen_y)

    def screen_to_cm(self, screen_x, screen_y):
        """Convert from screen coordinates to cm (Z remains unchanged)"""
        x = (screen_x - self.MARGIN) / self.SCALE
        y = (self.VISUALIZATION_AREA[1] - self.MARGIN - screen_y) / self.SCALE
        return (x, y)

    def draw_grid(self):
        """Draw coordinate grid"""
        for i in range(0, 1001, 100):
            start_x, start_y = self.cm_to_screen(i, 0, 0)
            end_x, end_y = self.cm_to_screen(i, 1000, 0)
            color = self.GRAY if i % 500 == 0 else self.LIGHT_GRAY
            pygame.draw.line(self.screen, color, (start_x, start_y), (end_x, end_y), 2 if i % 500 == 0 else 1)

            start_x, start_y = self.cm_to_screen(0, i, 0)
            end_x, end_y = self.cm_to_screen(1000, i, 0)
            pygame.draw.line(self.screen, color, (start_x, start_y), (end_x, end_y), 2 if i % 500 == 0 else 1)

        # Draw axis labels
        for i in range(0, 1001, 200):
            x_pos, y_pos = self.cm_to_screen(i, -30, 0)
            text = self.small_font.render(f'{i}', True, self.BLACK)
            self.screen.blit(text, (x_pos - 10, y_pos))

            x_pos, y_pos = self.cm_to_screen(-30, i, 0)
            text = self.small_font.render(f'{i}', True, self.BLACK)
            self.screen.blit(text, (x_pos, y_pos - 10))

    def draw_distance_circles(self):
        """Draw distance circles from anchors"""
        if not self.show_circles:
            return

        for anchor_id, pos in self.anchor_positions.items():
            if anchor_id <= len(self.current_distances):
                distance = self.current_distances[anchor_id - 1]
                if distance > 0:
                    screen_x, screen_y = self.cm_to_screen(pos[0], pos[1], pos[2])
                    radius = distance * self.SCALE
                    if radius > 5:
                        pygame.draw.circle(self.screen, self.ORANGE,
                                         (int(screen_x), int(screen_y)), int(radius), 2)

    def draw_anchors(self):
        """Draw anchor points"""
        for anchor_id, pos in self.anchor_positions.items():
            screen_x, screen_y = self.cm_to_screen(pos[0], pos[1], pos[2])

            # Choose color based on state
            if self.selected_anchor == anchor_id:
                color = self.YELLOW
                radius = 12
            elif self.editing_anchor == anchor_id:
                color = self.RED
                radius = 10
            else:
                color = self.BLUE
                radius = 8

            pygame.draw.circle(self.screen, color, (int(screen_x), int(screen_y)), radius)
            pygame.draw.circle(self.screen, self.BLACK, (int(screen_x), int(screen_y)), radius, 2)

            # Draw anchor ID
            text = self.font.render(f'A{anchor_id}', True, self.WHITE)
            text_rect = text.get_rect(center=(screen_x, screen_y))
            self.screen.blit(text, text_rect)

            # Draw distance if available
            if self.show_distances and anchor_id <= len(self.current_distances):
                distance = self.current_distances[anchor_id - 1]
                dist_text = f'{distance:.1f}cm'
                text_surface = self.small_font.render(dist_text, True, self.BLACK)
                self.screen.blit(text_surface, (screen_x + 15, screen_y - 20))

    def draw_tag_position(self):
        """Draw current tag position"""
        screen_x, screen_y = self.cm_to_screen(self.current_position[0], self.current_position[1], self.current_position[2])

        # Draw tag with cross-hair
        pygame.draw.circle(self.screen, self.GREEN, (int(screen_x), int(screen_y)), 8)
        pygame.draw.circle(self.screen, self.BLACK, (int(screen_x), int(screen_y)), 8, 2)

        # Draw cross-hair
        pygame.draw.line(self.screen, self.BLACK,
                        (screen_x - 12, screen_y), (screen_x + 12, screen_y), 2)
        pygame.draw.line(self.screen, self.BLACK,
                        (screen_x, screen_y - 12), (screen_x, screen_y + 12), 2)

        # Draw position text with background
        text = f'Tag: ({self.current_position[0]:.1f}, {self.current_position[1]:.1f}, {self.current_position[2]:.1f})'
        text_surface = self.font.render(text, True, self.BLACK)
        text_rect = text_surface.get_rect()
        text_rect.topleft = (screen_x + 15, screen_y - 25)

        # Background rectangle
        bg_rect = text_rect.inflate(4, 2)
        pygame.draw.rect(self.screen, self.WHITE, bg_rect)
        pygame.draw.rect(self.screen, self.BLACK, bg_rect, 1)
        self.screen.blit(text_surface, text_rect)

    def draw_control_panel(self):
        """Draw the control panel"""
        panel_x = self.VISUALIZATION_AREA[0]

        # Draw panel background
        panel_rect = pygame.Rect(panel_x, 0, self.CONTROL_PANEL_WIDTH, self.WINDOW_SIZE[1])
        pygame.draw.rect(self.screen, self.LIGHT_GRAY, panel_rect)
        pygame.draw.line(self.screen, self.BLACK, (panel_x, 0), (panel_x, self.WINDOW_SIZE[1]), 2)

        # Title
        title = self.large_font.render('Anchor Control (3D)', True, self.BLACK)
        self.screen.blit(title, (panel_x + 10, 10))

        # Draw anchor controls
        for i in range(1, 5):
            y_pos = 50 + (i-1) * 150
            anchor_pos = self.anchor_positions[i]

            # Anchor label
            label = self.font.render(f'Anchor {i}:', True, self.BLACK)
            self.screen.blit(label, (panel_x + 10, y_pos - 5))

            # Current position display
            pos_text = f'({anchor_pos[0]:.1f}, {anchor_pos[1]:.1f}, {anchor_pos[2]:.1f})'
            pos_surface = self.small_font.render(pos_text, True, self.DARK_GRAY)
            self.screen.blit(pos_surface, (panel_x + 100, y_pos - 5))

            # X input
            x_label = self.font.render('X:', True, self.BLACK)
            self.screen.blit(x_label, (panel_x + 10, y_pos + 25))

            x_box = self.input_boxes[f'anchor_{i}_x']
            color = self.WHITE if self.active_field != f'anchor_{i}_x' else self.YELLOW
            pygame.draw.rect(self.screen, color, x_box)
            pygame.draw.rect(self.screen, self.BLACK, x_box, 2)

            x_text = str(anchor_pos[0]) if self.editing_anchor != i else self.edit_fields.get('x', str(anchor_pos[0]))
            x_surface = self.font.render(x_text, True, self.BLACK)
            self.screen.blit(x_surface, (x_box.x + 5, x_box.y + 5))

            # Y input
            y_label = self.font.render('Y:', True, self.BLACK)
            self.screen.blit(y_label, (panel_x + 10, y_pos + 65))

            y_box = self.input_boxes[f'anchor_{i}_y']
            color = self.WHITE if self.active_field != f'anchor_{i}_y' else self.YELLOW
            pygame.draw.rect(self.screen, color, y_box)
            pygame.draw.rect(self.screen, self.BLACK, y_box, 2)

            y_text = str(anchor_pos[1]) if self.editing_anchor != i else self.edit_fields.get('y', str(anchor_pos[1]))
            y_surface = self.font.render(y_text, True, self.BLACK)
            self.screen.blit(y_surface, (y_box.x + 5, y_box.y + 5))

            # Z input
            z_label = self.font.render('Z:', True, self.BLACK)
            self.screen.blit(z_label, (panel_x + 10, y_pos + 105))

            z_box = self.input_boxes[f'anchor_{i}_z']
            color = self.WHITE if self.active_field != f'anchor_{i}_z' else self.YELLOW
            pygame.draw.rect(self.screen, color, z_box)
            pygame.draw.rect(self.screen, self.BLACK, z_box, 2)

            z_text = str(anchor_pos[2]) if self.editing_anchor != i else self.edit_fields.get('z', str(anchor_pos[2]))
            z_surface = self.font.render(z_text, True, self.BLACK)
            self.screen.blit(z_surface, (z_box.x + 5, z_box.y + 5))

            # Distance display
            if i <= len(self.current_distances):
                dist_text = f'Dist: {self.current_distances[i-1]:.1f}cm'
                dist_surface = self.small_font.render(dist_text, True, self.BLUE)
                self.screen.blit(dist_surface, (panel_x + 280, y_pos + 65))

        # Draw buttons
        self.draw_button('save', 'Save', self.GREEN)
        self.draw_button('reset', 'Reset', self.RED)
        self.draw_button('toggle_distances', 'Show Distances' if self.show_distances else 'Hide Distances', self.BLUE)
        self.draw_button('toggle_circles', 'Show Circles' if self.show_circles else 'Hide Circles', self.ORANGE)

        # Current position display
        pos_y = 750
        current_pos_label = self.font.render('Current Tag Position:', True, self.BLACK)
        self.screen.blit(current_pos_label, (panel_x + 10, pos_y))

        current_pos_text = f'X: {self.current_position[0]:.2f} cm'
        current_pos_surface = self.font.render(current_pos_text, True, self.GREEN)
        self.screen.blit(current_pos_surface, (panel_x + 10, pos_y + 25))

        current_pos_text = f'Y: {self.current_position[1]:.2f} cm'
        current_pos_surface = self.font.render(current_pos_text, True, self.GREEN)
        self.screen.blit(current_pos_surface, (panel_x + 10, pos_y + 50))

        current_pos_text = f'Z: {self.current_position[2]:.2f} cm'
        current_pos_surface = self.font.render(current_pos_text, True, self.GREEN)
        self.screen.blit(current_pos_surface, (panel_x + 10, pos_y + 75))

    def draw_button(self, button_name, text, color):
        """Draw a button"""
        button_rect = self.buttons[button_name]
        pygame.draw.rect(self.screen, color, button_rect)
        pygame.draw.rect(self.screen, self.BLACK, button_rect, 2)

        text_surface = self.font.render(text, True, self.WHITE)
        text_rect = text_surface.get_rect(center=button_rect.center)
        self.screen.blit(text_surface, text_rect)

    def handle_mouse_click(self, pos):
        """Handle mouse clicks"""
        if pos[0] < self.VISUALIZATION_AREA[0]:
            for anchor_id, anchor_pos in self.anchor_positions.items():
                screen_x, screen_y = self.cm_to_screen(anchor_pos[0], anchor_pos[1], anchor_pos[2])
                distance = math.sqrt((pos[0] - screen_x)**2 + (pos[1] - screen_y)**2)
                if distance < 15:
                    self.selected_anchor = anchor_id
                    return
            self.selected_anchor = None
        else:
            for box_name, box_rect in self.input_boxes.items():
                if box_rect.collidepoint(pos):
                    anchor_id = int(box_name.split('_')[1])
                    field = box_name.split('_')[2]
                    self.editing_anchor = anchor_id
                    self.active_field = box_name
                    current_pos = self.anchor_positions[anchor_id]
                    self.edit_fields = {'x': str(current_pos[0]), 'y': str(current_pos[1]), 'z': str(current_pos[2])}
                    return

            for button_name, button_rect in self.buttons.items():
                if button_rect.collidepoint(pos):
                    self.handle_button_click(button_name)
                    return

            self.active_field = None
            self.editing_anchor = None

    def handle_button_click(self, button_name):
        """Handle button clicks"""
        if button_name == 'save':
            self.save_anchor_positions()
        elif button_name == 'reset':
            self.anchor_positions = {
                1: (0.0, 0.0, 0.0),
                2: (1000.0, 0.0, 0.0),
                3: (1000.0, 1000.0, 0.0),
                4: (0.0, 1000.0, 0.0)
            }
            self.save_anchor_positions()
        elif button_name == 'toggle_distances':
            self.show_distances = not self.show_distances
        elif button_name == 'toggle_circles':
            self.show_circles = not self.show_circles

    def handle_text_input(self, text):
        """Handle text input for active field"""
        if self.active_field and self.editing_anchor:
            field = self.active_field.split('_')[2]
            if text.isdigit() or text == '.' or text == '-':
                self.edit_fields[field] += text

    def handle_backspace(self):
        """Handle backspace for active field"""
        if self.active_field and self.editing_anchor:
            field = self.active_field.split('_')[2]
            if self.edit_fields[field]:
                self.edit_fields[field] = self.edit_fields[field][:-1]

    def handle_enter(self):
        """Handle enter key to confirm input"""
        if self.editing_anchor:
            try:
                x = float(self.edit_fields['x'])
                y = float(self.edit_fields['y'])
                z = float(self.edit_fields['z'])
                self.anchor_positions[self.editing_anchor] = (x, y, z)
                self.get_logger().info(f'Updated Anchor {self.editing_anchor} to ({x}, {y}, {z})')
                self.publish_anchor_positions()
            except ValueError:
                self.get_logger().warn('Invalid input values')

            self.editing_anchor = None
            self.active_field = None

    def update(self):
        """Update display and handle events"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

            elif event.type == pygame.MOUSEBUTTONDOWN:
                self.handle_mouse_click(event.pos)

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_RETURN:
                    self.handle_enter()
                elif event.key == pygame.K_BACKSPACE:
                    self.handle_backspace()
                elif event.key == pygame.K_ESCAPE:
                    self.editing_anchor = None
                    self.active_field = None

            elif event.type == pygame.TEXTINPUT:
                self.handle_text_input(event.text)

        # Clear screen
        self.screen.fill(self.WHITE)

        # Draw visualization area background
        vis_rect = pygame.Rect(0, 0, self.VISUALIZATION_AREA[0], self.VISUALIZATION_AREA[1])
        pygame.draw.rect(self.screen, self.WHITE, vis_rect)

        # Draw elements
        self.draw_grid()
        self.draw_distance_circles()
        self.draw_anchors()
        self.draw_tag_position()
        self.draw_control_panel()

        pygame.display.flip()

def main(args=None):
    rclpy.init(args=args)
    try:
        visualizer = EnhancedUWBVisualizer()
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()
        if 'visualizer' in locals():
            visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
