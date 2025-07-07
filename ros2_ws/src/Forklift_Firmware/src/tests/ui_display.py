#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import pygame
import sys
import threading
import time
from collections import deque
import math

class UWBDistanceUI(Node):
    def __init__(self):
        super().__init__('uwb_distance_ui')

        # ROS subscriber
        self.subscription = self.create_subscription(
            Float32,
            '/uwb_1_responder_data',
            self.distance_callback,
            10
        )

        # Distance data storage
        self.current_distance = 0.0
        self.distance_history = deque(maxlen=100)  # Store last 100 readings
        self.last_update_time = time.time()
        self.data_lock = threading.Lock()

        # PyGame initialization
        pygame.init()
        self.width = 800
        self.height = 600
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("UWB Distance Monitor")

        # Colors
        self.BLACK = (0, 0, 0)
        self.WHITE = (255, 255, 255)
        self.GREEN = (0, 255, 0)
        self.RED = (255, 0, 0)
        self.BLUE = (0, 0, 255)
        self.GRAY = (128, 128, 128)
        self.LIGHT_GRAY = (200, 200, 200)
        self.DARK_GREEN = (0, 150, 0)

        # Fonts
        self.font_large = pygame.font.Font(None, 48)
        self.font_medium = pygame.font.Font(None, 32)
        self.font_small = pygame.font.Font(None, 24)

        # UI elements
        self.clock = pygame.time.Clock()
        self.running = True

        self.get_logger().info('UWB Distance UI initialized')
        self.get_logger().info('Subscribing to topic: /uwb_1_responder_data')

    def distance_callback(self, msg):
        """Callback function for distance data"""
        with self.data_lock:
            self.current_distance = msg.data
            self.distance_history.append(msg.data)
            self.last_update_time = time.time()

        self.get_logger().info(f'Received distance: {msg.data:.6f} meters')

    def draw_distance_display(self):
        """Draw the main distance display"""
        # Current distance display
        distance_text = f"{self.current_distance:.3f} m"
        text_surface = self.font_large.render(distance_text, True, self.GREEN)
        text_rect = text_surface.get_rect(center=(self.width // 2, 100))
        self.screen.blit(text_surface, text_rect)

        # Label
        label_text = "Current Distance"
        label_surface = self.font_medium.render(label_text, True, self.WHITE)
        label_rect = label_surface.get_rect(center=(self.width // 2, 60))
        self.screen.blit(label_surface, label_rect)

    def draw_distance_bar(self):
        """Draw a visual distance bar (0-10 meters range)"""
        bar_x = 50
        bar_y = 200
        bar_width = self.width - 100
        bar_height = 40

        # Background bar
        pygame.draw.rect(self.screen, self.GRAY, (bar_x, bar_y, bar_width, bar_height))
        pygame.draw.rect(self.screen, self.WHITE, (bar_x, bar_y, bar_width, bar_height), 2)

        # Distance indicator (assuming max range of 150 meters)
        max_distance = 150.0
        distance_ratio = min(self.current_distance / max_distance, 1.0)
        indicator_width = int(bar_width * distance_ratio)

        # Color based on distance
        if self.current_distance < 20.0:
            color = self.RED
        elif self.current_distance < 75.0:
            color = (255, 165, 0)  # Orange
        else:
            color = self.GREEN

        pygame.draw.rect(self.screen, color, (bar_x, bar_y, indicator_width, bar_height))

        # Scale markings
        scale_values = [0, 25, 50, 75, 100, 125, 150]  # 0 to 150 meters with better spacing
        for i, val in enumerate(scale_values):
            x = bar_x + (bar_width * val / 150)
            pygame.draw.line(self.screen, self.WHITE, (x, bar_y + bar_height), (x, bar_y + bar_height + 10), 1)

            # Scale labels
            scale_text = str(val)
            scale_surface = self.font_small.render(scale_text, True, self.WHITE)
            scale_rect = scale_surface.get_rect(center=(x, bar_y + bar_height + 20))
            self.screen.blit(scale_surface, scale_rect)

    def draw_distance_graph(self):
        """Draw a real-time graph of distance readings"""
        if len(self.distance_history) < 2:
            return

        graph_x = 50
        graph_y = 300
        graph_width = self.width - 100
        graph_height = 200

        # Graph background
        pygame.draw.rect(self.screen, self.BLACK, (graph_x, graph_y, graph_width, graph_height))
        pygame.draw.rect(self.screen, self.WHITE, (graph_x, graph_y, graph_width, graph_height), 2)

        # Graph title
        title_text = "Distance History"
        title_surface = self.font_medium.render(title_text, True, self.WHITE)
        title_rect = title_surface.get_rect(center=(self.width // 2, graph_y - 20))
        self.screen.blit(title_surface, title_rect)

        # Plot the data
        history_list = list(self.distance_history)
        if len(history_list) > 1:
            max_val = max(history_list) if max(history_list) > 0 else 1.0
            min_val = min(history_list)
            range_val = max_val - min_val if max_val != min_val else 1.0

            points = []
            for i, distance in enumerate(history_list):
                x = graph_x + (i * graph_width / len(history_list))
                y = graph_y + graph_height - ((distance - min_val) / range_val * graph_height)
                points.append((x, y))

            # Draw the line graph
            if len(points) > 1:
                pygame.draw.lines(self.screen, self.GREEN, False, points, 2)

            # Draw points
            for point in points:
                pygame.draw.circle(self.screen, self.GREEN, (int(point[0]), int(point[1])), 2)

    def draw_statistics(self):
        """Draw statistics panel"""
        if not self.distance_history:
            return

        stats_x = 50
        stats_y = 520

        history_list = list(self.distance_history)
        avg_distance = sum(history_list) / len(history_list)
        min_distance = min(history_list)
        max_distance = max(history_list)

        # Statistics text
        stats = [
            f"Samples: {len(history_list)}",
            f"Average: {avg_distance:.3f} m",
            f"Min: {min_distance:.3f} m",
            f"Max: {max_distance:.3f} m"
        ]

        for i, stat in enumerate(stats):
            stat_surface = self.font_small.render(stat, True, self.WHITE)
            self.screen.blit(stat_surface, (stats_x + i * 150, stats_y))

    def draw_connection_status(self):
        """Draw connection status indicator"""
        status_x = self.width - 200
        status_y = 20

        # Check if we've received data recently (within last 2 seconds)
        time_since_update = time.time() - self.last_update_time
        is_connected = time_since_update < 2.0

        # Status indicator
        color = self.GREEN if is_connected else self.RED
        status_text = "CONNECTED" if is_connected else "DISCONNECTED"

        pygame.draw.circle(self.screen, color, (status_x, status_y), 10)

        status_surface = self.font_small.render(status_text, True, color)
        self.screen.blit(status_surface, (status_x + 20, status_y - 8))

        # Time since last update
        time_text = f"Last update: {time_since_update:.1f}s ago"
        time_surface = self.font_small.render(time_text, True, self.WHITE)
        self.screen.blit(time_surface, (status_x - 50, status_y + 20))

    def run_ui(self):
        """Main UI loop"""
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        self.running = False
                    elif event.key == pygame.K_c:
                        # Clear history
                        with self.data_lock:
                            self.distance_history.clear()

            # Clear screen
            self.screen.fill(self.BLACK)

            # Draw UI elements
            with self.data_lock:
                self.draw_distance_display()
                self.draw_distance_bar()
                self.draw_distance_graph()
                self.draw_statistics()

            self.draw_connection_status()

            # Instructions
            instructions = [
                "Press ESC to exit",
                "Press C to clear history"
            ]
            for i, instruction in enumerate(instructions):
                inst_surface = self.font_small.render(instruction, True, self.LIGHT_GRAY)
                self.screen.blit(inst_surface, (10, self.height - 40 + i * 20))

            # Update display
            pygame.display.flip()
            self.clock.tick(30)  # 30 FPS

    def destroy_node(self):
        """Clean up resources when node is destroyed"""
        self.running = False
        pygame.quit()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    try:
        uwb_ui = UWBDistanceUI()

        # Start ROS spinning in a separate thread
        ros_thread = threading.Thread(target=lambda: rclpy.spin(uwb_ui), daemon=True)
        ros_thread.start()

        # Run the UI in the main thread
        uwb_ui.run_ui()

    except KeyboardInterrupt:
        print('\nShutdown requested by user')
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'uwb_ui' in locals():
            uwb_ui.destroy_node()
        rclpy.shutdown()
        pygame.quit()
        sys.exit()

if __name__ == '__main__':
    main()
