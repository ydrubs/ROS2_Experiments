#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import curses


class BackgroundColorChanger(Node):
    def __init__(self):
        super().__init__('background_color_changer')

        # Declare the initial background blue value parameter
        self.declare_parameter('background_b', 0)
        self.blue_value = self.get_parameter('background_b').value
        self.get_logger().info(f'Initial background_b value: {self.blue_value}')

        # Initialize curses for key press detection
        self.stdscr = curses.initscr()
        curses.cbreak()
        self.stdscr.keypad(1)
        curses.noecho()

        self.run()

    def run(self):
        self.get_logger().info('Press "+" to increase blue or "q" to quit.')
        while True:
            key = self.stdscr.getch()
            self.get_logger().info(f'Key pressed: {key}')

            if key == ord('+'):
                # Increment blue value by 5
                self.blue_value = min(self.blue_value + 5, 255)
                self.set_background_color()
                self.get_logger().info(f'Blue value set to: {self.blue_value}')

            if key == ord('q'):  # Press 'q' to quit
                self.get_logger().info('Exiting...')
                break

    def set_background_color(self):
        # Update the 'background_b' parameter
        self.set_parameters([
            rclpy.parameter.Parameter('background_b', rclpy.Parameter.Type.INTEGER, self.blue_value)
        ])
        self.get_logger().info(f'Set parameter background_b to {self.blue_value}')


def main(args=None):
    rclpy.init(args=args)
    node = BackgroundColorChanger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        curses.endwin()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
