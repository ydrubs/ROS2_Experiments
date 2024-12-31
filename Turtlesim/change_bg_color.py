#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import curses
from std_srvs.srv import Empty


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

        # Prepare the clear service client
        self.clear_client = self.create_client(Empty, '/clear')
        while not self.clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /clear service...')

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

        # Call the /clear service to apply the changes
        self.call_clear_service()

    def call_clear_service(self):
        request = Empty.Request()
        future = self.clear_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Background refreshed.')
        else:
            self.get_logger().error('Failed to call /clear service.')


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
