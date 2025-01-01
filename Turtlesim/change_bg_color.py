import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
import curses


class BackgroundColorChanger(Node):
    def __init__(self):
        super().__init__('background_color_changer')

        # Initialize the background blue value
        self.blue_value = 0

        # Set up the parameter
        self.set_parameter(rclpy.parameter.Parameter('background_b', rclpy.Parameter.Type.INTEGER, self.blue_value))

        # Initialize curses for key press detection
        self.stdscr = curses.initscr()
        curses.cbreak()
        self.stdscr.keypad(1)
        curses.noecho()

        self.run()

    def run(self):
        while True:
            key = self.stdscr.getch()

            if key == ord('+'):
                # Increment blue value by 5
                self.blue_value = min(self.blue_value + 5, 255)
                self.set_background_color()
                self.get_logger().info(f'Blue value set to: {self.blue_value}')

            if key == ord('q'):  # Press 'q' to quit
                break

    def set_background_color(self):
        # Set the new blue value on the turtlesim background
        self.get_parameter('background_b').value = self.blue_value
        self.get_node_parameters_interface().set_parameters([self.get_parameter('background_b')])


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
