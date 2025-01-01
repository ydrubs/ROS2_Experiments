import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from turtlesim.srv import SetPen
import curses


class BackgroundColorChanger(Node):
    def __init__(self):
        super().__init__('background_color_changer')

        # Declare parameters (optional, if you want to store the background colors)
        self.declare_parameter('background_r', 0)
        self.declare_parameter('background_g', 0)
        self.declare_parameter('background_b', 255)

        # Initialize color values (using parameters)
        self.red_value = self.get_parameter('background_r').value
        self.green_value = self.get_parameter('background_g').value
        self.blue_value = self.get_parameter('background_b').value

        # Create client for the /set_background_color service (direct color change)
        self.set_background_client = self.create_client(SetPen, '/turtle1/set_pen')

        # Create a timer to handle key presses
        self.get_logger().info('BackgroundColorChanger node started. Press "+" to increment blue value.')

        # Start a background thread for key press detection using curses
        self.screen = curses.initscr()
        curses.noecho()
        curses.cbreak()
        self.screen.keypad(True)

        # Create a timer to run the curses loop
        self.create_timer(0.1, self.handle_key_presses)

    def handle_key_presses(self):
        # Non-blocking key press detection
        key = self.screen.getch()

        if key == ord('+'):  # Detect '+' key
            self.get_logger().info(
                f'Key "+" pressed. Incrementing blue value from {self.blue_value} to {min(self.blue_value + 5, 255)}')
            self.blue_value = min(self.blue_value + 5, 255)
            self.set_background_color()

    def set_background_color(self):
        self.get_logger().info(
            f'Updating background color to: R={self.red_value}, G={self.green_value}, B={self.blue_value}')

        # Call the set_background_color service
        try:
            # Change the pen color and update the background color directly
            if not self.set_background_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('Service not available!')
                return

            request = SetPen.Request()
            request.r = self.red_value
            request.g = self.green_value
            request.b = self.blue_value
            request.width = 0  # No pen width change (just background color)
            request.off = False  # Turn on the pen (show the color change)

            self.set_background_client.call_async(request)
            self.get_logger().info(
                f'Successfully called set_background_color service with R={self.red_value}, G={self.green_value}, B={self.blue_value}')
        except Exception as e:
            self.get_logger().error(f'Failed to call set_background_color service: {e}')

    def destroy_node(self):
        # Clean up curses
        curses.endwin()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BackgroundColorChanger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
