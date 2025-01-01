import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Empty
import curses


class BackgroundColorChanger(Node):
    def __init__(self):
        super().__init__('background_color_changer')

        # Declare parameters
        self.declare_parameter('background_r', 0)
        self.declare_parameter('background_g', 0)
        self.declare_parameter('background_b', 255)

        # Initialize color values
        self.red_value = self.get_parameter('background_r').value
        self.green_value = self.get_parameter('background_g').value
        self.blue_value = self.get_parameter('background_b').value

        # Create client for the /clear service
        self.clear_client = self.create_client(Empty, '/clear')

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
            f'Attempting to set parameters: R={self.red_value}, G={self.green_value}, B={self.blue_value}')
        try:
            self.set_parameters([
                Parameter('background_r', Parameter.Type.INTEGER, self.red_value),
                Parameter('background_g', Parameter.Type.INTEGER, self.green_value),
                Parameter('background_b', Parameter.Type.INTEGER, self.blue_value),
            ])
            # Log the updated parameter value
            self.get_logger().info(f'Updated background_b to {self.get_parameter("background_b").value}')
            self.call_clear_service()
        except Exception as e:
            self.get_logger().error(f'Failed to set parameters: {e}')

    def call_clear_service(self):
        if not self.clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Clear service not available!')
            return

        self.get_logger().info('Calling /clear service...')
        request = Empty.Request()
        self.clear_client.call_async(request)
        self.get_logger().info('Successfully called /clear service.')

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
