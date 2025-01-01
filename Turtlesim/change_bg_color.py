import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from turtlesim.srv import Kill, Spawn

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

        # Timer to update the color
        self.timer = self.create_timer(0.1, self.update_color)

        # Service to clear turtlesim
        self.clear_client = self.create_client(Kill, '/clear')

        self.get_logger().info('BackgroundColorChanger node started. Press "+" to increment blue value.')

    def update_color(self):
        # Placeholder for key press simulation
        key_pressed = self.simulate_key_press()  # Replace this with real key press detection

        if key_pressed == '+':
            self.blue_value = min(self.blue_value + 5, 255)
            self.set_background_color()

    def set_background_color(self):
        try:
            # Update all background parameters
            self.set_parameters([
                Parameter('background_r', Parameter.Type.INTEGER, self.red_value),
                Parameter('background_g', Parameter.Type.INTEGER, self.green_value),
                Parameter('background_b', Parameter.Type.INTEGER, self.blue_value),
            ])

            # Log the updated values
            updated_b = self.get_parameter('background_b').value
            self.get_logger().info(f'Updated parameter: background_b={updated_b}')

            # Call the clear service to refresh the background
            self.call_clear_service()
        except Exception as e:
            self.get_logger().error(f'Failed to set parameter: {e}')

    def call_clear_service(self):
        if not self.clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Clear service not available!')
            return

        request = Kill.Request()
        self.clear_client.call_async(request)
        self.get_logger().info('Called /clear service to refresh the background.')

    def simulate_key_press(self):
        # This is a placeholder; replace it with actual key press detection
        return None


def main(args=None):
    rclpy.init(args=args)
    node = BackgroundColorChanger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
