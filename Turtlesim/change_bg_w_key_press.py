import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from turtlesim.srv import SetParameters
from std_srvs.srv import Empty

class BackgroundColorChanger(Node):
    def __init__(self):
        super().__init__('background_color_changer')

        # Parameter client for `/turtlesim`
        self.parameter_client = self.create_client(SetParameters, '/turtlesim/set_parameters')
        while not self.parameter_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for parameter service...')

        # Service client for `/clear`
        self.clear_client = self.create_client(Empty, '/clear')
        while not self.clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for clear service...')

        # Initial parameter values
        self.current_blue = self.get_current_blue_value()

        # Monitor the keyboard input
        self.declare_key_listeners()

    def get_current_blue_value(self):
        """Retrieve the current blue value from `/turtlesim`."""
        qos = QoSProfile(depth=10)
        current_blue = 255  # Default if parameter fetching fails

        try:
            self.get_logger().info('Fetching current blue value...')
            response = self.parameter_client.call_async(
                SetParameters.Request(
                    parameters=[Parameter(name='background_b', type=Parameter.Type.INTEGER, value=current_blue)]
                )
            )
            rclpy.spin_until_future_complete(self, response)
            if response.result():
                return response.result().parameters[0].value
        except Exception as e:
            self.get_logger().error(f'Failed to fetch blue value: {e}')

        return current_blue

    def declare_key_listeners(self):
        """Listen for the 'm' key press to reduce the blue value."""
        import keyboard  # Ensure this is installed or remove if unnecessary.

        def on_key_press(event):
            if event.name == 'm':
                self.change_blue_value(-5)

        keyboard.on_press(on_key_press)

    def change_blue_value(self, delta):
        """Change the blue background color by the delta value."""
        self.current_blue = max(0, self.current_blue + delta)
        self.set_background_color(0, 0, self.current_blue)

    def set_background_color(self, r, g, b):
        """Set the background color in the turtlesim node."""
        try:
            # Update the parameters
            param_b = Parameter(name='background_b', type=Parameter.Type.INTEGER, value=b)
            request = SetParameters.Request(parameters=[param_b.to_parameter_msg()])
            response = self.parameter_client.call_async(request)
            rclpy.spin_until_future_complete(self, response)

            if response.result():
                self.get_logger().info(f'Background color updated to: R={r}, G={g}, B={b}')

            # Call the clear service to refresh the background
            clear_request = Empty.Request()
            clear_response = self.clear_client.call_async(clear_request)
            rclpy.spin_until_future_complete(self, clear_response)
        except Exception as e:
            self.get_logger().error(f'Failed to update background color: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = BackgroundColorChanger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down background color changer...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
