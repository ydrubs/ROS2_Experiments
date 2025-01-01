import rclpy
from rclpy.node import Node
from turtlesim.srv import SetBackgroundColor


class SetBackgroundColorNode(Node):
    def __init__(self):
        super().__init__('set_background_color_node')

        # Create a service client for the /set_background_color service
        self.client = self.create_client(SetBackgroundColor, '/turtlesim/set_background_color')

        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /turtlesim/set_background_color service to be available...')

        self.set_background_color()

    def set_background_color(self):
        # Create a request to set the background color (RGB)
        request = SetBackgroundColor.Request()
        request.r = 0  # Red value
        request.g = 0  # Green value
        request.b = 100  # Blue value

        # Call the service and send the request
        future = self.client.call_async(request)
        future.add_done_callback(self.callback)

    def callback(self, future):
        # Handle response or error from the service call
        try:
            response = future.result()
            self.get_logger().info(f'Background color set successfully: {response}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SetBackgroundColorNode()
    rclpy.spin_once(node)  # Run the node once to apply the change
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
