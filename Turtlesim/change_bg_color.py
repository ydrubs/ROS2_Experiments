import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty


class UpdateTurtlesimBackground(Node):
    def __init__(self):
        super().__init__('update_turtlesim_background')

        # Create a parameter client for the turtlesim node
        self.parameter_client = self.create_client(rclpy.parameter.Parameter, '/turtlesim')

        # Create a service client to call the /clear service
        self.clear_client = self.create_client(Empty, '/clear')

        # Wait for the turtlesim services to become available
        self.wait_for_services()

        # Update the background color parameters
        self.update_background_color(0, 0, 100)

        # Call the /clear service to apply the changes
        self.call_clear_service()

    def wait_for_services(self):
        self.get_logger().info('Waiting for turtlesim services...')
        while not self.parameter_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for parameter service...')
        while not self.clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /clear service...')
        self.get_logger().info('All services are available!')

    def update_background_color(self, r, g, b):
        self.get_logger().info(f'Setting background color to (R: {r}, G: {g}, B: {b})')

        # Use ros2 parameter command interface to update turtlesim's parameters
        self.declare_parameter('/turtlesim/background_r', 0)
        self.get_service().send_set():


def main(args=None):
    rclpy.init(args=args)
    node = UpdateTurtlesimBackground()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
