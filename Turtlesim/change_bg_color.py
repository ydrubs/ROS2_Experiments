import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Empty
from rclpy.parameter import Parameter

#here
class UpdateTurtlesimBackground(Node):
    def __init__(self):
        super().__init__('update_turtlesim_background')

        # Create a parameter client for the /turtlesim node
        self.parameter_client = self.create_client(SetParameters, '/turtlesim/set_parameters')
        self.clear_client = self.create_client(Empty, '/clear')

        # Wait for services to become available
        self.wait_for_services()

        # Set background color
        self.set_background_color(0, 0, 100)

    def wait_for_services(self):
        while not self.parameter_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /turtlesim parameter service...')
        while not self.clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /clear service...')
        self.get_logger().info('Services are available!')

    def set_background_color(self, r, g, b):
        # Set parameters for the /turtlesim node
        self.get_logger().info(f'Setting background color to (R: {r}, G: {g}, B: {b})')

        param_r = Parameter(name='background_r', type=Parameter.Type.INTEGER, value=r)
        param_g = Parameter(name='background_g', type=Parameter.Type.INTEGER, value=g)
        param_b = Parameter(name='background_b', type=Parameter.Type.INTEGER, value=b)

        request = SetParameters.Request(parameters=[param_r.to_parameter_msg(),
                                                    param_g.to_parameter_msg(),
                                                    param_b.to_parameter_msg()])
        self.parameter_client.call_async(request)

        # Call /clear service to apply changes
        self.clear_background()

    def clear_background(self):
        request = Empty.Request()
        self.clear_client.call_async(request)
        self.get_logger().info('Background cleared and updated!')

def main(args=None):
    rclpy.init(args=args)
    node = UpdateTurtlesimBackground()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
