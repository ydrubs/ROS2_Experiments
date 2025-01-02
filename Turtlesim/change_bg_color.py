import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Empty


class SetTurtlesimBackground(Node):
    def __init__(self):
        super().__init__('set_turtlesim_background')

        # Set background color parameters for turtlesim node
        self.declare_parameter('/turtlesim/background_r', 0)
        self.declare_parameter('/turtlesim/background_g', 0)
        self.declare_parameter('/turtlesim/background_b', 100)

        self.set_parameter(Parameter('/turtlesim/background_r', Parameter.Type.INTEGER, 0))
        self.set_parameter(Parameter('/turtlesim/background_g', Parameter.Type.INTEGER, 0))
        self.set_parameter(Parameter('/turtlesim/background_b', Parameter.Type.INTEGER, 100))

        # Call the /clear service to apply changes
        self.clear_service = self.create_client(Empty, '/clear')
        while not self.clear_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /clear service...')

        self.send_clear_request()

    def send_clear_request(self):
        request = Empty.Request()
        self.clear_service.call_async(request)
        self.get_logger().info('Background cleared and updated!')


def main(args=None):
    rclpy.init(args=args)
    node = SetTurtlesimBackground()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
