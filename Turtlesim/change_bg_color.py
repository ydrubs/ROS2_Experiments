import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter


class SetBackgroundColor(Node):
    def __init__(self):
        super().__init__('set_background_color')

        # Declare the parameters under the /turtlesim namespace
        self.declare_parameter('/turtlesim/background_r', 0)
        self.declare_parameter('/turtlesim/background_g', 0)
        self.declare_parameter('/turtlesim/background_b', 100)  # Set the blue value to 100

        # Set the parameters to change the background color
        self.set_parameter(Parameter('/turtlesim/background_r', Parameter.Type.INTEGER, 0))
        self.set_parameter(Parameter('/turtlesim/background_g', Parameter.Type.INTEGER, 0))
        self.set_parameter(Parameter('/turtlesim/background_b', Parameter.Type.INTEGER, 100))

        # Retrieve and log the updated parameter values
        background_r = self.get_parameter('/turtlesim/background_r').value
        background_g = self.get_parameter('/turtlesim/background_g').value
        background_b = self.get_parameter('/turtlesim/background_b').value

        self.get_logger().info(f'Background color set to (R: {background_r}, G: {background_g}, B: {background_b})')


def main(args=None):
    rclpy.init(args=args)
    node = SetBackgroundColor()
    rclpy.spin_once(node)  # Run node briefly to apply the parameter change
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
