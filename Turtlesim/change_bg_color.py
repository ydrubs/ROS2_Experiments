import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

class SetBackgroundColor(Node):
    def __init__(self):
        super().__init__('set_background_color')

        # Declare the background_b parameter
        self.declare_parameter('background_b', 100)

        # Set the background_b parameter to 100
        self.set_parameter(Parameter('background_b', Parameter.Type.INTEGER, 100))

        self.get_logger().info('Set background_b parameter to 100')

def main(args=None):
    rclpy.init(args=args)
    node = SetBackgroundColor()
    rclpy.spin_once(node)  # Run the node once to apply the change
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
