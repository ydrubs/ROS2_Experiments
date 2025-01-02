import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Empty

class BackgroundColorChanger(Node):
    def __init__(self):
        super().__init__('background_color_changer')

        # Service client for `/clear`
        self.clear_client = self.create_client(Empty, '/clear')
        while not self.clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for `/clear` service...')

        # Get the initial background blue value
        self.current_blue = self.get_parameter_or('background_b', 255)

    def change_background_blue(self, delta):
        """Change the background blue value by the specified delta."""
        self.current_blue = max(0, self.current_blue + delta)
        self.set_background_color(0, 0, self.current_blue)

    def set_background_color(self, r, g, b):
        """Set the background color in the turtlesim node."""
        # Set the parameters
        self.declare_parameter('background_b', b)
        self.set_parameters([
            Parameter(name='background_b', value=b),
            Parameter(name='background_r', value=r),
            Parameter(name='background_g', value=g)
        ])
        self.get_logger().info(f'Set background color to R={r}, G={g}, B={b}')

        # Call the `/clear` service
        self.clear_client.call_async(Empty.Request())

def main(args=None):
    rclpy.init(args=args)
    node = BackgroundColorChanger()

    try:
        print("Press 'm' to decrease the blue value by 5. Press 'q' to quit.")
        while rclpy.ok():
            key = input("Enter a key: ").strip().lower()
            if key == 'm':
                node.change_background_blue(-5)
            elif key == 'q':
                print("Exiting...")
                break
            else:
                print("Invalid key. Use 'm' to decrease blue or 'q' to quit.")
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
