import rclpy
from rclpy.node import Node
from turtlesim.srv import SetPen

class TurtleBgColorChanger(Node):
    def __init__(self):
        super().__init__('turtle_bg_color_changer')
        self.cli = self.create_client(SetPen, '/turtlesim/set_pen')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetPen.Request()

    def send_request(self, r, g, b):
        self.req.r = r
        self.req.g = g
        self.req.b = b
        self.req.width = 0  # Set width to 0 to only change the background color
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    color_changer = TurtleBgColorChanger()
    color_changer.send_request(255, 0, 0)  # Change background to red
    rclpy.spin_until_future_complete(color_changer, color_changer.future)
    rclpy.shutdown()

if __name__ == '__main__':
    main()