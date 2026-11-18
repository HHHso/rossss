import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
class MinimalService(Node):
    def __init__ (self):
        super().__init__('minimal_service')
        self.srv = self.create_service(Empty, 'hello',self.callback)

    def callback(self, request, response):
        self.get_logger().info('hello world!')
        return response

def main():
    rclpy.init()
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':main()
