import rclpy
from rclpy.node import Node

class hello(Node):
    def __init__(self):
        super().__init__('hello')
        self.get_logger().info('helloworld')

def main():
    rclpy.init()
    hello_node = hello()
    hello_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':main()
