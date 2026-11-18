import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')
        self.client = self.create_client(Empty, 'hello')
        # 等待服務可用
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待服務hello...')
        self.send_request()

    def send_request(self):
        request = Empty.Request()
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('呼叫服務成功！')
        except Exception as e:
            self.get_logger().error(f"呼叫服務失敗: {e}")

def main():
    rclpy.init()
    service_client = ServiceClient()
    rclpy.spin(service_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

