# coding=UTF-8 
import rclpy 
from rclpy.node import Node 
from std_msgs.msg import String 

class simple_sub(Node): 
    def __init__(self): 
        super().__init__('simple_sub') 
        self.subscription = self.create_subscription(String,"chatter",self.listener_callback,10) 
        self.subscription #prevent unused variable warning 

    def listener_callback(self, msg): 
        self.get_logger().info('I heard: "%s"' % msg.data) 

def main():
    rclpy.init() 
    node_subscriber = simple_sub() 
    rclpy.spin(node_subscriber) 
    node_subscriber.destroy_node() 
    rclpy.shutdown()
	
22.	if __name__ == '__main__': main()
