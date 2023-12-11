import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np


class MinimalPublisherSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_publisher_subscriber')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'topic', 10)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'back_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.i = np.zeros(10)

        # Initial message sending
        self.send_message()

    def send_message(self):
        msg = Float32MultiArray()
        msg.data = self.i.tolist()
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')
        self.i = np.array(msg.data) + 1
        self.send_message()


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher_subscriber = MinimalPublisherSubscriber()
    rclpy.spin(minimal_publisher_subscriber)
    minimal_publisher_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
