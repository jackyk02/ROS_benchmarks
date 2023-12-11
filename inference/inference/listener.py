import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np


class MinimalSubscriberPublisher(Node):

    def __init__(self):
        super().__init__('minimal_subscriber_publisher')
        self.publisher_ = self.create_publisher(
            Float32MultiArray, 'back_topic', 10)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        received_array = np.array(msg.data)
        self.get_logger().info(
            f'I heard: {received_array}, responding with {received_array}')
        self.send_message(received_array)

    def send_message(self, array):
        msg = Float32MultiArray()
        msg.data = array.tolist()
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber_publisher = MinimalSubscriberPublisher()
    rclpy.spin(minimal_subscriber_publisher)
    minimal_subscriber_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
