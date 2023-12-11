import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32
import numpy as np


class Policy2(Node):

    def __init__(self):
        super().__init__('policy2')
        self.publisher_ = self.create_publisher(
            Int32, 'policy_topic2', 10)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'env_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        received_array = np.array(msg.data[:324])
        self.get_logger().info(
            f'I heard: {received_array}, responding with {received_array}')
        self.send_message(received_array)

    def send_message(self, array):
        msg = Int32()
        msg.data = 1
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    policy_node = Policy2()
    rclpy.spin(policy_node)
    policy_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
