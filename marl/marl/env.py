import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32
from message_filters import ApproximateTimeSynchronizer, Subscriber


class Env(Node):
    def __init__(self):
        super().__init__('env')

        # Subscribers
        self.policy1 = Subscriber(self, Int32, 'policy_topic1')
        self.policy2 = Subscriber(self, Int32, 'policy_topic2')

        # Synchronize the subscribers
        ats = ApproximateTimeSynchronizer(
            [self.policy1, self.policy2], 10, 0.1, allow_headerless=True)
        ats.registerCallback(self.callback)

        # Publisher
        self.publisher = self.create_publisher(
            Float32MultiArray, 'env_topic', 10)

        self.send_message()

    def callback(self, p1, p2):
        # Logging information
        self.get_logger().info(str(p1.data))
        self.get_logger().info(str(p2.data))

        self.send_message()

        # Publishing a message
        msg = Float32MultiArray()
        msg.data = [1.0] * 10  # Creating a zero-filled list
        self.publisher.publish(msg)

    def send_message(self):
        msg = Float32MultiArray()
        msg.data = [1.0] * 10  # Creating a zero-filled list
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    env_node = Env()
    rclpy.spin(env_node)
    env_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
