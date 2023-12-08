import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class MinimalSubscriberPublisher(Node):

    def __init__(self):
        super().__init__('minimal_subscriber_publisher')
        self.publisher_ = self.create_publisher(Int32, 'back_topic', 10)
        self.subscription = self.create_subscription(
            Int32,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        received_value = msg.data
        self.get_logger().info('I heard: "%d", responding with "%d"' %
                               (msg.data, received_value))
        self.send_message(received_value)

    def send_message(self, value):
        msg = Int32()
        msg.data = value
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber_publisher = MinimalSubscriberPublisher()
    rclpy.spin(minimal_subscriber_publisher)
    minimal_subscriber_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
