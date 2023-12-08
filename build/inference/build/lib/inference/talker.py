import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class MinimalPublisherSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_publisher_subscriber')
        self.publisher_ = self.create_publisher(Int32, 'topic', 10)
        self.subscription = self.create_subscription(
            Int32,
            'back_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.i = 0

        # Initial message sending
        self.send_message()

    def send_message(self):
        msg = Int32()
        msg.data = self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.data)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%d"' % msg.data)
        self.i += msg.data
        self.send_message()


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher_subscriber = MinimalPublisherSubscriber()
    rclpy.spin(minimal_publisher_subscriber)
    minimal_publisher_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
