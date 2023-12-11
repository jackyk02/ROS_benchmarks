import rclpy
from rclpy.node import Node
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import numpy as np


class MinimalSubscriberPublisher(Node):

    def __init__(self):
        super().__init__('minimal_subscriber_publisher')
        self.publisher_ = self.create_publisher(
            numpy_msg(Floats), 'back_topic', 10)
        self.subscription = self.create_subscription(
            numpy_msg(Floats),
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        received_array = np.array(msg.data)
        self.get_logger().info('I heard: "%s", responding with "%s"' %
                               (received_array, received_array))
        self.send_message(received_array)

    def send_message(self, value):
        msg = Floats()
        msg.data = value
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber_publisher = MinimalSubscriberPublisher()
    rclpy.spin(minimal_subscriber_publisher)
    minimal_subscriber_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
