import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32
from message_filters import ApproximateTimeSynchronizer, Subscriber
import gym
import ma_gym


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

        # Simulation
        self.env = gym.make("ma_gym:TrafficJunction4-v1")
        self.round_num = 0
        self.actions = [()] * 4
        self.total_reward = 0
        self.state = self.env.reset()
        self.done_n = [False] * 4

        self.send_message()

    def callback(self, p1, p2):
        # Logging information
        self.get_logger().info(str(p1.data))
        self.get_logger().info(str(p2.data))

        self.send_message()

    def send_message(self):
        msg = Float32MultiArray()
        flatten_list = [j for sub in self.state for j in sub]
        msg.data = flatten_list
        self.publisher.publish(msg)


def main(args=None):
    gym_make = gym.make('ma_gym:TrafficJunction4-v1')
    rclpy.init(args=args)
    env_node = Env()
    rclpy.spin(env_node)
    env_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
