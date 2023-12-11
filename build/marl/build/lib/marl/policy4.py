import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32

# Preamble
import gym
import numpy as np
import torch
import torch.nn as nn
import time
import logging

env = gym.make("ma_gym:TrafficJunction4-v1")
EPISODES = 10000
amount = 100
logging.basicConfig(filename="infer_log.log", level=logging.INFO,
                    format="%(asctime)s - %(levelname)s - %(message)s")
# End of Preamble


class PolicyNetwork(nn.Module):
    def __init__(self, obs_space, action_space):
        super(PolicyNetwork, self).__init__()
        self.fc = nn.Linear(obs_space, 64)
        self.action_head = nn.Linear(64, action_space)

    def forward(self, x):
        x = torch.relu(self.fc(x))
        action_probs = torch.softmax(self.action_head(x), dim=-1)
        return action_probs


def load_policy(agent_idx):
    policy = PolicyNetwork(
        env.observation_space[agent_idx].shape[0], env.action_space[agent_idx].n)
    # absolute path
    policy_load_path = f"/mnt/c/Users/jacky/Desktop/ros/example/policy_agent_{agent_idx}.pth"
    policy.load_state_dict(torch.load(policy_load_path))
    policy.eval()
    return policy


class policy4(Node):

    def __init__(self):
        super().__init__('policy4')
        self.publisher_ = self.create_publisher(
            Int32, 'policy_topic4', 10)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'env_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # other client code
        self.agent_idx = 3
        self.policy = load_policy(self.agent_idx)

    def listener_callback(self, msg):
        received_array = np.array(msg.data[:324])
        state = torch.from_numpy(np.array(received_array)).float().unsqueeze(0)
        with torch.no_grad():
            probs = self.policy(state)
        action = torch.argmax(probs, dim=-1)

        self.send_message(int(action.item()))

    def send_message(self, action_val):
        msg = Int32()
        msg.data = action_val
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    policy_node = policy4()
    rclpy.spin(policy_node)
    policy_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
