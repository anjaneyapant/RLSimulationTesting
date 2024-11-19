#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sb3_contrib import TRPO
from stable_baselines3.common.env_util import make_vec_env
import gym

#from my_gazebo_simulation.my_gazebo_scripts.custom_turtlebot_env import CustomTurtleBotEnv
from src.my_gazebo_simulation.my_gazebo_scripts.custom_turtlebot_env import CustomTurtleBotEnv


class TRPOTrainerNode(Node):
    def __init__(self):
        super().__init__('trpo_trainer_node')
        self.get_logger().info("Initializing TRPO Trainer Node...")

    def train(self):
        # Create the environment
        self.get_logger().info("Creating the training environment...")
        env = make_vec_env(CustomTurtleBotEnv, n_envs=1)

        # Initialize the TRPO model
        self.get_logger().info("Initializing the TRPO model...")
        model = TRPO("MlpPolicy", env, verbose=1)

        # Start training
        self.get_logger().info("Starting training...")
        model.learn(total_timesteps=100)

        # Save the model
        self.get_logger().info("Saving the trained model...")
        model.save("trpo_turtlebot")
        self.get_logger().info("Training complete!")


def main(args=None):
    rclpy.init(args=args)
    trainer_node = TRPOTrainerNode()

    try:
        trainer_node.train()
    except Exception as e:
        trainer_node.get_logger().error(f"An error occurred during training: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
