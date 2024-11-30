#!/usr/bin/env python3

import rclpy
import csv
import os
from rclpy.node import Node
from sb3_contrib import TRPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.logger import configure
import gym

#from my_gazebo_simulation.my_gazebo_scripts.custom_turtlebot_env import CustomTurtleBotEnv
from custom_turtlebot_env import CustomTurtleBotEnv


class TRPOTrainerNode(Node):
    def __init__(self):
        super().__init__('trpo_trainer_node')
        self.get_logger().info("Initializing TRPO Trainer Node...")
        
        self.log_dir = "/home/user/gym_ros_envs/logs"
        os.makedirs(self.log_dir, exist_ok = True)
        
        self.logger = configure(self.log_dir, ["stdout", "csv", "tensorboard"])

    def train(self):
        # Create the environment
        self.get_logger().info("Creating the training environment...")
        env = make_vec_env(CustomTurtleBotEnv, n_envs=1)

        # Initialize the TRPO model
        self.get_logger().info("Initializing the TRPO model...")
        model = TRPO("MlpPolicy", env, verbose=1)

        # Start training
        self.get_logger().info("Starting training...")
        #model.learn(total_timesteps=100)
        timesteps = 100
        
        episode_rewards = []
        
        def reward_log_callbacks(_locals, _globals):
            reward = _locals['infos'][0].get('episode', {}).get('r')
            if reward:
                episode_rewards.append(reward)
                self.logger.record("episode_reward", reward)
                self.logger.dump(step=_locals['env'].num_envs)
                
        model.learn(total_timesteps=timesteps, callback=reward_log_callbacks)

        # Save the model
        self.get_logger().info("Saving the trained model...")
        #model.save("trpo_turtlebot")
        model.save(os.path.join(self.log_dir, "trpo_turtlebot"))
        self.get_logger().info("Training complete!")
        
        
        with open(os.path.join(self.log_dir, "episode_rewards.csv"), "w") as file:
            writer = csv.writer(file)
            writer.writenow(["Episode", "Reward"])
            for i, r in enumerate(episode_rewards):
                writer.writenow([i+1, r])


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
