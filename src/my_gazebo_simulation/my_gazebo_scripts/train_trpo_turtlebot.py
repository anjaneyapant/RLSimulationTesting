#!/usr/bin/env python3

import rclpy
import csv
import os
from rclpy.node import Node
from sb3_contrib import TRPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.logger import configure
import gym
from torch.utils.tensorboard import SummaryWriter
from stable_baselines3.common.callbacks import BaseCallback

#from my_gazebo_simulation.my_gazebo_scripts.custom_turtlebot_env import CustomTurtleBotEnv
from custom_turtlebot_env import CustomTurtleBotEnv

class RewardLoggingCallback(BaseCallback):
    def __init__(self, log_dir, writer, verbose=0):
        super(RewardLoggingCallback, self).__init__(verbose)
        self.log_dir = log_dir
        self.writer = writer
        self.episode_rewards = []
        
    def _on_step(self):
        info = self.locals['infos'][0]
        
        if 'episode' in info:
            reward = info['episode']['r']
            self.episode_rewards.append(reward)
            self.logger.record("episode_reward", reward)
            self.writer.add_scalar("Rewards/Episode", reward, len(self.episode_rewards))
            
        return True
    
    def save_rewards(self):
        with open(os.path.join(self.log_dir, "episode_rewards.csv"), "w") as file:
            writer = csv.writer(file)
            writer.writerow(["Episode", "Reward"])
            
            for i, r in enumerate(self.episode_rewards):
                writer.writerow([i+1, r])
    


class TRPOTrainerNode(Node):
    def __init__(self):
        super().__init__('trpo_trainer_node')
        self.get_logger().info("Initializing TRPO Trainer Node...")
        
        #self.log_dir = "/home/user/gym_ros_envs/logs"
        self.log_dir = "/home/aj_karti/RLSimulationTesting/logs"
        self.get_logger().info(f"Log Directory: {self.log_dir}")
        os.makedirs(self.log_dir, exist_ok = True)
        
        self.logger = configure(self.log_dir, ["stdout", "csv", "tensorboard"])
        self.writer = SummaryWriter(log_dir=self.log_dir)

    def train(self):
        # Create the environment
        self.get_logger().info("Creating the training environment...")
        env = make_vec_env(CustomTurtleBotEnv, n_envs=1)

        # Initialize the TRPO model
        self.get_logger().info("Initializing the TRPO model...")
        model = TRPO("MlpPolicy", env, verbose=1, device="cpu")

        # Start training
        self.get_logger().info("Starting training...")
        #model.learn(total_timesteps=100)
        timesteps = 100
        
        callback = RewardLoggingCallback(self.log_dir, self.writer)
        model.learn(total_timesteps=timesteps, callback=callback)
        
        # Save the model
        self.get_logger().info("Saving the trained model...")
        #model.save("trpo_turtlebot")
        model.save(os.path.join(self.log_dir, "trpo_turtlebot"))
        self.get_logger().info("Training complete!")
        
        callback.save_rewards()        
        self.writer.flush()
        self.writer.close()


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
