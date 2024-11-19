#!/usr/bin/env python3

from sb3_contrib import TRPO
from stable_baselines3.common.env_util import make_vec_env
import gym

from my_gazebo_simulation.my_gazebo_scripts.custom_turtlebot_env import CustomTurtleBotEnv

env = make_vec_env(CustomTurtleBotEnv, n_envs=1)

model = TRPO("MlpPolicy", env, verbose=1)

model.learn(total_timesteps=100)

model.save("trpo_turtlebot")

print("Training complete!")