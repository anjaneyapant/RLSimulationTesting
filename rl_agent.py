
import gym
import torch

class RLAgent:
    def __init__(self, env):
        self.env = env
        self.model = self.build_model()