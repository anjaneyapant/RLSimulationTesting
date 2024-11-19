import gym
from gym import spaces
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class CustomTurtleBotEnv(gym.Env):
    
    def __init__(self):
        super().__init__()
        
        #rclpy.init()
        self.node = rclpy.create_node('turtlebot_env')
        
        self.cmd_vel_publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.laser_scan_subscriber = self.node.create_subscription(
            LaserScan, '/scan', self.laser_scan_callback, 10
        )
        
        self.laser_scan_data = None
        
        self.action_space = spaces.Discrete(3)
        self.observation_space = spaces.Box(low=0.0, high=10.0, shape=(360,), dtype=np.float32)
        
        self.collision_threshold = 0.2
        self.goal_reward = 10.0
        self.collision_penalty = -10.0
        
        
    def laser_scan_callback(self, msg):
        self.laser_scan_data = np.array(msg.ranges)
        
    def step(self, action):
        twist = Twist()
        
        if action == 0:
            twist.linear.x = 0.5
            
        elif action == 1:
            twist.angular.z = 0.5
            
        elif action == 2:
            twist.angular.z = -0.5
            
        self.cmd_vel_publisher.publish(twist)
        
        rclpy.spin_once(self.node, timeout_sec=0.1)
        
        obs = self.get_observation()
        
        reward, done = self.compute_reward_and_done(obs)
        
        return obs, reward, done, {}
    
    def reset(self):
        
        self.laser_scan_data = None
        
        twist = Twist()
        
        self.cmd_vel_publisher.publish(twist)
        
        rclpy.spin_once(self.node, timeout_sec=0.1)
        
        return self.get_observation()
    
    def render(self, mode='human'):
        pass
    
    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()
        
    def get_observation(self):
        
        if self.laser_scan_data is None:
            return np.zeros(360, dtype=np.float32)
        
        return np.clip(self.laser_scan_data, 0, 10)
    
    def compute_reward_and_done(self, obs):
        
        if np.min(obs) < self.collision_threshold:
            return self.collision_penalty, True
        
        return 1.0, False