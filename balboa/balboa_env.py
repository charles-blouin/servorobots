import gym
import time
import balboa.characterization.a_star
import numpy as np

class BalboaEnvMotor(gym.Env):
    def __init__(self):
        comms = AStar()
        self._seed()
        
        
        # Balancer observation: All in SI
        # Motor Rot Right, Rot Left (2)
        # Motor Speed right, left (2)
        # Angular velocity (3), Local frame
        # Acceleration (3), Local frame for later
        # Voltage (1)
        
        observation_dim = 10
        high_obs = np.ones([observation_dim])
        self.observation_space = spaces.Box(high_obs*0, high_obs*1.5)
        
        act_high = np.asarray([1, 1])
        self.action_space = spaces.Box(-act_high, act_high)

    def step(self, action):
        
        return np.array([rot_right, rot_left, vel_right, vel_left])
        
        
        
    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
        