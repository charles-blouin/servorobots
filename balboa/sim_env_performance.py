import gym
from gym import spaces
import balboa.balboa_sim_base as balboa_sim
import numpy as np
import pybullet_data
import random
import os
import pybullet as p

### This environment contains the reward and the location information


class PerformanceEnvSim(gym.Env):
    def __init__(self, renders=False):
        self.renders = renders
        self.resets = 0
        self.num_timestep = 0
        p.connect(p.DIRECT)
        high_obs = np.ones(1)
        self.observation_space = spaces.Box(high_obs * -1, high_obs * 1)
        act_high = np.asarray([1])
        self.action_space = spaces.Box(-act_high, act_high)

    def render(self, mode='human'):
        self.sim.render()

    def step(self, action, ctrl=None):
        p.stepSimulation()
        done = 0

        return np.asarray([1]), 1, done, {}

    def reset(self, x=0, y=0, z=0.05, q1=0, q2=0, q3=0, q4=1, gravity = -9.81,
                    ML_R=21.5, ML_Kv=10.5, ML_Kvis=0.0005,
                    MR_R=21.5, MR_Kv=10.5, MR_Kvis=0.0005,
                    latency=0.02):
        obs = np.asarray([1])
        return obs

    def _seed(self, seed=None):
        return [0]