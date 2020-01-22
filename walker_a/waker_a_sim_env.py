import pybullet as p
import gym
import gym.spaces as spaces
import numpy as np
import walker_a.walker_a_sim_base as base

### This environment contains the reward and the location information


class WalkerA(gym.Env):
    def __init__(self, renders=False):
        self.renders = renders
        self.sim = base.WalkerASim(renders=renders)
        self.resets = 0
        self.num_timestep = 0
        high_obs = np.ones(1)
        self.observation_space = spaces.Box(high_obs * -1, high_obs * 1)
        act_high = np.asarray([1])
        self.action_space = spaces.Box(-act_high, act_high)

    def render(self, mode='human'):
        self.sim.render()

    def step(self, action, ctrl=None):
        self.sim.step(action)
        done = 0

        return np.asarray([1]), 1, done, {}

    def reset(self, x=0, y=0, z=0.05, q1=0, q2=0, q3=0, q4=1, gravity = -9.81,
                    ML_R=21.5, ML_Kv=10.5, ML_Kvis=0.0005,
                    MR_R=21.5, MR_Kv=10.5, MR_Kvis=0.0005,
                    latency=0.02):
        self.sim.reset()

        obs = np.asarray([1])
        return obs

    def _seed(self, seed=None):
        return [0]