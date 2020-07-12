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
        self.observation_size = self.sim.observation_size
        high_obs = np.ones(self.observation_size)
        self.observation_space = spaces.Box(high_obs * -1, high_obs * 1)

        self.act_high = np.ones(self.sim.action_size)
        self.act_low = -self.act_high
        self.action_space = spaces.Box(self.act_low, self.act_high)

    def render(self, mode='human'):
        self.sim.render()

    def step(self, action, ctrl=None):
        action = np.clip(action, self.act_low, self.act_high)
        state, contact, self.time, position, orientation, local_vel, local_rot_vel = self.sim.step(action)

        if contact != 0:
            done = 1
        else:
            done = 0
        reward = local_vel[0] * position[2] * position[2]
        return state, reward, done, {}

    def reset(self, x=0, y=0, z=0.05, q1=0, q2=0, q3=0, q4=1, gravity = -9.81,
                    ML_R=21.5, ML_Kv=10.5, ML_Kvis=0.0005,
                    MR_R=21.5, MR_Kv=10.5, MR_Kvis=0.0005,
                    latency=0.02):
        obs = self.sim.reset()
        return obs

    def _seed(self, seed=None):
        return [0]