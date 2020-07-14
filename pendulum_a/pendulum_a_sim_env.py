import gym
import gym.spaces as spaces
import numpy as np
import pendulum_a.pendulum_a_sim_base as base

### This environment contains the reward and the location information


class PendulumA(gym.Env):
    def __init__(self, renders=False):
        self.renders = renders
        self.sim = base.PendulumASim(renders=renders)
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
        state, self.time = self.sim.step(action)
        done = 0
        # print(state)
        if (state[0] > -1) and (state[0] < 1) and (state[1] > -1) and (state[1] < 1):
            # reward = 10-state[1]**2
            reward = 1  # (3 - np.absolute((state[0])) - np.absolute(state[1])) # More points if closer to straigth
            # More points if going slow
            # + min( np.absolute(state[3]), 4) + \
            #  min(1 / np.absolute(state[2]), 4)
            reward -= np.absolute(action[0])
            # print(action)
            # print(reward)
        else:
            reward = 0
        if (state[0] > 20) or (state[0] < -20) or (state[1] > 20) or (state[1] < -20):
            done = 1
        # print(state)
        return state, reward, done, {}

    def reset(self):
        obs = self.sim.reset()
        return obs

    def _seed(self, seed=None):
        return [0]