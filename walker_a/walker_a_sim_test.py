# python -m walker_a.walker_a_sim_test

import gym

env = gym.make('WalkerA-v0')

env.reset()

while 1:
    env.step([1])