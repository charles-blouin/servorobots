# python -m walker_a.walker_a_sim_test

import gym

env = gym.make('WalkerA-v0')

obs = env.reset()
print(obs)

while 1:

    obs, reward, _, _ = env.step([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])