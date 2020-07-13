# python -m pendulum_a.pendulum_a_sim_test

import gym

env = gym.make('PendulumA-render-v0')

obs = env.reset()
print(obs)

while 1:

    obs, reward, _, _ = env.step([1])