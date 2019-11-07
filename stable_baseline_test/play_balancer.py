import gym

import time

env = gym.make('InvertedPendulum-v0')

from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.vec_env import SubprocVecEnv
from stable_baselines import PPO2
if __name__ == '__main__':
    # multiprocess environment
    n_cpu = 1

    env = SubprocVecEnv([lambda: env for i in range(n_cpu)])
    model = PPO2.load("ppo2_cartpole")
    print(env)
    # Enjoy trained agent
    obs = env.reset()
    while True:
        a = time.time()
        action, _states = model.predict(obs)
        print(time.time()-a)
        obs, rewards, dones, info = env.step(action)
        env.render()