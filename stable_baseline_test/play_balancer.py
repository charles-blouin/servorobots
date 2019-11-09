import gym

import time

env = gym.make('InvertedPendulum-v0')

from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.vec_env import SubprocVecEnv
from stable_baselines import PPO2
if __name__ == '__main__':
    # multiprocess environment
    n_cpu = 4

    env = SubprocVecEnv([lambda: env for i in range(n_cpu)])
    model = PPO2.load("PPO2_cartpole_tensorboard/ppo2_cartpole_5")
    print(env)
    # Enjoy trained agent
    obs = env.reset()
    cumul = 0

    # Passing state=None to the predict function means
    # it is the initial state
    state = None
    # When using VecEnv, done is a vector
    done = [False for _ in range(env.num_envs)]
    while True:
        a = time.time()
        action, states = model.predict(obs, state=state, mask=done)
        # print(time.time()-a)
        obs, rewards, dones, info = env.step(action)
        cumul = rewards[0] + cumul
        env.render()

        if dones[0]:
            print(cumul)
            cumul = 0