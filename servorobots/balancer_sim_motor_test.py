import gym
import time
env = gym.make('Motor_balancer-render-v0')

from stable_baselines.common.policies import MlpPolicy
# from stable_baselines.common.vec_env import SubprocVecEnv
from stable_baselines import PPO2
if __name__ == '__main__':
    # multiprocess environment
    n_cpu = 1

    # env = SubprocVecEnv([lambda: env for i in range(n_cpu)])
    # model = PPO2.load("PPO2_cartpole_tensorboard/ppo2_cartpole_5")
    print(env)
    # Enjoy trained agent
    obs = env.reset()
    cumul = 0

    # Passing state=None to the predict function means
    # it is the initial state
    state = None
    # When using VecEnv, done is a vector
#    done = [False for _ in range(env.num_envs)]
    i = 0
    while True:
        a = time.time()
        # action, states = model.predict(obs, state=state, mask=done)
        # print(time.time()-a)
        obs, rewards, dones, info = env.step(env.action_space.sample())
        env.render()
        i = i + 1
        if i == 200:
            env.reset()
            i = 0
