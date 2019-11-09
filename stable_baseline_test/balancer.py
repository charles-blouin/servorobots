import gym

from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.vec_env import SubprocVecEnv
from stable_baselines import PPO2
if __name__ == '__main__':
    # multiprocess environment
    n_cpu = 1
    env = SubprocVecEnv([lambda: gym.make('InvertedPendulum-v0') for i in range(n_cpu)])

    #model = PPO2.load("PPO2_cartpole_tensorboard/ppo2_cartpole_5")
    model = PPO2('MlpLstmPolicy', env, verbose=1, tensorboard_log="PPO2_cartpole_tensorboard/", nminibatches=1)
    model.learn(total_timesteps=50000)
    model.save("PPO2_cartpole_tensorboard/ppo2_cartpole_6")