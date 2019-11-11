import gym
import tensorflow as tf
import numpy as np
from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.vec_env import SubprocVecEnv
from stable_baselines import PPO2
from stable_baselines.common.policies import FeedForwardPolicy, LstmPolicy, register_policy
if __name__ == '__main__':
    ap = argparse.ArgumentParser(description='Learn or continue learning')
    ap.add_argument("--continue", "-c", default="", help="Path to continue from. If nil, start from scratch")
    ap.add_argument("-p", "--path", help="Save path")
    args = ap.parse_args()


    # multiprocess environment
    n_cpu = 8
    env = SubprocVecEnv([lambda: gym.make("Pendulum-v1") for i in range(n_cpu)])


    policy_kwargs = dict(act_fun=tf.nn.relu, net_arch=[32, 32])
    #model = PPO2.load("PPO2_cartpole_tensorboard/ppo2_cartpole_5")
    model = PPO2("MlpPolicy", env, policy_kwargs=policy_kwargs, verbose=1, tensorboard_log="results/pendulum/", nminibatches=64,
                 n_steps=2048, lam=0.95, gamma=0.99, noptepochs=10,
                 ent_coef=0.0, learning_rate=3e-4, cliprange=0.2)

    model.learn(total_timesteps=500000)
    model.save("results/pendulum/03")
