# python -m balboa.sim_env_balance_train
import gym
import tensorflow as tf


if __name__ == '__main__':

    log_dir = "balboa/results/"

    from stable_baselines.common.vec_env import SubprocVecEnv
    from stable_baselines import PPO2

    n_cpu = 8
    env = SubprocVecEnv([lambda: gym.make('Balboa-balance-v1') for i in range(n_cpu)])

    # Define policy
    policy_kwargs = dict(act_fun=tf.nn.relu, net_arch=[32, 32])

    # Train
    model = PPO2("MlpPolicy", env, policy_kwargs=policy_kwargs, verbose=1, tensorboard_log=log_dir, nminibatches=64,
                 n_steps=2048, lam=0.95, gamma=0.99, noptepochs=10,
                 ent_coef=0.0, learning_rate=3e-4, cliprange=0.2)

    model.learn(total_timesteps=500000, reset_num_timesteps=False)
    model.save("balboa/results/")