import gym
import tensorflow as tf
import argparse
import numpy as np
from stable_baselines.results_plotter import load_results, ts2xy
from stable_baselines.common.vec_env import SubprocVecEnv
from stable_baselines import PPO2

# To start from scratch
# python -m stable_baseline_test.train_upright -p "results/pendulum/01" -e Pendulum-v1
# To continue
# python -m stable_baseline_test.train_upright -p "results/pendulum/08" -c "results/pendulum/06" -e Pendulum-v1

if __name__ == '__main__':
    ap = argparse.ArgumentParser(description='Learn or continue learning')
    ap.add_argument("-c", "--cont", default="", help="Path to continue from. If nil, start from scratch")
    ap.add_argument("-p", "--path", help="Save path")
    ap.add_argument("-e", "--env", help="Save path")
    args = ap.parse_args()

    best_mean_reward, n_steps = -np.inf, 0


    def callback(_locals, _globals):
        """
        Callback called at each step (for DQN an others) or after n steps (see ACER or PPO2)
        :param _locals: (dict)
        :param _globals: (dict)
        """
        global n_steps, best_mean_reward
        # Print stats every 1000 calls
        if (n_steps + 1) % 1 == 0:
            print("Hi")
            _locals['self'].save(log_dir + 'best_model.pkl')
        n_steps += 1
        return True


    # Create log dir
    log_dir = "results/pendulum/"


    # multiprocess environment
    n_cpu = 8
    env = SubprocVecEnv([lambda: gym.make(args.env) for i in range(n_cpu)])

    if args.cont == "":
        policy_kwargs = dict(act_fun=tf.nn.relu, net_arch=[32, 32])
        model = PPO2("MlpPolicy", env, policy_kwargs=policy_kwargs, verbose=1, tensorboard_log="results/pendulum/", nminibatches=64,
                     n_steps=2048, lam=0.95, gamma=0.99, noptepochs=10,
                     ent_coef=0.0, learning_rate=3e-4, cliprange=0.2)
    else:
        model = PPO2.load(args.cont, env=env)
        model.tensorboard_log = "results/pendulum/"

    model.learn(total_timesteps=500000, reset_num_timesteps=False, callback=callback)
    model.save(args.path)
