# python -m walker_a.walker_a_sim_train
import gym
import tensorflow as tf
import balboa.utils
import stable_baselines
import argparse
import random
import xml.dom.minidom
import pybulletgym


ap = argparse.ArgumentParser(description='Learn or continue learning')
ap.add_argument("-i", "--load_id", default=None, help="Start from test id")
ap.add_argument("-a", "--algo", default="ppo2", help="Algorithm")
args = ap.parse_args()

log_dir = "walker_a/results/"
# New id
id = balboa.utils.tensorboard_latest_directory_number(log_dir)
n_steps = 0
def callback(_locals, _globals):
    """
    Callback called at each step (for DQN an others) or after n steps (see ACER or PPO2)
    :param _locals: (dict)
    :param _globals: (dict)
    """
    global n_steps, best_mean_reward
    # Print stats every 1000 calls
    print(n_steps)
    if (n_steps + 1) % 1 == 0:
        print("########## Saving ##########")
        n_steps += 1
    return True

if __name__ == '__main__':

    from stable_baselines.common.vec_env import SubprocVecEnv
    from stable_baselines import PPO2
    from stable_baselines import A2C

    n_cpu = 12

    env = SubprocVecEnv([lambda: gym.make('WalkerA-v0') for i in range(n_cpu)])

    # Define policy
    policy_kwargs = dict(act_fun=tf.nn.tanh, net_arch=[64, 64])
    print("Tensorflow ########################################")
    sess = tf.Session(config=tf.ConfigProto(log_device_placement=True))
    # Train
    if args.algo == "ppo2":
        if args.load_id == None:
            # tensorboard_log=log_dir
            model = PPO2("MlpPolicy", env, policy_kwargs=policy_kwargs, verbose=1, nminibatches=32,
                         n_steps=256, lam=0.95, gamma=0.99, noptepochs=10,
                         ent_coef=0.0, cliprange=0.2)
        else:
            print("Loading model: " + str(args.load_id))
            model = PPO2.load(log_dir + str(args.load_id) + ".zip", env=env)
        model.tensorboard_log = log_dir
        model.learning_rate = stable_baselines.common.schedules.LinearSchedule(1.0, 0.00025, initial_p=0.00025  ).value

    model.learn(total_timesteps=10000000, reset_num_timesteps=False, callback=callback)
    model.save(log_dir + str(id+1))