# python -m balboa.sim_env_balance_train
import gym
import tensorflow as tf
import balboa.utils
import stable_baselines
import argparse

ap = argparse.ArgumentParser(description='Learn or continue learning')
ap.add_argument("-i", "--load_id", default=None, help="Start from test id")
ap.add_argument("-a", "--algo", default="ppo2", help="Algorithm")
args = ap.parse_args()

log_dir = "balboa/results/"
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
    if (n_steps + 1) % 2 == 0:
        print("########## Saving ##########")
        # _locals['self'].save(log_dir + 'best_model_' + id + '.pkl')
        n_steps += 1
    return True

if __name__ == '__main__':

    from stable_baselines.common.vec_env import SubprocVecEnv
    from stable_baselines import PPO2
    from stable_baselines import A2C

    n_cpu = 8
    env = SubprocVecEnv([lambda: gym.make('Balboa-balance-ctrl-v1') for i in range(n_cpu)])

    # Define policy
    policy_kwargs = dict(act_fun=tf.nn.relu, net_arch=[32, 32])


    # Train
    if args.algo == "ppo2":
        if args.load_id == None:
            model = PPO2("MlpPolicy", env, policy_kwargs=policy_kwargs, verbose=1, tensorboard_log=log_dir, nminibatches=32,
                         n_steps=2048, lam=0.95, gamma=0.99, noptepochs=10,
                         ent_coef=0.001, cliprange=0.2)
            model.learning_rate = stable_baselines.common.schedules.LinearSchedule(1.0, 0.001, initial_p=0.0005).value
        else:
            model = PPO2.load(log_dir + str(args.load_id) + ".zip", env=env)
            model.tensorboard_log = log_dir
    if args.algo == "a2c":
        if args.load_id == None:

            model = A2C("MlpPolicy", env, policy_kwargs=policy_kwargs, verbose=1, tensorboard_log=log_dir, nminibatches=32,
                         n_steps=2048, lam=0.95, gamma=0.99, noptepochs=10,
                         ent_coef=0.001, learning_rate=5e-4, cliprange=0.2)
            # model.learning_rate = stable_baselines.common.schedules.LinearSchedule(1.0, 0.001, initial_p=0.0005).value
        else:
            model = PPO2.load(log_dir + str(args.load_id) + ".zip", env=env)
            model.tensorboard_log = log_dir

    model.learn(total_timesteps=2000000, reset_num_timesteps=False, callback=callback)
    model.save(log_dir + str(id+1))