# python -m pendulum_a.pendulum_a_sim_train
import gym
import tensorflow as tf
import balboa.utils
import stable_baselines
import argparse



ap = argparse.ArgumentParser(description='Learn or continue learning')
ap.add_argument("-i", "--load_id", default=None, help="Start from test id")
ap.add_argument("-a", "--algo", default="ppo2", help="Algorithm")
args = ap.parse_args()

log_dir = "pendulum_a/results/"
# New id

n_steps = 0
def callback(_locals, _globals):
    """
    Callback called at each step (for DQN an others) or after n steps (see ACER or PPO2)
    :param _locals: (dict)
    :param _globals: (dict)
    """
    global n_steps, best_mean_reward
    # Print stats every 100 calls
    # print(n_steps)
    if (n_steps + 1) % 100 == 0:
        print("########## Saving ##########")
        n_steps += 1
    return True

if __name__ == '__main__':

    from stable_baselines.common.vec_env import SubprocVecEnv
    from stable_baselines import PPO2
    from stable_baselines import A2C
    from stable_baselines import ACKTR

    n_cpu = 12

    env = SubprocVecEnv([lambda: gym.make('PendulumA-v0') for i in range(n_cpu)])

    # Define policy
    policy_kwargs = dict(act_fun=tf.nn.tanh, net_arch=[32, 32])
    print("Tensorflow ########################################")
    sess = tf.Session(config=tf.ConfigProto(log_device_placement=True))
    # Train
    if args.algo == "ppo2":
        id = balboa.utils.tensorboard_latest_directory_number(log_dir, 'PPO2_')
        if args.load_id == None:
            # tensorboard_log=log_dir
            model = PPO2("MlpPolicy", env, policy_kwargs=policy_kwargs, verbose=1, nminibatches=1,
                         n_steps=48, lam=0.94, gamma=0.98, noptepochs=20,
                         ent_coef=0.0, cliprange=0.1)
        else:
            print("Loading model: " + str(args.load_id))
            model = PPO2.load(log_dir + 'PPO_' + str(args.load_id) + ".zip", env=env)
        model.tensorboard_log = log_dir
        model.learning_rate = stable_baselines.common.schedules.LinearSchedule(1.0, 0.00025, initial_p=0.00024).value
        # model.cliprange = stable_baselines.common.schedules.LinearSchedule(1.0, 0.2, initial_p=0).value
        model.learn(total_timesteps=2000000, reset_num_timesteps=False, callback=callback)
        model.save(log_dir + 'model_PPO_' + str(id+1))

    if args.algo == "acktr":
        id = balboa.utils.tensorboard_latest_directory_number(log_dir, 'ACKTR_')
        print('Using acktr')
        if args.load_id == None:
            # tensorboard_log=log_dir
            model = ACKTR("MlpPolicy", env, policy_kwargs=policy_kwargs,  ent_coef=0.0,
                          verbose=1)
            # verbose=1, n_steps=48, learning_rate=0.1, lr_schedule='constant',
        else:
            print("Loading model: " + str(args.load_id))
            model = ACKTR.load(log_dir + 'ACKTR_' + str(args.load_id) + ".zip", env=env)
        model.tensorboard_log = log_dir
        # model.learning_rate = stable_baselines.common.schedules.LinearSchedule(1.0, 0.06, initial_p=0.06).value
        # model.cliprange = stable_baselines.common.schedules.LinearSchedule(1.0, 0.2, initial_p=0).value

        model.learn(total_timesteps=3000000, reset_num_timesteps=False, callback=callback)
        print("Saving to: " + log_dir + 'ACKTR_' + str(id+1))
        model.save(log_dir + 'model_ACKTR_' + str(id+1))