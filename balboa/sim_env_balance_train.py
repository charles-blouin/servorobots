# python -m balboa.sim_env_balance_train
import gym
import tensorflow as tf

import stable_baselines

log_dir = "balboa/results/"
id = "22"
n_steps = 0
def callback(_locals, _globals):
    """
    Callback called at each step (for DQN an others) or after n steps (see ACER or PPO2)
    :param _locals: (dict)
    :param _globals: (dict)
    """
    global n_steps, best_mean_reward
    # Print stats every 1000 calls
    if (n_steps + 1) % 10 == 0:
        print("########## Saving ##########")
        _locals['self'].save(log_dir + 'best_model_' + id + '.pkl')
        n_steps += 1
    return True

if __name__ == '__main__':

    from stable_baselines.common.vec_env import SubprocVecEnv
    from stable_baselines import PPO2

    n_cpu = 8
    env = SubprocVecEnv([lambda: gym.make('Balboa-balance-v1') for i in range(n_cpu)])

    # Define policy
    policy_kwargs = dict(act_fun=tf.nn.relu, net_arch=[32, 32])


    # Train
    model = PPO2("MlpPolicy", env, policy_kwargs=policy_kwargs, verbose=1, tensorboard_log=log_dir, nminibatches=8,
                 n_steps=256, lam=0.8, gamma=0.98, noptepochs=10,
                 ent_coef=0.0)
    model.learning_rate = stable_baselines.common.schedules.LinearSchedule(1.0, 0.001, initial_p=0.0005).value
    model.cliprange = stable_baselines.common.schedules.LinearSchedule(1.0, 0.2, initial_p=0.1).value

    model.learn(total_timesteps=1000000, reset_num_timesteps=False, callback=callback)
    model.save("balboa/results/" + id)