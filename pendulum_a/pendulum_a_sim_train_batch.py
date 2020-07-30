# python -m pendulum_a.pendulum_a_sim_train_batch
import gym
import tensorflow as tf
import balboa.utils
import stable_baselines
import argparse
from stable_baselines.common.callbacks import BaseCallback, EvalCallback, CheckpointCallback
import os




ap = argparse.ArgumentParser(description='Learn or continue learning')
ap.add_argument("-i", "--load_id", default=None, help="Start from test id")
ap.add_argument("-a", "--algo", default="ppo2", help="Algorithm")
args = ap.parse_args()

log_dir = "pendulum_a/results/"
# New id

n_steps = 0
class TBCallback(BaseCallback):
    """
    Custom callback for plotting additional values in tensorboard.
    """
    def __init__(self, verbose=0):
        self.is_tb_set = False
        super(TBCallback, self).__init__(verbose)

    def _on_training_start(self) -> bool:
        # Log additional tensor
        # Log scalar value (here a random variable)

        batch_size = self.locals['self'].n_batch // self.locals['self'].nminibatches
        lr_start = self.locals['self'].learning_rate(1)
        lr_end = self.locals['self'].learning_rate(0)
        noptepochs = self.locals['self'].noptepochs
        clip_range_start = self.locals['self'].cliprange(1)
        nminibatches = self.locals['self'].nminibatches

        summary = tf.Summary(value=[tf.Summary.Value(tag='Hparams/batch_size', simple_value=batch_size,
                                                     metadata=tf.SummaryMetadata(display_name='batch_size', summary_description='n_cpu*n_step/n_minibatches')),
                                    tf.Summary.Value(tag='Hparams/lr_start', simple_value=lr_start,
                                                     metadata=tf.SummaryMetadata(display_name='lr_start')),
                                    tf.Summary.Value(tag='Hparams/lr_end', simple_value=lr_end,
                                                     metadata=tf.SummaryMetadata(display_name='lr_end')),
                                    tf.Summary.Value(tag='Hparams/noptepochs', simple_value=noptepochs,
                                                     metadata=tf.SummaryMetadata(display_name='noptepochs')),
                                    tf.Summary.Value(tag='Hparams/clip_range_start', simple_value=clip_range_start,
                                                     metadata=tf.SummaryMetadata(display_name='clip_range_start')),
                                    tf.Summary.Value(tag='Hparams/nminibatches', simple_value=nminibatches,
                                                     metadata=tf.SummaryMetadata(display_name='nminibatches'))
                                    ])

        # self.run_dir = self.locals['writer'].get_logdir()
        return True



if __name__ == '__main__':

    from stable_baselines.common.vec_env import SubprocVecEnv
    from stable_baselines import PPO2
    from stable_baselines import A2C
    from stable_baselines import ACKTR

    n_cpu = 12

    env = SubprocVecEnv([lambda: gym.make('PendulumA-v0') for i in range(n_cpu)])

    env_eval = gym.make('PendulumA-eval-v0')

    log_hparam_callback = TBCallback()


    # Define policy
    policy_kwargs = dict(act_fun=tf.nn.tanh, net_arch=[32, 32])
    print("Tensorflow ########################################")
    sess = tf.Session(config=tf.ConfigProto(log_device_placement=True))
    # Train
    if args.algo == "ppo2":
        id = balboa.utils.tensorboard_latest_directory_number(log_dir, 'PPO2_')
        print(log_dir + 'PPO2_' + str(id+1))
        eval_callback = EvalCallback(env_eval, best_model_save_path=log_dir + '/model_PPO2_' + str(id+1),
                                     log_path=log_dir + '/model_PPO2_' + str(id+1), eval_freq=1000, n_eval_episodes=5,
                                     deterministic=True, render=False)
        if args.load_id == None:
            # tensorboard_log=log_dir
            model = PPO2("MlpPolicy", env, policy_kwargs=policy_kwargs, verbose=1, nminibatches=16,
                         n_steps=192, lam=0.94, gamma=0.98, noptepochs=5,
                         ent_coef=0.0, cliprange=0.2)
        else:
            print("Loading model: " + str(args.load_id))
            model = PPO2.load(log_dir + 'model_PPO2_' + str(args.load_id) + "/final.zip", env=env)
        model.tensorboard_log = log_dir
        model.learning_rate = stable_baselines.common.schedules.LinearSchedule(1.0, 0.00025, initial_p=0.00001).value
        # model.cliprange = stable_baselines.common.schedules.LinearSchedule(1.0, 0.2, initial_p=0).value
        model.learn(total_timesteps=1000000, reset_num_timesteps=False, callback=[log_hparam_callback, eval_callback])
        model.save(log_dir + '/model_PPO2_' + str(id+1) + '/final')


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