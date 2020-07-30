import gym
from stable_baselines.common.vec_env import SubprocVecEnv
from stable_baselines import PPO2
from stable_baselines import ACKTR
import tensorflow as tf
import shutil
import os
import balboa.utils
import time
# python -m pendulum_a.sim_play -a ppo -i 3
import argparse
ap = argparse.ArgumentParser(description='Play learning')
ap.add_argument("-i", "--load_id", default=None, help="Start from test id")
ap.add_argument("-a", "--algo", default='ppo', help="Start from test id")
args = ap.parse_args()

log_dir = "pendulum_a/results/"
if args.algo == 'ppo':
    result_string = 'PPO2_'
elif args.algo == 'acktr':
    result_string = 'ACKTR_'

if args.load_id == None:
    id = str(balboa.utils.tensorboard_latest_directory_number(log_dir))
else:
    id = str(args.load_id)
file = "pendulum_a/results/model_" + result_string + id + '/best_model.zip'

def generate_checkpoint_from_model(model, checkpoint_name):
    with model.graph.as_default():
        # if os.path.exists(checkpoint_name):
        #     shutil.rmtree(checkpoint_name)

        tf.saved_model.simple_save(model.sess, checkpoint_name, inputs={"obs": model.act_model.obs_ph},
                                   outputs={"action": model.act_model._deterministic_action})

if __name__ == '__main__':
    if os.path.isdir(file):
        shutil.rmtree(file)
    if args.algo == 'ppo':
        model = PPO2.load(file)
    elif args.algo == 'acktr':
        model = ACKTR.load(file)


    # generate_checkpoint_from_model(model, file)
    # converter = tf.lite.TFLiteConverter.from_saved_model(file)
    # tflite_model = converter.convert()
    # open(file + "/converted_model.tflite", "wb").write(tflite_model)

    # multiprocess environment
    n_cpu = 1
    env = SubprocVecEnv([lambda: gym.make('PendulumA-v0', renders=True) for i in range(n_cpu)])
    obs = env.reset()
    # When using VecEnv, done is a vector
    done = [False for _ in range(env.num_envs)]

    while True:
        action, _states = model.predict(obs, deterministic=False)
        obs, rewards, dones, info = env.step(action)
        rewards[0] += rewards[0]
        if dones[0] == 1:
            # print("Yaw speed: {:.2f}, Speed: {:.2f}".format(obs[0,-1], obs[0,-2]))
            # print(rewards[0])
            rewards[0] = 0
        # env.render()