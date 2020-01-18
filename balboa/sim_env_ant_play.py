import gym
from stable_baselines.common.vec_env import SubprocVecEnv
from stable_baselines import PPO2
import tensorflow as tf
import shutil
import os
import balboa.utils
import pybulletgym
import time
# python -m balboa.sim_env_ant_play
import argparse
ap = argparse.ArgumentParser(description='Play learning')
ap.add_argument("-i", "--load_id", default=None, help="Start from test id")
args = ap.parse_args()

log_dir = "balboa/results/"

if args.load_id == None:
    id = str(balboa.utils.tensorboard_latest_directory_number(log_dir))
else:
    id = str(args.load_id)
file = "balboa/results/" + id

def generate_checkpoint_from_model(model, checkpoint_name):
    with model.graph.as_default():
        # if os.path.exists(checkpoint_name):
        #     shutil.rmtree(checkpoint_name)

        tf.saved_model.simple_save(model.sess, checkpoint_name, inputs={"obs": model.act_model.obs_ph},
                                   outputs={"action": model.act_model._deterministic_action})

if __name__ == '__main__':
    if os.path.isdir(file):
        shutil.rmtree(file)
    model = PPO2.load(file)

    generate_checkpoint_from_model(model, file)
    converter = tf.lite.TFLiteConverter.from_saved_model(file)
    tflite_model = converter.convert()
    open(file + "/converted_model.tflite", "wb").write(tflite_model)

    # multiprocess environment
    n_cpu = 1
    # 'Balboa-balance-ctrl-render-v1'
    env = gym.make('AntPyBulletEnv-v0')
    env.render(mode="human")
    obs = env.reset()
    # When using VecEnv, done is a vector

    while True:
        action, _states = model.predict(obs, deterministic=False)
        obs, reward, done, info = env.step(action)
        reward += reward
        time.sleep(0.02)
        if done == 1:
            reward = 0
            obs = env.reset()
        # env.render()