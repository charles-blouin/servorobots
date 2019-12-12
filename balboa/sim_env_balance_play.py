import gym
from stable_baselines.common.vec_env import SubprocVecEnv
from stable_baselines import PPO2
import tensorflow as tf
import shutil
import os
import balboa.utils
# python -m balboa.sim_env_balance_play
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
    env = SubprocVecEnv([lambda: gym.make('Balboa-balance-ctrl-render-v1') for i in range(n_cpu)])
    obs = env.reset()
    # When using VecEnv, done is a vector
    done = [False for _ in range(env.num_envs)]

    while True:
        action, _states = model.predict(obs, deterministic=True)
        obs, rewards, dones, info = env.step(action)
        speed = (obs[0, 2] + obs[0, 3]) / 25
        f_speed = 1 / (speed ** 2 + 1)
        print(obs[0,2])
        # env.render()