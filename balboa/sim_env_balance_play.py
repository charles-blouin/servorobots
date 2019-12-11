import gym
from stable_baselines.common.vec_env import SubprocVecEnv
from stable_baselines import PPO2
import tensorflow as tf
import shutil
import os
# python -m balboa.sim_env_balance_play
file = "balboa/results/23"

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
    env = SubprocVecEnv([lambda: gym.make('Balboa-balance-render-v1') for i in range(n_cpu)])
    obs = env.reset()
    # When using VecEnv, done is a vector
    done = [False for _ in range(env.num_envs)]

    while True:
        action, _states = model.predict(obs, deterministic=True)
        obs, rewards, dones, info = env.step(action)
        print(obs[0,4])
        # env.render()