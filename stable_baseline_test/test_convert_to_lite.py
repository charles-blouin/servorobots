import tensorflow as tf
import argparse
import gym
from stable_baselines import PPO2
from stable_baselines.common.vec_env import DummyVecEnv

import numpy as np
import shutil

if __name__ == '__main__':
    ap = argparse.ArgumentParser(description='Play an evaluate the model')
    ap.add_argument("-i", "--in", help="path of input")
    ap.add_argument("-o", "--out", help="path of output")
    args = ap.parse_args()

    env = gym.make('Pendulum-v2')
    env = DummyVecEnv([lambda: env])
    model = PPO2.load("results/pendulum/02")

    obs = env.reset()


    def generate_checkpoint_from_model(model, checkpoint_name):
        with model.graph.as_default():
            # if os.path.exists(checkpoint_name):
            #     shutil.rmtree(checkpoint_name)

            tf.saved_model.simple_save(model.sess, checkpoint_name, inputs={"obs": model.act_model.obs_ph},
                                       outputs={"action": model.act_model._deterministic_action})


    obs_used = np.array([[-0.5662416,  0.8242393,  0.6782238]], dtype=np.float32)

    shutil.rmtree('results/test/full_model')
    generate_checkpoint_from_model(model, "results/test/full_model")
    converter = tf.lite.TFLiteConverter.from_saved_model("results/test/full_model")
    tflite_model = converter.convert()

    #### Interpreter ####
    interpreter = tf.lite.Interpreter(model_content=tflite_model)
    interpreter.allocate_tensors()

    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    print(input_details[0]['index'])
    interpreter.set_tensor(input_details[0]['index'], obs_used)

    interpreter.invoke()
    tflite_results = interpreter.get_tensor(output_details[0]['index'])

    print("Input details: ")
    print(input_details)
    print("Output details: ")
    print(output_details)

    print("A sample observations is: ")
    print(obs)
    print("This observation will be used:")
    print()
    print("The actions of the original model are: ")
    action, _states = model.predict(obs_used, deterministic=True)
    print(action)
    print("If the actions were not deterministic, you might obtain:")
    action, _states = model.predict(obs_used)
    print(action)
    action, _states = model.predict(obs_used)
    print(action)

    print("The actions of the tflite model are: ")
    print(tflite_results)

    # model = PPO2.load(args.path)
    # tf.saved_model.simple_save(model.sess, checkpoint_name, inputs={"obs": model.act_model.obs_ph},
    #                                    outputs={"action": model.act_model._policy_proba})

