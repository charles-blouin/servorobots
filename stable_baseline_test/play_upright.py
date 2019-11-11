import gym
import argparse
import os
# For tensorflow, INFO and WARNING messages are not printed
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

import time



from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.vec_env import SubprocVecEnv
from stable_baselines import PPO2
if __name__ == '__main__':
    ap = argparse.ArgumentParser(description='Play an evaluate the model')
    ap.add_argument("--eval", default=0, help="Evaluate with random and real agent")
    ap.add_argument("-p", "--path", help="path of the tested model")
    ap.add_argument("-e", "--env", help="Save path")
    args = ap.parse_args()

    # python -m stable_baseline_test.play_upright -p "results/pendulum/01" -e "Pendulum-v1"
    # python -m stable_baseline_test.play_upright -p "results/pendulum/01" -e "Pendulum-v2" --eval 1

    if args.eval == 0:
        model = PPO2.load(args.path)
        # multiprocess environment
        n_cpu = 1
        env = SubprocVecEnv([lambda: gym.make(args.env) for i in range(n_cpu)])
        obs = env.reset()
        # When using VecEnv, done is a vector
        done = [False for _ in range(env.num_envs)]
        while True:
            action, _states = model.predict(obs)
            obs, rewards, dones, info = env.step(action)
            env.render()
    else:
        model = PPO2.load("results/pendulum/random")
        n_cpu = 8
        env = SubprocVecEnv([lambda: gym.make(args.env) for i in range(n_cpu)])
        obs = env.reset()
        num_test = 0
        num_tests = 128
        cumul = 0
        # Passing state=None to the predict function means
        # it is the initial state
        state = None
        # When using VecEnv, done is a vector
        done = [False for _ in range(env.num_envs)]

        # For a random agent
        #step = 0
        ## The test will be evaluated for
        #for step in range(5000):
        #    action, _states = model.predict(obs)
        #    obs, rewards, dones, info = env.step(action)
        #    cumul = sum(rewards) + cumul
        # print("A random agent has an average reward of: " + str(cumul/5000/8) + " per step")

        model = PPO2.load(args.path)
        step = 0
        cumul = 0
        for step in range(5000):
            action, _states = model.predict(obs)
            obs, rewards, dones, info = env.step(action)
            cumul = sum(rewards) + cumul
        print("The tested agent has an average reward of: " + str(cumul/5000/8) + " per step")