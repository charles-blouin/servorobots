import gym
import time

env = gym.make('Balboa-v0')
env.reset()

while True:
    states, reward, _, _ = env.step([0,0])
    
    print(states[2])
    time.sleep(0.05)