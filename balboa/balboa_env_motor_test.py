import gym
import time
import json

# python -m balboa.balboa_env_motor_test

env = gym.make('Balboa-motors-render-v1')
env.reset()

file_name = 'balboa_env_motor_test_01.txt'

actions_array = []
states_array = []
timestamps = []

fps = 50
frame_length = 1 / fps
episode_length = 3  # in seconds

start_time = time.time()

for step in range(0, fps * episode_length):



    if step < 1 * fps:
        actions = [0, 0]
    elif step < 2 * fps:
        actions = [0, 0]
    elif step < 3 * fps:
        actions = [0.1, 0.1]


    states, reward, _, info = env.step(actions)
    timestamps.append(info["time"])
    states_array.append(states.tolist())
    actions_array.append(actions)


with open(file_name, 'w') as filehandle:
    json.dump({"timestamps": timestamps, "actions": actions_array,
               "states": states_array
               }, filehandle)