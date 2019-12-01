import gym
import time
import json
import os

# python3 -m balboa.real_env_motor_test

env = gym.make('Balboa-v0')

# Relative to the directory of this script
file_name = 'characterization/tests/real_env_motor_test_01.txt'

actions_array = []
states_array = []
timestamps = []
looptime = []

fps = 100
frame_length = 1 / fps
episode_length = 4  # in seconds



states = env.reset()
start_time = time.time()

for step in range(0, fps * episode_length):
    time_of_this_step = time.time()
    # action = NN(states)

    if step < 1 * fps:
        actions = [0, 0]
    elif step < 1.5 * fps:
        actions = [0.5, 0.5]
    elif step < 2 * fps:
        actions = [1, 1]
    elif step < 2.5 * fps:
        actions = [0, 0]
    elif step < 3 * fps:
        actions = [-0.5, -0.5]
    elif step < 3.5 * fps:
        actions = [-1, -1]
    elif step < 4 * fps:
        actions = [0, 0]


    states, reward, _, info = env.step(actions)
    timestamps.append(time_of_this_step - start_time)
    states_array.append(states.tolist())
    actions_array.append(actions)
    looptime.append(time.time() - time_of_this_step)
    
    delay = start_time + (step+1) * frame_length - time.time()
    time.sleep(delay)


script_dir = os.path.dirname(__file__) #<-- absolute dir the script is in
abs_file_path = os.path.join(script_dir, file_name)
with open(abs_file_path, 'w') as filehandle:
    json.dump({"timestamps": timestamps, "actions": actions_array,
               "states": states_array, "looptime": looptime
               }, filehandle)