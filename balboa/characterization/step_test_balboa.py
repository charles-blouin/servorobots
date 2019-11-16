import gym
import time
import json

env = gym.make('Balboa-v0')
env.reset()

file_name = 'step_test_01.txt'

actions_array = []
states_array = []
timestamps = []

fps = 50
frame_length = 1/fps
episode_length = 3 # in seconds

start_time = time.time()

for step in range(0,fps*episode_length):
    
    timestamps.append(time.time() - start_time)
    
    
    
    
    if step < 1*fps:
        actions = [0, 0]
    elif step < 2*fps:
        actions = [1, -1]
    elif step < 3*fps:
        actions = [0, 0]
        
    states, reward, _, _ = env.step(actions)
    
    states_array.append(states.tolist())
    actions_array.append(actions)
    
    
    delay = start_time + (step+1) * frame_length - time.time()
    time.sleep(delay)
    
    
with open(file_name, 'w') as filehandle:
    json.dump({"timestamps": timestamps, "actions": actions_array, 
               "states": states_array
        }, filehandle)