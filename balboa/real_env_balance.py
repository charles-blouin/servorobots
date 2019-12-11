import gym
import time
import json
import os
import tflite_runtime.interpreter as tflite

# python3 -m balboa.real_env_balance

env = gym.make('Balboa-v0')

# Relative to the directory of this script
file_name = 'results/real/01.txt'

actions_array = []
states_array = []
timestamps = []
looptime = []

fps = 100
frame_length = 1 / fps
episode_length = 4  # in seconds

states = env.reset()
start_time = time.time()

#### TF lite
interpreter = tflite.Interpreter(model_path="results/real/converted_model.tflite")
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

for step in range(0, fps * episode_length):
    time_of_this_step = time.time()

    # TF lite
    interpreter.set_tensor(input_details[0]['index'], states)
    interpreter.invoke()
    actions = interpreter.get_tensor(output_details[0]['index'])


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