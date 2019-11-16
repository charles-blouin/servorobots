from a_star import AStar
import json
a_star = AStar()
import time

file_name = 'step_test.txt'
gear_ratio = 1322
fps = 50
frame_length = 1/fps
episode_length = 3 # in seconds
encoders = []
motors = []
timestamps = []
velocities = []
voltages = []

start_time = time.time()


start_encoder = a_star.read_encoders()
start_rot = [start_encoder[0]/ gear_ratio, start_encoder[1]/ gear_ratio]
previous_rot = [0, 0]

for step in range(0,fps*episode_length):
    timestamps.append(time.time() - start_time)
    
    current_encoder = a_star.read_encoders()
    current_rot = [current_encoder[0]/ gear_ratio - start_rot[0], current_encoder[1]/ gear_ratio - start_rot[1]]
    encoders.append(current_rot)
    
    velocities.append(((current_rot[0] - previous_rot[0])/frame_length, (current_rot[1] - previous_rot[1])/frame_length))
    previous_rot = current_rot
    
    voltages.append(a_star.read_battery_millivolts()[0]/1000)
    if step < 1*fps:
        a_star.motors(0,0)
        motors.append((0,0))
    elif step < 2*fps:
        a_star.motors(400,400)
        motors.append((400,400))
    elif step < 3*fps:
        a_star.motors(0,0)
        motors.append((0,0))
    
    
    delay = start_time + (step+1) * frame_length - time.time()
    time.sleep(delay)
    # a_star.motors(0, 0)
    # a_star.motors(400, 0)
    # time.sleep(1)
    # a_star.motors(0, 0)

with open(file_name, 'w') as filehandle:
    json.dump({"timestamps": timestamps, "encoders": encoders, "motors": motors, "velocities": velocities, 
            "voltages": voltages
        }, filehandle)
    
with open(file_name, 'r') as filehandle:
    test_result = json.load(filehandle)