from a_star import AStar
import json
a_star = AStar()
import time

file_name = 'step_test.txt'
gear_ratio = 1322
fps = 100
frame_length = 1/fps
episode_length = 3 # in seconds
encoders = []
motors = []
timestamp = []
velocities = []

start_time = time.time()

previous_encoder = a_star.read_encoders()

for step in range(0,fps*episode_length):
    timestamp.append(time.time() - start_time)
    
    current_encoder = a_star.read_encoders()
    encoders.append((current_encoder[0]/gear_ratio, current_encoder[1]/gear_ratio))
    
    velocities.append((current_encoder[0] - previous_encoder[0]/frame_length, current_encoder[1] - previous_encoder[1]/frame_length))
    current_encoder = previous_encoder
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
    json.dump({"timestamp": timestamp, "encoders": encoders, "motors": motors, "velocities": velocities}, filehandle)
    
with open(file_name, 'r') as filehandle:
    test_result = json.load(filehandle)