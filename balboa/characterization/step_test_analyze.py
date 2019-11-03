import numpy as np
import matplotlib.pyplot as plt
import json
import servorobots.components.test_motor as motor

import scipy.optimize as opt

# should be run from the top servorobots folder with
# python -m balboa.characterization.step_test_analyze
import time
print("hi!")
file_name = 'balboa/characterization/step_test.txt'
gear_ratio = 1322
fps = 50
frame_length = 1/fps
episode_length = 3 # in seconds

    
with open(file_name, 'r') as filehandle:
    tr = json.load(filehandle)
    encoders = np.array(tr["encoders"])
    timestamps = np.array(tr["timestamps"])
    velocities = np.array(tr["velocities"])
    motors = np.array(tr["motors"])
    voltages = np.array(tr["voltages"])
    
    
print(encoders[0:3, 0])

sim_pos = []
sim_vel = []


m = motor.CharacterizeMotor(frame_length=frame_length)

def to_opt(arg):
    R = arg[0]
    Kv = arg[1]

    y = 0
    m.reset(R=R, Kv=Kv)


    timestamps_iter = np.nditer(timestamps, ['c_index'])
    for timestamp in timestamps_iter:
        pos, vel = m.step([voltages[timestamps_iter.index], motors[timestamps_iter.index, 0]])
        y = y + (pos/2/3.1415 - encoders[timestamps_iter.index, 0]) ** 2

    return y
bnds = ((20, 60), (7, 9))
x = opt.minimize(to_opt, [22, 8], bounds=bnds)

print(x)
m.reset(R=40.2, Kv=9)
timestamps_iter = np.nditer(timestamps, ['c_index'])
for timestamp in timestamps_iter:
    pos, vel = m.step([voltages[timestamps_iter.index], motors[timestamps_iter.index, 0]])
    sim_pos.append(pos/2/3.1415)
    sim_vel.append(vel / 2 / 3.1415)


fig, axs = plt.subplots(2, 1)

axs[0].plot(timestamps, encoders[:, 0], timestamps, encoders[:, 1],
            timestamps, sim_pos)
axs[0].set_xlabel('Time')
axs[0].set_ylabel('Encoders')
axs[0].grid(True)
axs[1].plot(timestamps, velocities[:, 0], timestamps, velocities[:, 1], timestamps, sim_vel)
fig.tight_layout()
plt.show()