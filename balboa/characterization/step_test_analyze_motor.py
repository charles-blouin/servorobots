import numpy as np
import matplotlib.pyplot as plt
import json
import servorobots.components.test_motor as motor

# This file opens a step_test output file and finds the motor constants that fit the motors best.
import scipy.optimize as opt

# should be run from the top servorobots folder with
# python -m balboa.characterization.step_test_analyze
import time

file_name = 'balboa/characterization/step_test.txt'
gear_ratio = 1322
fps = 50
frame_length = 1/fps

    
with open(file_name, 'r') as filehandle:
    tr = json.load(filehandle)
    encoders = np.array(tr["encoders"])
    timestamps = np.array(tr["timestamps"])
    velocities = np.array(tr["velocities"])
    motors = np.array(tr["motors"])
    voltages = np.array(tr["voltages"])

sim_pos = []
sim_vel = []

# For simulation
m = motor.CharacterizeMotor(frame_length=frame_length)

def to_opt(arg, motor_id):
    R = arg[0]
    Kv = arg[1]

    y = 0
    m.reset(R=R, Kv=Kv)


    timestamps_iter = np.nditer(timestamps, ['c_index'])
    for timestamp in timestamps_iter:
        pos, vel = m.step([voltages[timestamps_iter.index], motors[timestamps_iter.index, motor_id]])
        y = y + (pos/2/3.1415 - encoders[timestamps_iter.index, motor_id]) ** 2

    return y
## For motor 0
# Optimizing R and Kv.
bnds = ((20, 60), (7, 9))

args = (0) # Motor 0
motor_0 = opt.minimize(to_opt, [22, 8], bounds=bnds, args=args)
print("Motor 0 R and Kv:")
print(motor_0.x)

## For motor 1
# Optimizing R and Kv.
args = (1) # Motor 1
motor_1 = opt.minimize(to_opt, [22, 8], bounds=bnds, args=args)
print("Motor 1 R and Kv:")
print(motor_1.x)


# Simulate motor 0 and 1 with constants found
def sim_motor(motor_id, motor_constants, frame_length=frame_length):
    m = motor.CharacterizeMotor(frame_length=frame_length)
    sim_pos = []
    sim_vel = []
    m.reset(R=motor_constants.x[0], Kv=motor_constants.x[1])
    timestamps_iter = np.nditer(timestamps, ['c_index'])
    for timestamp in timestamps_iter:
        pos, vel = m.step([voltages[timestamps_iter.index], motors[timestamps_iter.index, motor_id]])
        sim_pos.append(pos/2/3.1415)
        sim_vel.append(vel / 2 / 3.1415)
    return (sim_pos, sim_vel)


(sim_pos_0, sim_vel_0) = sim_motor(0, motor_0)
(sim_pos_1, sim_vel_1) = sim_motor(1, motor_1)

# Plot simulated and real results
fig, axs = plt.subplots(2, 1)
axs[0].plot(timestamps, encoders[:, 0], "-", label="Real 0")
axs[0].plot(timestamps, encoders[:, 1], label="Real 1")
axs[0].plot(timestamps, sim_pos_0, label="Simulated 0")
axs[0].plot(timestamps, sim_pos_1, label="Simulated 1")
axs[0].set_xlabel('Time')
axs[0].set_ylabel('Encoders')
axs[0].grid(True)
axs[0].legend(loc='upper left')
axs[1].plot(timestamps, velocities[:, 0], label="Real 0")
axs[1].plot(timestamps, velocities[:, 1], label="Real 1")
axs[1].plot(timestamps, sim_vel_0, label="Simulated 0")
axs[1].plot(timestamps, sim_vel_1, label="Simulated 1")
axs[1].legend(loc='upper left')
fig.tight_layout()
plt.show()