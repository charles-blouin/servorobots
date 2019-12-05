import numpy as np
import matplotlib.pyplot as plt
import json
from balboa.sim_env_motor_test import sim_motor_test

import scipy.optimize as opt

# python -m balboa.characterization.analyze_env_motor

file_name = 'balboa/characterization/tests/real_env_motor_test_01.txt'

action_labels = ["Motor 0", "Motor 1"]
states_labels = ["Rot left", "Rot right", "Vel left", "Vel right",
                        "gyro_x", "gyro_y", "gyro_z", "acc_x", "acc_y", "acc_z",
                        "voltage"]

with open(file_name, 'r') as filehandle:
    tr = json.load(filehandle)
    timestamps = np.array(tr["timestamps"])
    actions = np.array(tr["actions"])
    states = np.array(tr["states"])

#### Plotting
#### N lines = max(n of states, n of actions)
#### Two columns, one for actions, one for states

timestamp_sim, actions_sim, states_sim, looptime_sim = sim_motor_test()
timestamp_sim = np.asarray(timestamp_sim)
actions_sim = np.asarray(actions_sim)
states_sim = np.asarray(states_sim)
looptime_sim = np.asarray(looptime_sim)

lines = max(actions.shape[1], states.shape[1])
fig, axs = plt.subplots(lines, 2)
for line in range(actions.shape[1]):
    axs[line][0].plot(timestamps, actions[:, line], label="Real")
    axs[line][0].plot(timestamp_sim, actions_sim[:, line], label="Sim")
    axs[line][0].set_xlabel('Time')
    axs[line][0].set_ylabel(action_labels[line])
    axs[line][0].grid(True)
    axs[line][0].legend(loc='upper left')

for line in range(states.shape[1]):
    axs[line][1].plot(timestamps, states[:, line], label="Real")
    axs[line][1].plot(timestamp_sim, states_sim[:, line], label="Sim")
    axs[line][1].set_xlabel('Time')
    axs[line][1].set_ylabel(states_labels[line])
    axs[line][1].grid(True)
    axs[line][1].legend(loc='upper left')



# Works on windows. Todo: make maximizing window work on other systems.
# https://stackoverflow.com/questions/12439588/how-to-maximize-a-plt-show-window-using-python
figManager = plt.get_current_fig_manager()
figManager.window.state('zoomed')
vel_measured = states[175, 2]
vel_calc = (states[200, 0] - states[160, 0]) / (timestamps[200]-timestamps[160])
print(vel_measured)
print(vel_calc)

vel_measured = states_sim[175, 2]
vel_calc = (states_sim[200, 0] - states_sim[160, 0]) / (timestamp_sim[200]-timestamp_sim[160])
print(vel_measured)
print(vel_calc)
plt.show()