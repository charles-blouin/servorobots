import numpy as np
import matplotlib.pyplot as plt
import json
# python -m balboa.characterization.step_test_analyze

# This file opens a step_test output file and finds the motor constants that fit the motors best using Pybullet.
import scipy.optimize as opt

file_name = 'balboa/characterization/tests/step_test_01.txt'
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


lines = max(actions.shape[1], states.shape[1])
fig, axs = plt.subplots(lines, 2)
for line in range(actions.shape[1]):
    axs[line][0].plot(timestamps, actions[:, line], label="Real")
    axs[line][0].set_xlabel('Time')
    axs[line][0].set_ylabel(action_labels[line])
    axs[line][0].grid(True)
    axs[line][0].legend(loc='upper left')

for line in range(states.shape[1]):
    axs[line][1].plot(timestamps, states[:, line], label="Real")
    axs[line][1].set_xlabel('Time')
    axs[line][1].set_ylabel(states_labels[line])
    axs[line][1].grid(True)
    axs[line][1].legend(loc='upper left')



# Works on windows. Todo: make maximizing window work on other systems.
# https://stackoverflow.com/questions/12439588/how-to-maximize-a-plt-show-window-using-python
figManager = plt.get_current_fig_manager()
figManager.window.state('zoomed')
plt.show()