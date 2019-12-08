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

def to_opt(arg):
    ML_R = arg[0]
    MR_R = arg[0]

    ML_Kv = arg[1]
    MR_Kv = arg[1]

    ML_Kvis = arg[2]
    MR_Kvis = arg[2]
    timestamp_sim, actions_sim, states_sim, looptime_sim = sim_motor_test(render=False,
                                                                          ML_R=ML_R, MR_R=MR_R,
                                                                          ML_Kv=ML_Kv, MR_Kv=MR_Kv,
                                                                          ML_Kvis=ML_Kvis, MR_Kvis=MR_Kvis)
    """
    timestamp_sim, actions_sim, states_sim, looptime_sim = sim_motor_test(render=False,
                                                                          ML_R=ML_R, MR_R=MR_R,
                                                                          ML_Kv=ML_Kv, MR_Kv=MR_Kv)
    """
    states_sim = np.asarray(states_sim)
    # Left motor error
    error = np.sum(np.square(states[:, 0] - states_sim[:, 0]))
    # Right motor error
    error += np.sum(np.square(states[:, 1] - states_sim[:, 1]))

    return error

# Optimizing
# R, Kv, Kvis
bnds = ((15, 25), (8, 10))
guess = [21.52, 10.5, 0.0005]
opt_results = opt.minimize(to_opt, guess, options={"maxiter": 100})
print(opt_results.success)
print(opt_results.message)
print(opt_results.nit)
results = np.asarray(opt_results.x)

np.set_printoptions(precision=3)
print("R, Kv, and Kvis:")
print(results[0])
print(results[1])
print(results[2])
R = results[0]
Kv = results[1]
Kvis = results[2]

timestamp_sim, actions_sim, states_sim, looptime_sim = sim_motor_test(render=False,
                                                                          ML_R=R, MR_R=R,
                                                                          ML_Kv=Kv, MR_Kv=Kv,
                                                                          ML_Kvis=Kvis, MR_Kvis=Kvis)
states_sim = np.asarray(states_sim)
timestamp_sim = np.asarray(timestamp_sim)
actions_sim = np.asarray(actions_sim)
looptime_sim = np.asarray(looptime_sim)



#### Plotting
#### N lines = max(n of states, n of actions)
#### Two columns, one for actions, one for states
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

vel_calc = (states[200, 0] - states[160, 0]) / (timestamps[200]-timestamps[160])

vel_calc = (states_sim[200, 0] - states_sim[160, 0]) / (timestamp_sim[200]-timestamp_sim[160])
plt.show()