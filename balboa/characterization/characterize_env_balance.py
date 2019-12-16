import numpy as np
import matplotlib.pyplot as plt
import json
import gym
from balboa.sim_env_motor_test import sim_motor_test

import scipy.optimize as opt

# python -m balboa.characterization.characterize_env_balance

file_name = 'balboa/characterization/tests/01_real_env_balance.txt'

action_labels = ["Motor 0", "Motor 1", "state_1", "state_2"]
states_labels = ["Vel left", "Vel right",
                        "gyro_x", "gyro_y", "gyro_z", "acc_x", "acc_y", "acc_z",
                        "voltage"]

with open(file_name, 'r') as filehandle:
    tr = json.load(filehandle)
    timestamps = np.array(tr["timestamps"])
    actions = np.array(tr["actions"])
    states = np.array(tr["states"])

env = gym.make('Balboa-balance-ctrl-render-v1')
obs = env.reset()
states_sim = []

for i in range(400):
    obs, rewards, dones, info = env.step(actions[i])
    states_sim.append(obs)

"""

timestamp_sim, actions_sim, states_sim, looptime_sim = sim_motor_test(render=False,
                                                                          ML_R=R, MR_R=R,
                                                                          ML_Kv=Kv, MR_Kv=Kv,
                                                                          ML_Kvis=Kvis, MR_Kvis=Kvis)
states_sim = np.asarray(states_sim)
timestamp_sim = np.asarray(timestamp_sim)
actions_sim = np.asarray(actions_sim)
looptime_sim = np.asarray(looptime_sim)

"""
states_sim =  np.asarray(states_sim)

#### Plotting
#### N lines = max(n of states, n of actions)
#### Two columns, one for actions, one for states
lines = max(actions.shape[1], states.shape[1])
fig, axs = plt.subplots(9, 2)
for line in range(actions.shape[1]):
    axs[line][0].plot(timestamps, actions[:, line], label="Real")
    #axs[line][0].plot(timestamps, states_sim[:, line], label="Sim")
    axs[line][0].set_xlabel('Time')
    axs[line][0].set_ylabel(action_labels[line])
    axs[line][0].grid(True)
    axs[line][0].legend(loc='upper left')

for line in range(8):
    axs[line][1].plot(timestamps, states[:, line], label="Real")
    axs[line][1].plot(timestamps , states_sim[:, line], label="Sim")
    axs[line][1].set_xlabel('Time')
    axs[line][1].set_ylabel(states_labels[line])
    axs[line][1].grid(True)
    axs[line][1].legend(loc='upper left')



# Works on windows. Todo: make maximizing window work on other systems.
# https://stackoverflow.com/questions/12439588/how-to-maximize-a-plt-show-window-using-python
figManager = plt.get_current_fig_manager()
figManager.window.state('zoomed')

plt.show()