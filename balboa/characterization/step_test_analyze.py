import numpy as np
import matplotlib.pyplot as plt
import json
# python -m balboa.characterization.step_test_analyze

# This file opens a step_test output file and finds the motor constants that fit the motors best using Pybullet.
import scipy.optimize as opt

file_name = 'balboa/characterization/tests/step_test_01.txt'

with open(file_name, 'r') as filehandle:
    tr = json.load(filehandle)
    timestamps = np.array(tr["timestamps"])
    actions = np.array(tr["actions"])
    states = np.array(tr["states"])

print(timestamps)


#### Plotting
#### N lines = max(n of states, n of actions)
#### Two columns, one for actions, one for states

fig, axs = plt.subplots(2, 1)


fig.tight_layout()
plt.show()