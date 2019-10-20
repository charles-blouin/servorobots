import numpy as np
import matplotlib.pyplot as plt
import json

import time

file_name = 'step_test.txt'
gear_ratio = 1322
fps = 100
frame_length = 1/fps
episode_length = 3 # in seconds

    
with open(file_name, 'r') as filehandle:
    tr = json.load(filehandle)
    
    
print(tr["encoders"][:],[0])

fig, axs = plt.subplots(2, 1)

axs[0].plot(tr["timestamps"], tr["encoders"][:],[0], tr["timestamps"], tr["encoders"][:],[1])