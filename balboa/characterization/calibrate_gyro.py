from balboa.characterization.a_star import AStar
from balboa.balboa_env import BalboaState
import time

#python3 -m balboa.characterization.calibrate_gyro

comms = AStar()

bs = BalboaState(comms)

while(1):
    bs.read_balboa_sensor()
    v = bs.state_vector()
    print(v[7], v[8], v[9])
    time.sleep(1)