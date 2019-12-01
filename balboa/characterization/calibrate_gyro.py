<<<<<<< HEAD
from balboa.a_star import AStar
from balboa.balboa_env import BalboaState
=======
from balboa.characterization.a_star import AStar
from balboa.real_env import BalboaState
>>>>>>> 5fa2b2190c87f962c89d7c0e6f9b32f188c19d12
import time

#python3 -m balboa.characterization.calibrate_gyro

comms = AStar()

bs = BalboaState(comms)

#### For calibration ####

length = 500

offset_gx = 0
offset_gy = 0
offset_gz = 0

for i in range(length):
    bs.read_balboa_sensor()
    v = bs.state_vector()
    offset_gx += v[4]
    offset_gy += v[5]
    offset_gz += v[6]

print("The gx, gy, gz coeffients are: ")
print(offset_gx/length, offset_gy/length, offset_gz/length)

#### For visualisation ####
while(1):
    bs.read_balboa_sensor()
    v = bs.state_vector()
    #print('gx: {:5.2f}, gy: {:5.2f}, az: {:5.2f}'.format(v[4], v[5], v[6]), end="\r\r")
    print('gx: {:5.2f}, gy: {:5.2f}, az: {:5.2f}'.format(v[4], v[5], v[6]) +
            ' ax: {:5.2f}, ay: {:5.2f}, az: {:5.2f}'.format(v[7], v[8], v[9]), end="\r")
    # print("Accel", f"{v[7]:2.2}", f"{v[8]:2.2}", f"{v[9]:2.2}")
    time.sleep(0.2)