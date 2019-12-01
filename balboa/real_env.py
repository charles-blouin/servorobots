import gym
import time
from balboa.a_star import AStar
import numpy as np
from balboa.BalboaRPiSlaveDemo.lsm6 import LSM6

"""
Instructions
Gyro calibration
The /balboa/__init__.py should be customized with the gyro calibration coefficients
To obtain them, run the calibrate_gyro script: python3 -m balboa.characterization.calibrate_gyro


"""

class BalboaState:
    def __init__(self, comms, offset_gx, offset_gy, offset_gz):
        self.lsm = LSM6()
        self.lsm.enable()
        self.gear_ratio = 1322

        self.comms = comms
        self.offset_gx = offset_gx
        self.offset_gy = offset_gy
        self.offset_gz = offset_gz
        
        self.comms.reset_encoders()
        self.previous_rot_left = 0
        self.previous_rot_right = 0
        self.previous_timestamp_left = 0
        self.previous_timestamp_right = 0
        
        self.read_balboa_sensor()

    def read_balboa_sensor(self):
        # Encoder calculation
        self.timestamp_left, self.timestamp_right, \
        self.rot_left, self.rot_right \
        = self.comms.read_sensors()
    
        self.rot_left = self.rot_left/self.gear_ratio * 6.29184 # in rad
        self.rot_right = self.rot_right/self.gear_ratio * 6.29184 # in rad


        # Velocity calculation
        if (self.timestamp_left == self.previous_timestamp_left):
            self.vel_left = 0
        else:
            self.vel_left = (self.rot_left - self.previous_rot_left) / \
                            (self.timestamp_left - self.previous_timestamp_left) * 1000000
            
        if (self.timestamp_right == self.previous_timestamp_right):
            self.vel_right = 0
        else:
            self.vel_right = (self.rot_right - self.previous_rot_right) / \
                             (self.timestamp_right - self.previous_timestamp_right) * 1000000
        
        self.previous_rot_left = self.rot_left
        self.previous_rot_right = self.rot_right
        self.previous_timestamp_left = self.timestamp_left
        self.previous_timestamp_right = self.timestamp_right
        
        # Voltage Calculation
        self.voltage = self.comms.read_battery_millivolts()[0]/1000
        
        ## Gyro and accelerometer
        self.lsm.read()
    
    def calibrate_gyro(self):
        for i in range(1000):
            self.lsm.read()
            gyro_x_cal += self.lsm.g.x
            gyro_y_cal += self.lsm.g.y
            gyro_z_cal += self.lsm.g.z
            
        print("Gyro cal:" )
        print(gyro_x_cal/1000)
        print(gyro_y_cal/1000)
        print(gyro_z_cal/1000)
        
        
    def state_vector(self):
        gyro_x = self.lsm.gx - self.offset_gx
        gyro_y = self.lsm.gy - self.offset_gy
        gyro_z = self.lsm.gz - self.offset_gz
        acc_x = self.lsm.ax
        acc_y = self.lsm.ay
        acc_z = self.lsm.az
        
        return np.array([self.rot_left, self.rot_right, self.vel_left, self.vel_right,
                        gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z,
                        self.voltage])

# Forward is battery cover when the balboa is up.
# Left motor is 0. Positive direction makes the balboa go forward.
# Right motor is 1

# gx is 9.8 when robot is balancing
# gy is 9.8 when robot is on its right when
# gz is 9.8 when battery cover is down.

class BalboaEnvMotor(gym.Env):
    def __init__(self, renders=False, offset_gx=0, offset_gy=0, offset_gz=0):
        self._seed()
        
        # Balancer observation: All in SI
        # Motor Rot Left, Rot Right (2)
        # Motor Speed Left, Right (2)
        # Angular velocity (3), Local frame
        # Acceleration (3), Local frame for later
        # Voltage (1)
        
        observation_dim = 10
        high_obs = np.ones([observation_dim])
        self.observation_space = gym.spaces.Box(high_obs*0, high_obs*1.5)
        
        act_high = np.asarray([1, 1])
        self.action_space = gym.spaces.Box(-act_high, act_high)
        
        
        # Comm object to access Balboa
        self.comms = AStar()
        self.balboa_state = BalboaState(self.comms, offset_gx, offset_gy, offset_gz)
        self.max_PWM_value = 400
        
        


    def step(self, action):
        np.clip(action, -1, 1)
        # Step the environment
        left = round(action[0]*self.max_PWM_value)
        right = round(action[1]*self.max_PWM_value)
        self.comms.motors(left, right)
        
        self.balboa_state.read_balboa_sensor()
        
        reward = 0
        done = 0
        current_time = time.time() - self.start_time
        return self.balboa_state.state_vector(), reward, done, {"time": current_time}
        
    def reset(self):
        self.start_time = time.time()
        self.comms.reset_encoders()
        self.balboa_state.read_balboa_sensor()
        return self.balboa_state.state_vector()
        
    def _seed(self, seed=None):
        return [0]
        