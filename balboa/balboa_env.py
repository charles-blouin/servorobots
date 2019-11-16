import gym
import time
from balboa.characterization.a_star import AStar
import numpy as np


class BalboaState:
    def __init__(self, comms):
        self.gear_ratio = 1322
        
        self.comms = comms
        self.read_balboa_sensor()

    def read_balboa_sensor(self):
        self.rot_left, self.rot_right = self.comms.read_encoders()
        self.rot_left = self.rot_left/self.gear_ratio * 6.29184 # in rad
        self.rot_right = self.rot_right/self.gear_ratio * 6.29184 # in rad
        
        time_left, time_right = self.comms.read_timeSinceLastEncoder()
        self.vel_left = (1/self.gear_ratio*6.29184)/time_left * 1000000
        self.vel_right = (1/self.gear_ratio*6.29184)/time_right * 1000000
        
    def state_vector(self):
        gyro_x = 0
        gyro_y = 0
        gyro_z = 0
        acc_x = 0
        acc_y = 0
        acc_z = 0
        voltage = 0
        
        return np.array([self.rot_left, self.rot_right, self.vel_left, self.vel_right,
                        gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z,
                        voltage])

# Forward is battery cover when the balboa is up.
# Left motor is 0. Positive direction makes the balboa go forward.
# Right motor is 1

class BalboaEnvMotor(gym.Env):
    def __init__(self, renders=False):
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
        self.balboa_state = BalboaState(self.comms)
        self.max_PWM_value = 400
        
        


    def step(self, action):
        
        # Step the environment
        # comms.motors(action[0], action[1])
        
        self.balboa_state.read_balboa_sensor()
        
        reward = 0
        done = 0
        return self.balboa_state.state_vector(), reward, done, {}
        
    def reset(self):
        self.balboa_state.read_balboa_sensor()
        return self.balboa_state.state_vector()
        
    def _seed(self, seed=None):
        return [0]
        