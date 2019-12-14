import gym
from gym import spaces
import balboa.balboa_sim_base as balboa_sim
import numpy as np
import pybullet_data
import random
import os

### This environment contains the reward and the location information


class BalboaEnvSimBalanceCtrl(gym.Env):
    def __init__(self, renders=False):
        self.sim = balboa_sim.BalboaSim(renders=renders)

        if renders:
            self.slider_speed = self.sim.p.addUserDebugParameter("Speed", -0.25, 0.25, 0)
            self.slider_yaw_speed = self.sim.p.addUserDebugParameter("Yaw Speed", -3, 3, 0)

        high_obs = np.ones(self.sim.observation_size + 2)
        self.observation_space = spaces.Box(high_obs * -1, high_obs * 1)
        self.action_space = self.sim.action_space

    def render(self, mode='human'):
        self.sim.render()

    def step(self, action, ctrl=None):
        obs, contact, time, position, orientation, local_vel = self.sim.step(action)



        ### Reward Calculation ###
        # The f_ are factors that are 1 when ideal, and tend to 0 when not ideal. They are annealed during training
        # to be always 1 at the beginning.
        speed = (local_vel[2] - self.desired_speed) * 4 * self.difficulty * 2 # Max speed, 0.25 m/s.
        f_speed = 1 / (speed**2 + 1)

        speed_yaw = (obs[2] - self.desired_yaw) * 0.33 * self.difficulty * 2
        f_speed_yaw = 1 / (speed_yaw**2 + 1)

        # Reduce oscillation pitch [3]
        speed_pitch = abs(obs[3]) * self.difficulty * 4
        f_speed_pitch = 1 / (speed_pitch ** 2 + 1)

        # Gives 1 per step for being straight, 0 for lying down.
        upright = 1 - abs(self.sim.p.getEulerFromQuaternion(orientation)[1]) / 1.58
        reward = upright * f_speed * f_speed_yaw * f_speed_pitch
        done = contact
        #TODO Normalize observations?
        if self.sim._renders:
            obs = np.append(obs, [self.sim.p.readUserDebugParameter(self.slider_speed),
                                  self.sim.p.readUserDebugParameter(self.slider_yaw_speed)])
        elif ctrl == None:
            obs = np.append(obs, [self.desired_speed, self.desired_yaw])
        else:
            obs = np.append(obs, ctrl)



        return np.array(obs), reward, done, {"time": time}

    def reset(self, x=0, y=0, z=0.05, q1=0, q2=0, q3=0, q4=1, gravity = -9.81,
                    ML_R=21.5, ML_Kv=10.5, ML_Kvis=0.0005,
                    MR_R=21.5, MR_Kv=10.5, MR_Kvis=0.0005,
                    latency=0.02):
        # Lying down q1 = 0 q2 = 0.7071 q3 = 0 q4 = 0.7071
        rand_num = random.random()*2-1
        rand_ori = np.asarray([0, rand_num, 0, abs(rand_num)]) + np.asarray([0, 0, 0, 3])
        rand_ori = rand_ori/np.linalg.norm(rand_ori)

        # rand_ori = np.asarray([0, -0.7071, 0, 0.7071])
        obs = self.sim.reset(x=x, y=y, z=z, q1=rand_ori[0], q2=rand_ori[1], q3=rand_ori[2], q4=rand_ori[3], gravity = gravity,
                    ML_R=ML_R, ML_Kv=ML_Kv, ML_Kvis=ML_Kvis,
                    MR_R=MR_R, MR_Kv=MR_Kv, MR_Kvis=MR_Kvis,
                    latency=latency)

        self.desired_speed = (random.random()*2.0 - 1.0) * 0.25
        self.desired_yaw = (random.random()*2.0 - 1.0) * 3 # About 1 turn per second

        self.sim.p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-20, cameraTargetPosition=[0, 0, 0])

        filename = os.path.join(pybullet_data.getDataPath(), "plane_stadium.sdf")
        self.ground_plane_mjcf = self.sim.p.loadSDF(filename)


        with open('balboa/progress.txt', 'r') as file:
            self.difficulty = file.read()
            self.difficulty = float(self.difficulty)

        obs = np.append(obs, [self.desired_speed, self.desired_yaw])

        return obs

    def _seed(self, seed=None):
        return [0]