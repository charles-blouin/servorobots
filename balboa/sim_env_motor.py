import gym
import balboa.balboa_sim_base as balboa_sim
import numpy as np
import pybullet as p
import pybullet_data
import os

class BalboaEnvSimMotor(gym.Env):
    def __init__(self, renders=False):
        self.sim = balboa_sim.BalboaSim(renders=renders)

        self.observation_space = self.sim.observation_space
        self.action_space = self.sim.action_space

    def step(self, action):
        obs, contact, time = self.sim.step(action)
        reward = 1
        done = 0
        return np.array(obs), reward, done, {"time": time}

    def reset(self):
        self.sim.reset(q1=0, q2=0.7071, q3=0, q4=0.7071)
        # self.sim.reset(q1=1, q2=0, q3=0, q4=0)
        p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=55, cameraPitch=-20, cameraTargetPosition=[0, 0, 0])

        filename = os.path.join(pybullet_data.getDataPath(), "plane_stadium.sdf")
        self.ground_plane_mjcf = p.loadSDF(filename)

    def _seed(self, seed=None):
        return [0]