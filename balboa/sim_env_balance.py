import gym
import balboa.balboa_sim_base as balboa_sim
import numpy as np
import pybullet as p
import pybullet_data
import os

### This environment contains the reward and the location information


class BalboaEnvSimBalance(gym.Env):
    def __init__(self, renders=False):
        self.sim = balboa_sim.BalboaSim(renders=renders)

        self.observation_space = self.sim.observation_space
        self.action_space = self.sim.action_space

    def step(self, action):
        obs, contact, time, orientation = self.sim.step(action)

        # Gives 1 at 20 rad (about three wheel turn)
        distance = (abs(obs[0])+abs(obs[1]))/40
        # Gives 1 per step for being straight, -0.5 for lying down.
        upright = 1-abs(p.getEulerFromQuaternion(orientation)[1])/1.55
        reward = upright - distance
        print(reward)
        done = 0
        return np.array(obs), reward, done, {"time": time}

    def reset(self, x=0, y=0, z=0.05, q1=0, q2=0, q3=0, q4=1, gravity = -9.81,
                    ML_R=21.5, ML_Kv=10.5, ML_Kvis=0.0005,
                    MR_R=21.5, MR_Kv=10.5, MR_Kvis=0.0005,
                    latency=0.02):
        # Lying down q1 = 0 q2 = 0.7071 q3 = 0 q4 = 0.7071

        rand_ori = np.random.uniform(low=-1, high=1, size=(4,)) + np.asarray([0, 0, 0, 4])
        rand_ori = rand_ori/np.linalg.norm(rand_ori)
        self.sim.reset(x=x, y=y, z=z, q1=rand_ori[0], q2=rand_ori[1], q3=rand_ori[2], q4=rand_ori[3], gravity = gravity,
                    ML_R=ML_R, ML_Kv=ML_Kv, ML_Kvis=ML_Kvis,
                    MR_R=MR_R, MR_Kv=MR_Kv, MR_Kvis=MR_Kvis,
                    latency=latency)
        p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=55, cameraPitch=-20, cameraTargetPosition=[0, 0, 0])

        filename = os.path.join(pybullet_data.getDataPath(), "plane_stadium.sdf")
        self.ground_plane_mjcf = p.loadSDF(filename)

    def _seed(self, seed=None):
        return [0]