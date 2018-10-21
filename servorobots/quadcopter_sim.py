"""
Classic cart-pole system implemented by Rich Sutton et al.
Copied from https://webdocs.cs.ualberta.ca/~sutton/book/code/pole.c
"""
import os,  inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0,parentdir)

import logging
import math
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import time
import subprocess
# See user manual https://usermanual.wiki/Document/pybullet20quickstart20guide.479068914.pdf
import pybullet as p
import pybullet_data
from pkg_resources import parse_version

logger = logging.getLogger(__name__)

class QuadcopterEnv(gym.Env):
    def __init__(self, renders=False):
        self._renders = renders
        if (renders):
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)

    # Quadcopter observation:
        # Quaternion (4)
        # Angular velocity (3), Local frame
        # Position (3)
        # Velocity (3), Local frame

        observation_dim = 13
        high_obs = np.ones([observation_dim])
        self.observation_space = spaces.Box(-high_obs, high_obs)

        # Thrust, torque in NED (roll, pitch, yaw)
        action_dim = 4
        act_high = np.ones([action_dim])
        self.action_space = spaces.Box(-act_high, act_high)

    def _configure(self, display=None):
        self.display = display

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _step(self, action):
        p.stepSimulation()
        if self._renders:
            time.sleep(self.timeStep)
            keys = p.getKeyboardEvents()
            for k, v in keys.items():

                if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_TRIGGERED)):
                    turn = -0.5
                if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED)):
                    turn = 0
                if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_TRIGGERED)):
                    turn = 0.5
                if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_RELEASED)):
                    turn = 0

                if (k == p.B3G_UP_ARROW and (v & p.KEY_WAS_TRIGGERED)):
                    self.forward = 0.05
                if (k == p.B3G_UP_ARROW and (v & p.KEY_WAS_RELEASED)):
                    self.forward = 0.0
                if (k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_TRIGGERED)):
                    self.forward = -0.05
                if (k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_RELEASED)):
                    self.forward = 0.0
                self.offset_command = self.offset_command + self.forward
                # print(self.offset_command)
                #print(keys)

        # p.getJointState(self.quad, 0)

        reward = 1
        #if self._renders:
            #print(math.fabs(x)/2.4)
            #print(math.fabs(self.acceleration)*1)
        world_pos, world_ori = p.getBasePositionAndOrientation(self.quad)
        world_vel, world_rot_vel = p.getBaseVelocity(self.quad)
        self.state =  world_pos + world_ori + world_vel + world_rot_vel
        print(self.state)
        done = 0
        return np.array(self.state), reward, done, {}

    def _reset(self):
    #    print("-----------reset simulation---------------")
        p.resetSimulation()
        self.quad = p.loadURDF(os.path.join(currentdir, "quad.urdf"),[0,0,0])

        self.timeStep = 0.01
        p.setGravity(0,0, -10)
        p.setTimeStep(self.timeStep)
        p.setRealTimeSimulation(0)

        initialCartPos = self.np_random.uniform(low=-2, high=2, size=(1,))
        initialAngle = self.np_random.uniform(low=-0.5, high=0.5, size=(1,))
        # p.resetJointState(self.quad, 1, initialAngle)
        # p.resetJointState(self.quad, 0, initialCartPos)


        world_pos, world_ori = p.getBasePositionAndOrientation(self.quad)
        world_vel, world_rot_vel = p.getBaseVelocity(self.quad)
        print(type(world_vel))

        self.state = world_pos + world_ori + world_vel + world_rot_vel
        print(self.state)

        return np.array(self.state)

    def _render(self, mode='human', close=False):
        return

    if parse_version(gym.__version__)>=parse_version('0.9.6'):
        render = _render
        reset = _reset
        seed = _seed
        step = _step