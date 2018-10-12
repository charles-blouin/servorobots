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


import time

logger = logging.getLogger(__name__)

class CartPoleServoEnv(gym.Env):

# parameter is passed when the environment is registered: https://github.com/openai/gym/issues/748
  def __init__(self, renders=False):
    # start the bullet physics server
    self._renders = renders
    if (renders):
	    p.connect(p.GUI)
    else:
    	p.connect(p.DIRECT)

    observation_high = np.array([
          np.finfo(np.float32).max,
          np.finfo(np.float32).max,
          np.finfo(np.float32).max,
          np.finfo(np.float32).max])
    action_high = np.array([0.1])

    self.action_space = spaces.Box(low=-2.4, high=2.4, shape=(1,))
    self.observation_space = spaces.Box(-observation_high, observation_high)

    self.theta_threshold_radians = 1
    self.x_threshold = 6
    self._seed()
#    self.reset()
    self.viewer = None
    self._configure()
    self.offset_command = 0  # For keyboard control, the offset to zero desired.
    self.last_velocity = 0  # to compute acceleration
    self.last_acceleration = 0 # to compute jerk

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
    theta, theta_dot, x, x_dot = p.getJointState(self.cartpole, 1)[0:2] + p.getJointState(self.cartpole, 0)[0:2]
    self.acceleration = action[0] - self.last_velocity
    self.last_velocity = action[0]
    self.jerk = self.acceleration - self.last_acceleration
    self.last_acceleration = self.acceleration
    x = x + self.offset_command
    self.state = (theta, theta_dot, x, x_dot)


    p.setJointMotorControl2(self.cartpole, 0, p.VELOCITY_CONTROL, targetVelocity=action, velocityGain=1)

    done =  x < -self.x_threshold \
                or x > self.x_threshold \
                or theta < -self.theta_threshold_radians \
                or theta > self.theta_threshold_radians
    reward = 1 - math.fabs(x)/2.4 - math.fabs(self.jerk)/1.5
    if self._renders:
        #print(math.fabs(x)/2.4)
        #print(math.fabs(self.acceleration)*1)
        print(self.jerk)

    return np.array(self.state), reward, done, {}

  def _reset(self):
#    print("-----------reset simulation---------------")
    p.resetSimulation()
    self.cartpole = p.loadURDF(os.path.join(pybullet_data.getDataPath(),"cartpole.urdf"),[0,0,0])

    self.timeStep = 0.01
    p.setJointMotorControl2(self.cartpole, 1, p.VELOCITY_CONTROL, force=0)
    p.setGravity(0,0, -10)
    p.setTimeStep(self.timeStep)
    p.setRealTimeSimulation(0)

    initialCartPos = self.np_random.uniform(low=-2, high=2, size=(1,))
    initialAngle = self.np_random.uniform(low=-0.5, high=0.5, size=(1,))
    p.resetJointState(self.cartpole, 1, initialAngle)
    p.resetJointState(self.cartpole, 0, initialCartPos)

    self.state = p.getJointState(self.cartpole, 1)[0:2] + p.getJointState(self.cartpole, 0)[0:2]

    return np.array(self.state)

  def _render(self, mode='human', close=False):
      return

  if parse_version(gym.__version__)>=parse_version('0.9.6'):
    render = _render
    reset = _reset
    seed = _seed
    step = _step
