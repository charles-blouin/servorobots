"""

Polulu Balboa
"""
#TODO Adjust the inertia and COG of the vehicle

import os, inspect
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

from servorobots.components.test_motor import GearedDcMotor
from servorobots.components.test_motor import TimestampInput

logger = logging.getLogger(__name__)
gym.Env()

class qt:
    def q_mult(q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
        z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
        return x, y, z, w

    def q_conjugate(q):
        x, y, z, w = q
        return (-x, -y, -z, w)

    def qv_mult(q1, v1):
        q2 = v1 + (0.0,)
        return qt.q_mult(qt.q_mult(q1, q2), qt.q_conjugate(q1))[0:3]

class BalancerEnv(gym.Env):
    def __init__(self, renders=False):

        self._renders = renders
        if (renders):
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)
        self._seed()
        self.timestep = 0.003
        self.gravity = -9.8

        self.voltageId = p.addUserDebugParameter("Input Voltage", -5, 5, 0)

        p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

        self.dx = 0
        self.time = 0
        self.max_voltage = 6
        self.motor_left = GearedDcMotor(R=4, Kv=5, K_viscous=0.000122489, K_load=0, timestep=self.timestep, latency=0)
        self.motor_right = GearedDcMotor(R=4, Kv=5, K_viscous=0.000122489, K_load=0, timestep=self.timestep, latency=0)

        # Quadcopter observation:
        # Motor Speed right, left (2)
        # Angular velocity (3), Local frame
        # Acceleration (3), Local frame for later

        observation_dim = 5
        high_obs = np.ones([observation_dim])
        self.observation_space = spaces.Box(high_obs*0, high_obs*1.5)

        # Duty cycle, approximately equal to voltage. Battery voltage is corrected by the batteries.
        action_dim = 1
        act_high = np.asarray([1])
        self.action_space = spaces.Box(-act_high, act_high)

    def _configure(self, display=None):
        self.display = display

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _step(self, action):
        p.stepSimulation()

        # This is to controll manually the quad.
        if self._renders:
            time.sleep(self.timestep)

                #self.offset_command = self.offset_command + self.forward
        if self._renders:
            p.resetBasePositionAndOrientation(self.desired_pos_sphere, [self.dx, 0, 0], [0, 0, 0, 1])

        world_pos, world_ori = p.getBasePositionAndOrientation(self.quad)
        world_pos_offset = tuple([world_pos[0] - self.dx]) + world_pos[1:3]
        world_vel, world_rot_vel = p.getBaseVelocity(self.quad)

        local_rot_vel = qt.qv_mult(qt.q_conjugate(world_ori), world_rot_vel)
        # local_rot = qt.qv_mult(qt.q_conjugate(world_ori), world_ori)



        _, vel_right, _, _ = p.getJointState(self.quad, 0)
        _, vel_left, _, _ = p.getJointState(self.quad, 1)

        # applied_Voltage = p.readUserDebugParameter(self.voltageId)
        torque_right, current_right = self.motor_right.torque_from_voltage(TimestampInput(action[0]*self.max_voltage, self.time), vel_right)
        torque_left, current_left = self.motor_left.torque_from_voltage(TimestampInput(action[0]*self.max_voltage, self.time), vel_left)
        motor_forces = [torque_right, torque_left]



        # world_rot_vel = (1, 0, 0)
        # To obtain local rot_vel
        p.setJointMotorControlArray(self.quad, [0, 1], p.VELOCITY_CONTROL, targetVelocities=[0, 0], forces=[0, 0])
        p.setJointMotorControlArray(self.quad, [0, 1], p.TORQUE_CONTROL, forces=motor_forces)






        self.state = tuple([vel_right, vel_left]) + local_rot_vel


        touched_ground = 0



        distance_from_center = np.linalg.norm(np.asarray(world_pos) - np.asarray([0, 0, 0.5]))
        power = abs(action[0] * self.max_voltage * current_left)
        # print((world_pos[2] - 0.041) * 50)
        # About 1 when 1, just below zero when down
        reward = (world_pos[2] - 0.042) * 50 - power/8

        done = 0
        contacts = p.getContactPoints(bodyA=self.quad, linkIndexA=-1)
        if contacts != ():
            touched_ground = -100
            done = 1

        if distance_from_center > 1:
            done = 1

        self.time += self.timestep

        return np.array(self.state), reward, done, {}

    def _reset(self):
        p.resetSimulation()
        # see https://github.com/bulletphysics/bullet3/issues/1934 to load multiple colors
        p.setGravity(0, 0, self.gravity)
        if self.render:
            visualShapeId = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.01, rgbaColor=[1, 0, 0, 1],
                            specularColor=[0.4, .4, 0])
            self.desired_pos_sphere = p.createMultiBody(baseMass=0, baseInertialFramePosition=[0, 0, 0], baseVisualShapeIndex=visualShapeId)
        rand_ori = self.np_random.uniform(low=-1, high=1, size=(4,)) + np.asarray([0,0,0,2])
        rand_ori = rand_ori/np.linalg.norm(rand_ori)
        rand_pos = self.np_random.uniform(low=-0.5, high=0.5, size=(3,))



        self.quad = p.loadURDF(os.path.join(currentdir, "balancer.urdf"),[0,0,0.05], [0, 0, 0, 1], flags=p.URDF_USE_INERTIA_FROM_FILE)

        p.changeDynamics(self.quad, -1, lateralFriction=0.3, restitution=0.5)
        p.changeVisualShape(self.quad, -1, rgbaColor=[1, 1, 0, 1]) # yellow

        p.changeDynamics(self.quad, 0, lateralFriction=1, restitution=0.5)
        p.changeVisualShape(self.quad, 0, rgbaColor=[1, 0, 0, 1]) # red

        p.changeDynamics(self.quad, 1, lateralFriction=1, restitution=0.5)
        p.changeVisualShape(self.quad, 1, rgbaColor=[0, 1, 0, 1])  # blue

        filename = os.path.join(pybullet_data.getDataPath(), "plane_stadium.sdf")
        self.ground_plane_mjcf = p.loadSDF(filename)
        # filename = os.path.join(pybullet_data.getDataPath(),"stadium_no_collision.sdf")
        # self.ground_plane_mjcf = self._p.loadSDF(filename)
        #
        for i in self.ground_plane_mjcf:
            p.changeDynamics(i, -1, lateralFriction=0.8, restitution=0.5)
            p.changeVisualShape(i, -1, rgbaColor=[1, 1, 1, 0.8])

        p.changeDynamics(self.quad, -1, linearDamping=0, angularDamping=0)
        for j in range(p.getNumJoints(self.quad)):
            p.changeDynamics(self.quad, j, linearDamping=0, angularDamping=0)




        # Default 0.04 for linear and angular damping.
        # p.changeDynamics(self.quad, -1, linearDamping=1)
        # p.changeDynamics(self.quad, -1, angularDamping=1)
        p.setTimeStep(self.timestep)
        p.setRealTimeSimulation(0)

        info = p.getDynamicsInfo(self.quad, -1)
        self.mass = info[0]
        self.zero_thrust = -self.mass*self.gravity

        initialCartPos = self.np_random.uniform(low=-2, high=2, size=(1,))
        initialAngle = self.np_random.uniform(low=-0.5, high=0.5, size=(1,))
        # p.resetJointState(self.quad, 1, initialAngle)
        # p.resetJointState(self.quad, 0, initialCartPos)

        world_pos, world_ori = p.getBasePositionAndOrientation(self.quad)
        world_vel, world_rot_vel = p.getBaseVelocity(self.quad)

        local_rot_vel = qt.qv_mult(qt.q_conjugate(world_ori), world_rot_vel)
        # local_rot = qt.qv_mult(qt.q_conjugate(world_ori), world_ori)

        _, vel_right, _, _ = p.getJointState(self.quad, 0)
        _, vel_left, _, _ = p.getJointState(self.quad, 1)

        self.state = tuple([vel_right, vel_left]) + local_rot_vel
        return np.array(self.state)

    def _render(self, mode='human', close=False):
        return

    if parse_version(gym.__version__)>=parse_version('0.9.6'):
        render = _render
        reset = _reset
        seed = _seed
        step = _step