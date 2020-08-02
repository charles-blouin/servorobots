import numpy as np
import time
import os, inspect
import pybullet as p
from gym import spaces
import time
from servorobots.tools.quaternion import qt
import random
import math

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))


class PendulumASim:
    def __init__(self, renders=False, evaluate=False, sim_timestep=0.0025, action_every_x_timestep=4):
        self._renders = renders
        self.evaluate = evaluate
        self.p = p
        if (renders):
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)
        self.sim_timestep = sim_timestep
        self.action_every_x_timestep = action_every_x_timestep

        self.time = 0
        self.last_vel = [0, 0, 0]

        self.include_joint_velocity = True
        self.cos_representation = True
        self.observation_size = 2 * ( 1 + self.include_joint_velocity + self.cos_representation)
        high_obs = np.ones(self.observation_size)
        self.observation_space = spaces.Box(high_obs * -1, high_obs * 1)
        self.number_of_joints = 2
        self.action_size = 1
        act_high = np.asarray([1 for i in range(self.number_of_joints)])
        self.action_space = spaces.Box(-act_high, act_high)

    def local_pose(self, linkID, timestep):
        # world_pos, world_ori = p.getBasePositionAndOrientation(self.robot)
        _, _, _, _, world_pos, world_ori, world_vel, world_rot_vel = p.getLinkState(self.robot, linkID, True, True)
        #        world_pos_offset = tuple([world_pos[0] - self.dx]) + world_pos[1:3]
        # world_vel, world_rot_vel = p.getBaseVelocity(self.robot)
        # Convert to local pose
        # Take a vector in world_orientation and express it in local orientation
        local_rot_vel = qt.qv_mult(qt.q_conjugate(world_ori), world_rot_vel)
        local_vel = qt.qv_mult(qt.q_conjugate(world_ori), world_vel)
        local_grav = qt.qv_mult(qt.q_conjugate(world_ori), (0, 0, 9.81))
        acc = (np.asarray(local_vel) - np.asarray(self.last_vel)) / timestep + np.asarray(local_grav)
        self.last_vel = local_vel

        return local_vel, local_rot_vel, acc

    def getJointStates(self, robot):
        joint_states = p.getJointStates(robot, range(p.getNumJoints(robot)))
        joint_positions = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        return joint_positions, joint_velocities


    def render(self, mode='human'):
        time.sleep(self.sim_timestep * self.action_every_x_timestep)

    def get_obs(self):
        jointPosition, jointVelocity = self.getJointStates(self.robot)

        if self.cos_representation:
            jointPosition = [math.sin(jointPosition[0]), math.cos(jointPosition[0]),
                             math.sin(jointPosition[1]), math.cos(jointPosition[1])]

        if self.include_joint_velocity:
            jointPosition.extend(jointVelocity)
            obs = jointPosition
        else:
            obs = jointPosition
        return obs


    def step(self, action):
        # action = np.multiply(action, self.limit)
        if self._renders:
            time.sleep(self.sim_timestep * self.action_every_x_timestep)

        # Obtain local pose
        # local_vel, local_rot_vel, acc = self.local_pose(0, self.sim_timestep * self.action_every_x_timestep)

        p.setJointMotorControlArray(self.robot, range(p.getNumJoints(self.robot)), p.VELOCITY_CONTROL,
                                    targetVelocities = [action[0] * 10, 0],
                                    forces=[0.25, 0]) # Assuming the torque is 0.26 Ncm

        # p.setJointMotorControlArray(self.robot, range(p.getNumJoints(self.robot)), p.TORQUE_CONTROL,
        #                            targetVelocities=[action[0] * 20, 0],
        #                            forces=[0.25, 0])  # Assuming the torque is 0.26 Ncm
        for i in range(0, self.action_every_x_timestep):
            p.stepSimulation()
            self.time += self.sim_timestep

        obs = self.get_obs()

        return obs, self.time

    def reset(self, gravity=-9.81):
        p.resetSimulation()
        p.createCollisionShape(p.GEOM_PLANE)
        p.createMultiBody(0, 0)
        p.setGravity(0, 0, gravity)
        self.time = 0
        self.robot = p.loadURDF('pendulum_a/urdf/pendulum_a.urdf', basePosition=[0, 0, 1],
                                useFixedBase=True,
        flags=p.URDF_USE_INERTIA_FROM_FILE & p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT)

        if self.evaluate:
            rand_vel = 10
        else:
            rand_vel = random.randint(0, 20)
        # To make the environment harder
        if rand_vel < 15:
            for i in range(20):
                p.setJointMotorControlArray(self.robot, range(p.getNumJoints(self.robot)), p.VELOCITY_CONTROL,
                                            targetVelocities=[rand_vel, rand_vel],
                                            forces=[0.25, 0])
                p.stepSimulation()


        local_vel, local_rot_vel, acc = self.local_pose(0, self.sim_timestep * self.action_every_x_timestep)
        obs = self.get_obs()
        # p.setJointMotorControlArray(self.robot, range(p.getNumJoints(self.robot)), p.VELOCITY_CONTROL, force=[0, 0])

        ##### Environment section

        return obs