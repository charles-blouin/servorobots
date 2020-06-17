
import numpy as np
import time
import os, inspect
import pybullet as p
from gym import spaces
import time
import walker_a.geometry as geo
from servorobots.tools.quaternion import qt

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))


class WalkerASim:
    def __init__(self, renders=False, sim_timestep=0.0025, action_every_x_timestep=4):
        self._renders = renders
        self.p = p
        if (renders):
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)
        self.sim_timestep = sim_timestep
        self.action_every_x_timestep = action_every_x_timestep

        self.time = 0
        self.last_vel = [0, 0, 0]
        self.limit = np.asarray([0.1, 0.4, 0.4,
                      0.1, 0.4, 0.4,
                      0.1, 0.4, 0.4,
                      0.1, 0.4, 0.4])
        self.include_gyro = False
        self.include_joint_velocity = True
        self.observation_size = 3*self.include_gyro + 12 + self.include_joint_velocity * 12
        high_obs = np.ones(self.observation_size)
        self.observation_space = spaces.Box(high_obs * -1, high_obs * 1)
        self.number_of_joints = 12
        self.action_size = 12
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

    def step(self, action):
        # action = np.multiply(action, self.limit)
        if self._renders:
            time.sleep(self.sim_timestep * self.action_every_x_timestep)

        # Obtain local pose
        local_vel, local_rot_vel, acc = self.local_pose(0, self.sim_timestep * self.action_every_x_timestep)

        p.setJointMotorControlArray(self.robot, range(p.getNumJoints(self.robot)), p.POSITION_CONTROL,
                                    targetPositions=action,
                                    forces=[50 for i in range(self.number_of_joints)])
        for i in range(0, self.action_every_x_timestep):
            p.stepSimulation()
            self.time += self.sim_timestep

        jointPosition, jointVelocity = self.getJointStates(self.robot)
        if self.include_joint_velocity:
            state = jointPosition.extend(jointVelocity)
        else:
            state = jointPosition
        # state_t_0 = np.concatenate((local_rot_vel, acc))
        #### Check for contact ####
        contacts = p.getContactPoints(bodyA=self.robot, linkIndexA=-1)
        if contacts != ():
            contact = 1
        else:
            contact = 0
        position, orientation = p.getBasePositionAndOrientation(self.robot)
        return jointPosition, contact, self.time, position, orientation, local_vel, local_rot_vel

    def reset(self, x=0, y=0, z=0.05, q1=0, q2=0, q3=0, q4=1, gravity=-9.81,
              ML_R=21.5, ML_Kv=10.5, ML_Kvis=0.0005,
              MR_R=21.5, MR_Kv=10.5, MR_Kvis=0.0005,
              latency=0.02):
        p.resetSimulation()
        p.createCollisionShape(p.GEOM_PLANE)
        p.createMultiBody(0, 0)
        p.setGravity(0, 0, gravity)
        self.time = 0
        self.robot = p.loadURDF('walker_a/urdf/walker_a_0_5.urdf', basePosition=[0, 0, 1],
                            flags=p.URDF_USE_INERTIA_FROM_FILE & p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT)


        local_vel, local_rot_vel, acc = self.local_pose(0, self.sim_timestep * self.action_every_x_timestep)
        jointPosition, jointVelocity = self.getJointStates(self.robot)
        if self.include_joint_velocity:
            state = jointPosition.extend(jointVelocity)
        else:
            state = jointPosition


        ##### Environment section

        return jointPosition