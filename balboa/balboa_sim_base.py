
import numpy as np
import time
import os, inspect
import pybullet as p
from gym import spaces
from servorobots.components.dc_motor import GearedDcMotor
from servorobots.components.dc_motor import TimestampInput
import random

from servorobots.tools.quaternion import qt

import pybullet_data

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

# The environment is separated from the robot. This way, it is easy to change the reward, environment, and
# robot parameters without touching the robot code.

class BalboaSim:
    def __init__(self, renders=False, sim_timestep=0.0020, action_every_x_timestep=5):
        self.resets = 0
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
        self.max_voltage = 7.2
        self.r_battery = 0.4

        # Balancer observation: All in SI
        # Motor Rot Left, Rot Right (2)
        # Motor Speed Left, Right (2)
        # Angular velocity (3), Local frame
        # Acceleration (3), Local frame for later
        # Voltage (1)
        self.observation_size = 38
        high_obs = np.ones(self.observation_size)
        self.observation_space = spaces.Box(high_obs * -1, high_obs * 1)

        self.action_size = 4
        act_high = np.asarray([1, 1, 1, 1])
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
        acc = (np.asarray(local_vel) - np.asarray(self.last_vel))/timestep + np.asarray(local_grav)
        self.last_vel = local_vel

        return local_vel, local_rot_vel, acc

    def render(self, mode='human'):
        time.sleep(self.sim_timestep*self.action_every_x_timestep)

    def step(self, action):

        if self._renders:
            time.sleep(self.sim_timestep*self.action_every_x_timestep)
        # Obtain local pose
        local_vel, local_rot_vel, acc = self.local_pose(2, self.sim_timestep*self.action_every_x_timestep)

        # Obtain encoders pose
        rot_left, vel_left, _, _ = p.getJointState(self.robot, 0)
        rot_right, vel_right, _, _ = p.getJointState(self.robot, 1)

        """

        vel_left = self.old_vel_left *0.9 + vel_left*0.1
        self.old_vel_left = vel_left
        vel_right = self.old_vel_right * 0.9 + vel_right * 0.1
        self.old_vel_right = vel_right

        """
        ''' '''
        vel_left = (rot_left - self.previous_rot_left) / 0.002
        self.previous_rot_left = rot_left
        vel_right = (rot_right - self.previous_rot_right) / 0.002
        self.previous_rot_right = rot_right

        if vel_left < -25: vel_left = -25
        if vel_right < -25: vel_right = -25

        ## The voltage goes downs as a function of current due to battery resistance.
        supplied_voltage = self.max_voltage - self.r_battery * self.max_voltage * \
                           abs(self.previous_current_left) + \
                           abs(self.previous_current_right)
        # Apply torque to motors
        torque_left, current_left = self.motor_left.torque_from_voltage(
            TimestampInput(action[0] * supplied_voltage, self.time), vel_left)
        torque_right, current_right = self.motor_right.torque_from_voltage(
            TimestampInput(action[1] * supplied_voltage, self.time), vel_right)
        motor_forces = [torque_left, torque_right]

        self.previous_current_left = current_left
        self.previous_current_right = current_right

        p.setJointMotorControlArray(self.robot, [0, 1], p.VELOCITY_CONTROL, targetVelocities=[0, 0], forces=[0, 0])
        p.setJointMotorControlArray(self.robot, [0, 1], p.TORQUE_CONTROL, forces=motor_forces)

        for i in range(0, self.action_every_x_timestep):

            # p.resetBasePositionAndOrientation(self.robot, [0, 0.05, 0.5], [0, 0, 0, 1])
            p.stepSimulation()
            self.time += self.sim_timestep
        self.internal_state = [self.internal_state[0] * 0.95 + 0.05 * action[2],
                               self.internal_state[1] * 0.99 + 0.01 * action[3]]
        # 0,        1,         2,   3,     4
        # Vel_left, vel_right, yaw, pitch, roll,
        state_t_0 = np.concatenate(([vel_left, vel_right], local_rot_vel, acc, [supplied_voltage]))

        self.state = np.concatenate((state_t_0, self.state_t_m_1, self.state_t_m_2, self.state_t_m_3, np.asarray(self.internal_state)))
        self.state_t_m_3 = self.state_t_m_2
        self.state_t_m_2 = self.state_t_m_1
        self.state_t_m_1 = state_t_0

        #### Check for contact ####
        contacts = p.getContactPoints(bodyA=self.robot, linkIndexA=-1)
        if contacts != ():
            contact = 1
        else:
            contact = 0
        position, orientation = p.getBasePositionAndOrientation(self.robot)
        return self.state, contact, self.time, position, orientation, local_vel


    def reset(self, x=0, y=0, z=0.05, q1=0, q2=0, q3=0, q4=1, gravity = -9.81,
                    ML_R=21.5, ML_Kv=10.5, ML_Kvis=0.0005,
                    MR_R=21.5, MR_Kv=10.5, MR_Kvis=0.0005,
                    latency=0.02):

        if self.resets == 0:
            p.resetSimulation()
            p.setGravity(0, 0, gravity)
            print("hi!")

            try:
                offset_cg_x = (random.random() * 2 - 1) * 0.002 + -0.003
                offset_cg_z = (random.random() * 2 - 1) * 0.005 + 0.03
                cg = str(offset_cg_x) + ", 0, " + str(offset_cg_z)
                doc = xml.dom.minidom.parse("balboa/balancer.urdf")
                doc.getElementsByTagName("link")[0].getElementsByTagName("inertial")[0].getElementsByTagName("origin")[
                    0].setAttribute("xyz", cg)
                with open("balboa/balancer_randomized.urdf", "w") as file_handle:
                    file_handle.write(doc.toxml())
            except:
                pass

            try:
                self.robot = p.loadURDF(os.path.join(currentdir, "balancer_randomized.urdf"), [x, y, z], [q1, q2, q3, q4],
                                   flags=p.URDF_USE_INERTIA_FROM_FILE)
            except:
                self.robot = p.loadURDF(os.path.join(currentdir, "balancer.urdf"), [x, y, z], [q1, q2, q3, q4],
                                        flags=p.URDF_USE_INERTIA_FROM_FILE)


        self.previous_rot_left, vel_left, _, _ = p.getJointState(self.robot, 0)
        self.previous_rot_right, vel_right, _, _ = p.getJointState(self.robot, 1)
        self.previous_current_left = 0
        self.previous_current_right = 0
        self.old_vel_left = 0
        self.old_vel_right = 0
        self.time = 0

        self.motor_left = GearedDcMotor(R=ML_R, Kv=ML_Kv, K_viscous=ML_Kvis, K_load=0,
                                        timestep=self.sim_timestep, latency=latency)
        self.motor_right = GearedDcMotor(R=MR_R, Kv=MR_Kv, K_viscous=MR_Kvis, K_load=0,
                                         timestep=self.sim_timestep, latency=latency)

        # Change friction of main body
        p.changeDynamics(self.robot, -1, lateralFriction=0.3, restitution=0.5)
        # Change friction of left wheel
        p.changeDynamics(self.robot, 0, lateralFriction=2, restitution=0.0)
        # Change friction of the right wheel
        p.changeDynamics(self.robot, 1, lateralFriction=2, restitution=0.0)
        

        self.internal_state = [0, 0]

        local_vel, local_rot_vel, acc = self.local_pose(2, self.sim_timestep * self.action_every_x_timestep)
        state_t_0 = np.concatenate(([0, 0], [0, 0, 0], acc, [self.max_voltage]))
        self.state_t_m_1 = state_t_0
        self.state_t_m_2 = state_t_0
        self.state_t_m_3 = state_t_0
        self.state = np.concatenate((state_t_0, self.state_t_m_1, self.state_t_m_2, self.state_t_m_3, np.asarray(self.internal_state)))


        ##### Environment section
        self.resets += 1

        return self.state