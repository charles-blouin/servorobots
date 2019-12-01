
import numpy as np
import time
import os, inspect
import pybullet as p
from gym import spaces
from servorobots.components.test_motor import GearedDcMotor
from servorobots.components.test_motor import TimestampInput

from servorobots.tools.quaternion import qt

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

# The environment is separated from the robot. This way, it is easy to change the reward, environment, and
# robot parameters without touching the robot code.

class BalboaSim:
    def __init__(self, renders=False, sim_timestep=0.01, action_every_x_timestep=1):
        self._renders = renders
        if (renders):
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)
        self.sim_timestep = sim_timestep
        self.action_every_x_timestep = action_every_x_timestep

        self.time = 0
        self.last_vel = [0, 0, 0]
        self.max_voltage = 7.4

        # Balancer observation: All in SI
        # Motor Rot Left, Rot Right (2)
        # Motor Speed Left, Right (2)
        # Angular velocity (3), Local frame
        # Acceleration (3), Local frame for later
        # Voltage (1)
        self.observation_size = 10
        high_obs = np.ones(self.observation_size)
        self.observation_space = spaces.Box(high_obs * -1, high_obs * 1)

        self.action_size = 2
        act_high = np.asarray([1, 1])
        self.action_space = spaces.Box(-act_high, act_high)

    def local_pose(self):
        world_pos, world_ori = p.getBasePositionAndOrientation(self.robot)
#        world_pos_offset = tuple([world_pos[0] - self.dx]) + world_pos[1:3]
        world_vel, world_rot_vel = p.getBaseVelocity(self.robot)
        # Convert to local pose
        local_rot_vel = qt.qv_mult(qt.q_conjugate(world_ori), world_rot_vel)
        local_vel = qt.qv_mult(qt.q_conjugate(world_ori), world_vel)
        local_grav = qt.qv_mult(qt.q_conjugate(world_ori), (0, 0, -9.81))
        acc = np.asarray(local_vel) - np.asarray(self.last_vel) + np.asarray(local_grav)
        self.last_vel = local_vel
        return local_rot_vel, acc

    def step(self, action):
        for i in range(0, self.action_every_x_timestep+1):
            p.stepSimulation()

            if self._renders:
                time.sleep(self.sim_timestep)

            # Obtain local pose
            local_rot_vel, acc = self.local_pose()

            # Obtain encoders pose
            rot_right, vel_right, _, _ = p.getJointState(self.robot, 0)
            rot_left, vel_left, _, _ = p.getJointState(self.robot, 1)


            # Apply torque to motors
            print('step')
            torque_left, current_left = self.motor_left.torque_from_voltage(
                TimestampInput(action[0] * self.max_voltage, self.time), vel_left)
            torque_right, current_right = self.motor_right.torque_from_voltage(
                TimestampInput(action[1] * self.max_voltage, self.time), vel_right)
            print(torque_left)
            print(rot_left)
            motor_forces = [torque_left, torque_right]

            p.setJointMotorControlArray(self.robot, [0, 1], p.VELOCITY_CONTROL, targetVelocities=[0, 0], forces=[0, 0])
            p.setJointMotorControlArray(self.robot, [0, 1], p.TORQUE_CONTROL, forces=motor_forces)

            state = np.concatenate(([rot_left, rot_right, vel_left, vel_right], local_rot_vel, acc, [self.max_voltage]))
            self.state = state

            #### Check for contact ####
            contacts = p.getContactPoints(bodyA=self.robot, linkIndexA=-1)
            if contacts != ():
                contact = 1
            else:
                contact = 0
            self.time += self.sim_timestep

        return self.state, contact, self.time


    def reset(self, x=0, y=0, z=0.05, q1=0, q2=0, q3=0, q4=1, gravity = -9.81,
                    ML_R=3, ML_Kv=9, ML_Kvis=0.0005,
                    MR_R=3, MR_Kv=9, MR_Kvis=0.0005,
                    latency=0.02):
        p.resetSimulation()
        p.setGravity(0, 0, gravity)

        self.robot = p.loadURDF(os.path.join(currentdir, "balancer.urdf"), [x, y, z], [q1, q2, q3, q4],
                               flags=p.URDF_USE_INERTIA_FROM_FILE)

        # Change friction of main body
        p.changeDynamics(self.robot, -1, lateralFriction=0.3, restitution=0.5)
        # Change friction of left wheel
        p.changeDynamics(self.robot, 0, lateralFriction=2, restitution=0.0)
        # Change friction of the right wheel
        p.changeDynamics(self.robot, 1, lateralFriction=2, restitution=0.0)

        self.motor_left = GearedDcMotor(R=ML_R, Kv=ML_Kv, K_viscous=ML_Kvis, K_load=0,
                                        timestep=self.sim_timestep, latency=latency)
        self.motor_right = GearedDcMotor(R=MR_R, Kv=MR_Kv, K_viscous=MR_Kvis, K_load=0,
                                         timestep=self.sim_timestep, latency=latency)