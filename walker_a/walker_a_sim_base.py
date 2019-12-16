
import numpy as np
import time
import os, inspect
import pybullet as p
from gym import spaces
import time

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

class WalkerASim:
    def __init__(self, renders=False, sim_timestep=0.0020, action_every_x_timestep=5):
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


        self.observation_size = 36
        high_obs = np.ones(self.observation_size)
        self.observation_space = spaces.Box(high_obs * -1, high_obs * 1)

        self.action_size = 8
        act_high = np.asarray([1, 1, 1, 1, 1, 1, 1, 1])
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

        def render(self, mode='human'):
            time.sleep(self.sim_timestep * self.action_every_x_timestep)

        def step(self, action):

            if self._renders:
                time.sleep(self.sim_timestep * self.action_every_x_timestep)
            # Obtain local pose
            local_vel, local_rot_vel, acc = self.local_pose(2, self.sim_timestep * self.action_every_x_timestep)
            for i in range(0, self.action_every_x_timestep):
                # Obtain encoders pose

                p.stepSimulation()
                self.time += self.sim_timestep
            # 0,        1,         2,   3,     4
            # Vel_left, vel_right, yaw, pitch, roll,
            state_t_0 = np.concatenate((local_rot_vel, acc))

            self.state = np.concatenate((state_t_0, self.state_t_m_1, self.state_t_m_2, self.state_t_m_3))
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

        def reset(self, x=0, y=0, z=0.05, q1=0, q2=0, q3=0, q4=1, gravity=-9.81,
                  ML_R=21.5, ML_Kv=10.5, ML_Kvis=0.0005,
                  MR_R=21.5, MR_Kv=10.5, MR_Kvis=0.0005,
                  latency=0.02):
            p.resetSimulation()
            p.setGravity(0, 0, gravity)

            self.robot = p.loadURDF(os.path.join(currentdir, "balancer_randomized.urdf"), [x, y, z], [q1, q2, q3, q4],
                                    flags=p.URDF_USE_INERTIA_FROM_FILE)


            local_vel, local_rot_vel, acc = self.local_pose(2, self.sim_timestep * self.action_every_x_timestep)
            state_t_0 = np.concatenate(([0, 0], [0, 0, 0], acc, [self.max_voltage]))
            self.state_t_m_1 = state_t_0
            self.state_t_m_2 = state_t_0
            self.state_t_m_3 = state_t_0
            self.state = np.concatenate((state_t_0, self.state_t_m_1, self.state_t_m_2, self.state_t_m_3))

            ##### Environment section

            return self.state