from roboschool.scene_abstract import SingleRobotEmptyScene
from roboschool.gym_mujoco_xml_env import RoboschoolMujocoXmlEnv
import gym, gym.spaces, gym.utils, gym.utils.seeding
import numpy as np
import os, sys


class gym_InvertedPendulum(RoboschoolMujocoXmlEnv):
    swingup = False

    def __init__(self):
        RoboschoolMujocoXmlEnv.__init__(self, 'inverted_pendulum.xml', 'cart', action_dim=1, obs_dim=5)

    def create_single_player_scene(self):
        return SingleRobotEmptyScene(gravity=9.8, timestep=0.0165, frame_skip=1)

    def robot_specific_reset(self):
        self.pole = self.parts["pole"]
        self.slider = self.jdict["slider"]
        self.j1 = self.jdict["hinge"]
        u = self.np_random.uniform(low=-.1, high=.1)
        self.j1.reset_current_position( u if not self.swingup else 3.1415+u , 0)
        self.j1.set_motor_torque(0)

    def apply_action(self, a):
        assert( np.isfinite(a).all() )
        self.slider.set_motor_torque( 100*float(np.clip(a[0], -1, +1)) )

    def calc_state(self):
        self.theta, theta_dot = self.j1.current_position()
        x, vx = self.slider.current_position()
        assert( np.isfinite(x) )
        return np.array([
            x, vx,
            np.cos(self.theta), np.sin(self.theta), theta_dot
            ])

    def _step(self, a):
        self.apply_action(a)
        self.scene.global_step()
        state = self.calc_state()  # sets self.pos_x self.pos_y
        vel_penalty = 0
        if self.swingup:
            reward = np.cos(self.theta)
            done = False
        else:
            reward = 1.0
            done = np.abs(self.theta) > .2
        self.rewards = [float(reward)]
        self.frame  += 1
        self.done   += done   # 2 == 1+True
        self.reward += sum(self.rewards)
        self.HUD(state, a, done)
        return state, sum(self.rewards), done, {}

    def camera_adjust(self):
        self.camera.move_and_look_at(0,1.2,1.0, 0,0,0.5)

class RoboschoolInvertedPendulumSwingup(gym_InvertedPendulum):
    swingup = True

