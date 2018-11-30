import gym
import servorobots
import servorobots.network.mlp_type



env = gym.make('RCB_balancer-render-v0')
_, p = env.reset()

cmd_forward = 0
cmd_turn = 0

for _ in range(1000000):
    env.render()
    env.step(env.action_space.sample()) # take a random action

    keys = p.getKeyboardEvents()
    for k, v in keys.items():

        if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_TRIGGERED)):
            cmd_turn = 1
        if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED)):
            cmd_turn = 0
        if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_TRIGGERED)):
            cmd_turn = -1
        if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_RELEASED)):
            cmd_turn = 0

        if (k == p.B3G_UP_ARROW and (v & p.KEY_WAS_TRIGGERED)):
            cmd_forward = 1
        if (k == p.B3G_UP_ARROW and (v & p.KEY_WAS_RELEASED)):
            cmd_forward = 0.0
        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_TRIGGERED)):
            cmd_forward = -1
        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_RELEASED)):
            cmd_forward = 0.0
        # self.offset_command = self.offset_command + self.forward

    clip = lambda x, l, h: l if x < l else h if x > h else x
    cmd_right = clip(cmd_forward - cmd_turn, -1, 1)
    cmd_left = clip(cmd_forward + cmd_turn, -1, 1)

