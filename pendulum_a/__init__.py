from gym.envs.registration import register

register(
    id='PendulumA-render-v0',
    entry_point='pendulum_a:PendulumA',
    max_episode_steps=1000,
    reward_threshold=950.0,
    kwargs={'renders': True},
    tags={ "pg_complexity": 1*1000000 },
    )

register(
    id='PendulumA-v0',
    entry_point='pendulum_a:PendulumA',
    max_episode_steps=1000,
    reward_threshold=950.0,
    kwargs={'renders': False},
    tags={ "pg_complexity": 1*1000000 },
    )

import os
# Check if on Raspberry Pi
# from balboa.balboa_env import BalboaEnvMotor
try:
    if (os.uname()[4].startswith("arm")):
        # We are on the Pi
        pass

except AttributeError:
    print("We are not on a RPi")
    from pendulum_a.pendulum_a_sim_env import PendulumA