from gym.envs.registration import register

register(
    id='WalkerA-v0',
    entry_point='walker_a:WalkerA',
    max_episode_steps=1000,
    reward_threshold=950.0,
    kwargs={'renders': True},
    tags={ "pg_complexity": 1*1000000 },
    )

import os
# Check if on Raspberry Pi
# from balboa.balboa_env import BalboaEnvMotor
try:
    if (os.uname()[4].startswith("arm")):
        # We are on the Pi
        from balboa.real_env import BalboaEnvMotor

except AttributeError:
    print("Balboa environment not available, we are not on a RPi")
    from walker_a.waker_a_sim_env import WalkerA