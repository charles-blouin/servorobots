from gym.envs.registration import register

register(
    id='Balboa-v0',
    entry_point='balboa:BalboaEnvMotor',
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
        from balboa.balboa_env import BalboaEnvMotor
except ImportError:
    print("Balboa environment not available, we are not on a RPi")