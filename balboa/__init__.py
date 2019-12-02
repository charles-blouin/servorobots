from gym.envs.registration import register

register(
    id='Balboa-v0',
    entry_point='balboa:BalboaEnvMotor',
    max_episode_steps=1000,
    reward_threshold=950.0,
    kwargs={'renders': True, 'offset_gx': 0.0069169671877200006, 'offset_gy': -0.05740454241144015, 'offset_gz': -0.06578842224624006},
    tags={ "pg_complexity": 1*1000000 },
    )

register(
    id='Balboa-motors-render-v1',
    entry_point='balboa:BalboaEnvSimMotor',
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
    from balboa.sim_env_motor import BalboaEnvSimMotor