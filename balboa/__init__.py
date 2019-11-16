from gym.envs.registration import register

register(
    id='Balboa-v0',
    entry_point='balboa:BalboaEnvMotor',
    max_episode_steps=1000,
    reward_threshold=950.0,
    kwargs={'renders': True},
    tags={ "pg_complexity": 1*1000000 },
    )


from balboa.balboa_env import BalboaEnvMotor