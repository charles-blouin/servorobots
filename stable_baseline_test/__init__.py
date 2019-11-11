from gym.envs.registration import register
#from gym.scoreboard.registration import add_task, add_group
'''
register(
    id='ServorobotsInvertedPendulum-v1',
    entry_point='servorobots:gym_InvertedPendulum',
    max_episode_steps=1000,
    reward_threshold=950.0,
    tags={ "pg_complexity": 1*1000000 },
    )
'''
register(
    id='InvertedPendulum-v0',
    entry_point='stable_baseline_test:CartPoleEnv',
    max_episode_steps=1000,
    reward_threshold=950.0,
    tags={ "pg_complexity": 1*1000000 },
    kwargs={'difficulty': 0},
    )
register(
    id='InvertedPendulum-v1',
    entry_point='stable_baseline_test:CartPoleEnv',
    max_episode_steps=1000,
    reward_threshold=950.0,
    tags={ "pg_complexity": 1*1000000 },
    kwargs={'difficulty': 1},
    )

register(
    id='Pendulum-v1',
    entry_point='stable_baseline_test:PendulumEnv',
    max_episode_steps=1000,
    reward_threshold=950.0,
    tags={ "pg_complexity": 1*1000000 },
    kwargs={'difficulty': 1},
    )

from stable_baseline_test.gym_modified_pendulum import CartPoleEnv
from stable_baseline_test.upright import PendulumEnv
