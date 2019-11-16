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
    id='ServobulletInvertedPendulum-v0',
    entry_point='servorobots:CartPoleServoEnv',
    max_episode_steps=1000,
    reward_threshold=950.0,
    tags={ "pg_complexity": 1*1000000 },
    )

register(
    id='ServobulletInvertedPendulum-play-v0',
    entry_point='servorobots:CartPoleServoEnv',
    max_episode_steps=1000,
    reward_threshold=950.0,
    kwargs={'renders': True},
    tags={ "pg_complexity": 1*1000000 },
    )

register(
    id='RCB_quadcopter-v0',
    entry_point='servorobots:QuadcopterEnv',
    max_episode_steps=1000,
    reward_threshold=950.0,
    tags={ "pg_complexity": 1*1000000 },
    )

register(
    id='RCB_quadcopter-render-v0',
    entry_point='servorobots:QuadcopterEnv',
    max_episode_steps=1000,
    reward_threshold=950.0,
    kwargs={'renders': True},
    tags={ "pg_complexity": 1*1000000 },
    )

register(
    id='RCB_balancer-v0',
    entry_point='servorobots:BalancerEnv',
    max_episode_steps=1000,
    reward_threshold=950.0,
    tags={ "pg_complexity": 1*1000000 },
    )

register(
    id='RCB_balancer-render-v0',
    entry_point='servorobots:BalancerEnv',
    max_episode_steps=1000,
    reward_threshold=950.0,
    kwargs={'renders': True},
    tags={ "pg_complexity": 1*1000000 },
    )

register(
    id='Motor_balancer-v0',
    entry_point='servorobots:BalancerEnv',
    max_episode_steps=200,
    reward_threshold=950.0,
    kwargs={'renders': False},
    tags={ "pg_complexity": 1*1000000 },
    )
register(
    id='Motor_balancer-render-v0',
    entry_point='servorobots:BalancerEnvMotor',
    max_episode_steps=200,
    reward_threshold=950.0,
    kwargs={'renders': True},
    tags={ "pg_complexity": 1*1000000 },
    )


#from servorobots.gym_pendulums import gym_InvertedPendulum
from servorobots.cartpole_servo import CartPoleServoEnv
from servorobots.quadcopter_sim import QuadcopterEnv
from servorobots.balancer_sim import BalancerEnv
from servorobots.balancer_sim_motor import BalancerEnvMotor

