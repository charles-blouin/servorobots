import time
import gym
import pybullet as p
# p.connect(p.GUI)
timestep = 0.01

# python -m balboa.sim_env_balance_test


env = gym.make('Balboa-balance-ctrl-render-v1')
action_slider = p.addUserDebugParameter("Action", -1, 1, 0)

env.reset()
while(1):
    action = p.readUserDebugParameter(action_slider)
    time.sleep(timestep)
    obs, reward, done, _ = env.step([action, action])
    # print(obs[2])