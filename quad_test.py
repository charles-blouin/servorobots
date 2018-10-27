import gym
import servorobots
import time

quad = gym.make("RCB_quadcopter-render-v0")
quad.reset()

while 1:
    states, reward, done, _ = quad.step([-0.01,0,0,0])
    time.sleep(0.01)

    if done:
        quad.reset()