# This file loads the balancer simulation and controls it with exported weights.
# It is an important step to verify that the network is correctly built from the weights.

import tensorflow as tf
import argparse
import gym

import servorobots
# from servorobots.network.agent_mlp import AgentMLP


def main():


    balancer = gym.make("RCB_balancer-render-v0")
    states = balancer.reset()


    while 1:
        for i in range(0, 500):
            action = balancer.action_space
            states, reward, done, _ = balancer.step([1])
        action = balancer.reset()


if __name__ == '__main__':
    main()

    # parser = argparse.ArgumentParser()
    # parser.add_argument("weight_file",
    #                     help="weights file for the agent")
    # parser.add_argument("layer_type",
    #                     help="Layer Type: relu or tanh")
    # args = parser.parse_args()
    #
    # if args.layer_type == 'tanh':
    #     activation = tf.nn.tanh
    # elif args.layer_type == 'relu':
    #     activation = tf.nn.relu
    # else:
    #     print('Invalid activation type')
    #
    # agent = AgentMLP(args.weight_file, activation)

    # with tf.Session() as sess:
    #     agent.test_time(sess)