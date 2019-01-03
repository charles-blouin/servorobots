# This file loads the balancer simulation and controls it with exported weights.
# It is an important step to verify that the network is correctly built from the weights.

import tensorflow as tf
import os
import argparse
import numpy as np
import time

class AgentMLP:
    def __init__(self, weight_file, activation):
        weight_list = {}
        exec(open(weight_file).read(), weight_list)
        for x in weight_list:
            print(x)
        self.obs_space = weight_list["w0"].shape[0]
        self.act_space = weight_list["wf"].shape[1]
        self.num_hidden = weight_list["w0"].shape[1]
        # Todo: Compute automatically the number of layers.
        self.num_layer = 2


        self.obs_tf = tf.placeholder(tf.float32, [1, 8], name='obs')

        w0 = tf.constant(weight_list["w0"], dtype=tf.float32)
        b0 = tf.constant(weight_list["w0"], dtype=tf.float32)
        x = activation(tf.matmul(self.obs_tf, w0) + b0)

        w1 = tf.constant(weight_list["w1"], dtype=tf.float32)
        b1 = tf.constant(weight_list["b1"], dtype=tf.float32)
        x = activation(tf.matmul(x, w1) + b1)

        wf = tf.constant(weight_list["wf"], dtype=tf.float32)
        bf = tf.constant(weight_list["bf"], dtype=tf.float32)
        self.x = activation(tf.matmul(x, wf) + bf)

    def test_time(self):
        t0 = time.time()
        with tf.Session() as sess:
            t1 = time.time()
            sess.run(self.x, feed_dict={self.obs_tf: np.asarray([[2, 2, 2, 2, 2, 2, 2, 2]])})
            t2 = time.time()
            for i in range(0, 100):
                sess.run(self.x, feed_dict={self.obs_tf: np.asarray([[2, 2, 2, 2, 2, 2, 2, 2]])})
            t3 = time.time()
            print('Time for with session: ' + str((t1 - t0)))
            print('Time to run the first session: ' + str(t2-t1))
            print('Time to run sessions after: ' + str((t3 - t2)/100))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("weight_file",
                        help="weights file for the agent")
    parser.add_argument("layer_type",
                        help="Layer Type: relu or tanh")
    args = parser.parse_args()

    if args.layer_type == 'tanh':
        activation = tf.nn.tanh
    elif args.layer_type == 'relu':
        activation = tf.nn.relu
    else:
        print('Invalid activation type')

    agent = AgentMLP(args.weight_file, activation)
    agent.test_time()


if __name__ == '__main__':
    main()