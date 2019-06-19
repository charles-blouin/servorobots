# This class is replicates the mlp type network only with tensorflow. It is useful for machines without baselines
# installed (i.e. Raspberry Pi)

import tensorflow as tf
import time
import numpy as np

class AgentMLP:
    def __init__(self, weight_file, activation):
        weight_list = {}
        exec(open(weight_file).read(), weight_list)
        # for x in weight_list:
        #    print(x)
        self.obs_space = weight_list["w0"].shape[0]
        self.act_space = weight_list["wf"].shape[1]
        self.num_hidden = weight_list["w0"].shape[1]
        # Todo: Compute automatically the number of layers.
        self.num_layer = 2


        self.obs_tf = tf.placeholder(tf.float32, [1, 8], name='obs')

        w0 = tf.constant(weight_list["w0"], dtype=tf.float32)
        b0 = tf.constant(weight_list["b0"], dtype=tf.float32)
        # [1, 8] x [8, 32] + [1, 32] = [1,32]
        self.x0 = activation(tf.matmul(self.obs_tf, w0) + b0)
        self.temp = self.obs_tf

        w1 = tf.constant(weight_list["w1"], dtype=tf.float32)
        b1 = tf.constant(weight_list["b1"], dtype=tf.float32)
        # [1, 32] x [32, 32] + [1, 32] = [1, 32]
        self.x1 = activation(tf.matmul(self.x0, w1) + b1)

        wf = tf.constant(weight_list["wf"], dtype=tf.float32)
        bf = tf.constant(weight_list["bf"], dtype=tf.float32)
        # [1, 32] x [32, 1] + [1, 1] = [1, 1]
        self.x = tf.matmul(self.x1, wf) + bf

    def act(self, sess, obs):
        action = sess.run(self.x, feed_dict={self.obs_tf: [obs]})
        return action

    def test_time(self, sess):
        t1 = time.time()
        print('here')
        # print(sess.run(self.x0, feed_dict={self.obs_tf: np.asarray([[2, 2, 2, 2, 2, 2, 2, 2]])}))

        print(sess.run(self.x, feed_dict={self.obs_tf: np.asarray([[0.0, 0.0, 3.9567904718998596e-17, 0.0, 0.0, 0.0, -3.982216160325529e-18, -9.908000000000001]])}))
        t2 = time.time()
        for i in range(0, 100):
            sess.run(self.x, feed_dict={self.obs_tf: np.asarray([[2, 2, 2, 2, 2, 2, 2, 2]])})
        t3 = time.time()
        # Conclusion: the first time the session is ran is significantly longer than the times after
        # 6 ms vs 0.3ms on desktop, 40 ms vs 1.3 ms on Raspberry Pi.
        print('Time to run the first session: ' + str(t2-t1))
        print('Time to run sessions after: ' + str((t3 - t2)/100))