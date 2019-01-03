# This file loads the balancer simulation and controls it with exported weights.
# It is an important step to verify that the network is correctly built from the weights.

import tensorflow as tf
import os
import argparse

class AgentMLP:
    def __init__(self, weight_file):
        weight_list = {}
        exec(open(weight_file).read(), weight_list)
        for x in weight_list:
            print(x)
        self.obs_space = weight_list["w0"].shape[0]
        self.act_space = weight_list["wf"].shape[1]
        self.num_hidden = weight_list["w0"].shape[1]
        # Todo: Compute automatically the number of layers.
        self.num_layer = 2



def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("weight_file",
                        help="weights file for the agent")
    parser.add_argument("layer_type",
                        help="Layer Type: relu or tanh")
    args = parser.parse_args()

    agent = AgentMLP(args.weight_file)


if __name__ == '__main__':
    main()