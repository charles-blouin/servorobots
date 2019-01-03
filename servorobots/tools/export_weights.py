# Thanks to peabody124 and olegklimov https://github.com/openai/roboschool/issues/44
# see also https://github.com/openai/baselines/issues/597
import joblib
import tensorflow as tf
import os

import argparse
import pdb;

def export_weight(file_name_in, file_name_out='cartpole_simple.weights'):

    sess = tf.Session()
    variables = tf.trainable_variables()

    loaded_params = joblib.load(file_name_in)
    print(loaded_params)
    print(loaded_params['ppo2_model/pi/mlp_fc1/b:0'])



    def recursive_np_array_print(a, idx):
        if len(idx)==len(a.shape)-1:
            return "[" + ",".join(["%+8.4f" % a[tuple(idx + [i])] for i in range(a.shape[len(idx)]) ]) + "]"
        else:
            return "[\n" + ",\n".join([recursive_np_array_print(a, idx + [i]) for i in range(a.shape[len(idx)]) ]) + "\n]"


    str = "import numpy as np\n"

    str +=  "w0" + " = np.array(" + recursive_np_array_print(loaded_params['ppo2_model/pi/mlp_fc0/w:0'], []) + ")\n"
    str +=  "b0" + " = np.array(" + recursive_np_array_print(loaded_params['ppo2_model/pi/mlp_fc0/b:0'], []) + ")\n"
    str +=  "w1" + " = np.array(" + recursive_np_array_print(loaded_params['ppo2_model/pi/mlp_fc1/w:0'], []) + ")\n"
    str +=  "b1" + " = np.array(" + recursive_np_array_print(loaded_params['ppo2_model/pi/mlp_fc1/b:0'], []) + ")\n"
    str +=  "wf" + " = np.array(" + recursive_np_array_print(loaded_params['ppo2_model/pi/w:0'], []) + ")\n"
    str +=  "bf" + " = np.array(" + recursive_np_array_print(loaded_params['ppo2_model/pi/b:0'], []) + ")\n"


    file = open(file_name_out, "w")
    file.write(str)
    file.close()

    print("Policy weights dumped to " + file_name_out)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("file_name_in",
                        help="display a square of a given number")
    parser.add_argument("file_name_out",  default="weights.weights")
    args = parser.parse_args()
    print(args.file_name_out)
    export_weight(args.file_name_in, args.file_name_out)

if __name__ == '__main__':
    main()

