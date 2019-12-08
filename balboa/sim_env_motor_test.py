import gym
import time
import json

### This test is used to check the real robot vs. the simulation
# python -m balboa.sim_env_motor_check
def sim_motor_test(render=True, x=0, y=0, z=0.05, q1=0, q2=0, q3=0, q4=1, gravity = -9.81,
                    ML_R=21.5, ML_Kv=10.5, ML_Kvis=0.0005,
                    MR_R=21.5, MR_Kv=10.5, MR_Kvis=0.0005,
                    latency=0.02):

    if render:
        env = gym.make('Balboa-motors-render-v1')
    else:
        env = gym.make('Balboa-motors-v1')

    env.reset(x=x, y=y, z=z, q1=q1, q2=q2, q3=q3, q4=q4, gravity = gravity,
                    ML_R=ML_R, ML_Kv=ML_Kv, ML_Kvis=ML_Kvis,
                    MR_R=MR_R, MR_Kv=MR_Kv, MR_Kvis=MR_Kvis,
                    latency=latency)

    actions_array = []
    states_array = []
    timestamps = []
    looptime = []

    fps = 100
    frame_length = 1 / fps
    episode_length = 4  # in seconds
    start_time = 0
    time_of_this_step = 0

    for step in range(0, fps * episode_length):

        loop_time = time.time()
        # action = NN(states)

        if step < 1 * fps:
            actions = [0, 0]
        elif step < 1.5 * fps:
            actions = [0.5, 0.5]
        elif step < 2 * fps:
            actions = [1, 1]
        elif step < 2.5 * fps:
            actions = [0, 0]
        elif step < 3 * fps:
            actions = [-0.2, -0.2]
        elif step < 3.5 * fps:
            actions = [-0.4, -0.4]
        elif step < 4 * fps:
            actions = [0, 0]

        states, reward, _, info = env.step(actions)
        timestamps.append(time_of_this_step - start_time)
        states_array.append(states.tolist())
        actions_array.append(actions)
        looptime.append(time.time() - loop_time)

        time_of_this_step += frame_length

    return timestamps, actions_array, states_array, looptime;


if __name__ == "__main__":
    file_name = 'balboa_env_motor_test_01.txt'

    timestamps, actions_array, states_array, looptime = sim_motor_test()
    with open(file_name, 'w') as filehandle:
        json.dump({"timestamps": timestamps, "actions": actions_array,
                   "states": states_array
                   }, filehandle)