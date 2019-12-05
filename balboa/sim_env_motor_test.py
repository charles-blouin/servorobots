import gym
import time
import json

# python -m balboa.sim_env_motor_test
def sim_motor_test():
    env = gym.make('Balboa-motors-render-v1')
    env.reset()

    actions_array = []
    states_array = []
    timestamps = []
    looptime = []

    fps = 100
    frame_length = 1 / fps
    episode_length = 4  # in seconds

    states = env.reset()
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