#!/usr/bin/env bash

# This is for the arguments
# -v is useful to run the same test multiple time without changing the shell script name.
# Interactive is there as a reference for future implementation. Thanks to http://linuxcommand.org/ for the tips.
action=0
version=

while [ "$1" != "" ]; do
    case $1 in
        -v | --version )        shift
                                version=$1
                                ;;
        -W | --play-weights )   action=4
                                ;;
        -w | --weights )        action=3
                                ;;
        -p | --play )           action=2
                                ;;
        -d | --draw )           action=1
                                ;;
        -h | --help )           usage
                                exit
                                ;;
        * )                     usage
                                exit 1
    esac
    shift
done

me=`basename "$0"`$version
cd ..

# Play weights
if [ $action = 4 ]
then
    python3 balancer_test.py results/${me}/weight.weights tanh

# Export weights
elif [ $action = 3 ]
then
    latest_file=$(ls results/${me}/checkpoints/ | tail -1)
    python3 servorobots/tools/export_weights.py \
     results/${me}/checkpoints/${latest_file} \
      results/${me}/weight.weights


# Play from latest checkpoint
elif [ $action = 2 ]
then
    latest_file=$(ls results/${me}/checkpoints/ | tail -1)
    echo Now playing
    python3 run.py --alg=ppo2 --env=RCB_balancer-render-v0 --network=mlp2x2 --num_timesteps=10e5 \
    --load_path results/${me}/checkpoints/${latest_file} \
    ent_coef=10 \
    --play

# Export drawing of results
elif [ $action = 1 ]
then
    python3 servorobots/tools/plot_results.py --dirs results/${me} --task_name ${me} \
    --no-show --save_dir results/${me}


# Train
else
    # Name of the current file + version
    result_dir=results/${me}

    python3 run.py --alg=ppo2 --env=RCB_balancer-v0 --network=mlp2x2 --num_timesteps=1e6 \
    --save_interval=1 --num_env=2 \
    --save_path results/${me}/save/save \
    --nsteps=1024 \
    --nminibatches=32 \
    --lam=0.95 \
    --gamma=0.99 \
    --noptepochs=10 \
    --log_interval=1 \
    --lr='lambda f: 3e-4 * f' \
    --progress_dir=results/${me}
    #--lr='lambda f: 3e-4 * f'
    #  cliprange=lambda f : f * 0.1,

    python3 servorobots/tools/plot_results.py --dirs results/${me} --task_name ${me} \
    --no-show --save_dir results/${me}

    python3 servorobots/tools/export_weights.py \
     results/${me}/save/save \
      results/${me}/weight.weights

fi
