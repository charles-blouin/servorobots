#!/usr/bin/env bash

# This is for the arguments
# -v is useful to run the same test multiple time without changing the shell script name.
# Interactive is there as a reference for future implementation. Thanks to http://linuxcommand.org/ for the tips.
play=0
version=

while [ "$1" != "" ]; do
    case $1 in
        -v | --version )        shift
                                version=$1
                                ;;
        -p | --play )           play=2
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

if [ $play -gt 1 ]
then
    echo Now playing
    python3 run.py --alg=ppo2 --env=ServobulletInvertedPendulum-play-v0 --network=mlp2x32 --num_timesteps=5e5 \
    --load_path results/${me}/save/save \
    ent_coef=10 \
    --play
else
    # Name of the current file + version
    result_dir=results/${me}

    python3 run.py --alg=ppo2 --env=ServobulletInvertedPendulum-v0 --network=mlp2x32 --num_timesteps=6e4 \
    --save_interval=1 --num_env=1 \
    --load_path results/launch_cartpole_06.sh/save/save \
    --save_path results/${me}/save/save \
    --nsteps=2048 \
    --nminibatches=32 \
    --lam=0.95 \
    --gamma=0.99 \
    --noptepochs=10 \
    --log_interval=1 \
    --lr=3e-4 \
    --progress_dir=results/${me}


    python3 servorobots/visualize/plot_results.py --dirs results/${me} --task_name ${me} \
    --no-show --save_dir results/${me}
fi
