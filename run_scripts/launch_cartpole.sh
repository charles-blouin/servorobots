#!/usr/bin/env bash
cd ..
me=`basename "$0"`
result_dir=results/${me}

python3 run.py --alg=ppo2 --env=ServobulletInvertedPendulum-v0 --network=mlp2x32 --num_timesteps=1e5 \
--save_interval=1 --num_env=32 \
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
