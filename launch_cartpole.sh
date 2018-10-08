#!/usr/bin/env bash
python3 run.py --alg=ppo2 --env=ServobulletInvertedPendulum-v0 --network=mlp2x32 --num_timesteps=2.5e7 \
--save_interval=10000 --num_env=32 \
--save_path /home/charles/Desktop/saves/save \
--nsteps=2048 \
--nminibatches=32 \
--lam=0.95 \
--gamma=0.99 \
--noptepochs=10 \
--log_interval=1 \
--lr=3e-4

