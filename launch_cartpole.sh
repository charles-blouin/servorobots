#!/usr/bin/env bash
python3 run.py --alg=ppo2 --env=ServobulletInvertedPendulum-v0 --network=mlp --num_timesteps=5e5 \
--save_interval=20000 --num_env=32 \
--save_path /home/charles/Desktop/saves/save \
--nsteps=512 \
--lr=1e-4 \
--noptepochs=15 \
--nminibatches=32
