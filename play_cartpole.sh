#!/usr/bin/env bash
python3 run.py --alg=ppo2 --env=ServobulletInvertedPendulum-play-v0 --network=mlp2x32 --num_timesteps=5e5 \
--load_path /home/charles/Desktop/saves/save \
ent_coef=10 \
--play