# servorobots
Designed to be an extension of roboschool, servo_robots contains realistic robots using RC servos.

With baselines installed, run 

python3 run.py --alg=ppo2 --env=RoboschoolHalfCheetah-v1 --network=mlp --num_timesteps=1e5 \
--save_interval=20000 --play --num_env=32 \
--nsteps=512 \
--lr=3e-4 \
--noptepochs=15 \
--nminibatches=32
