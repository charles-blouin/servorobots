# servorobots
Designed to be an extension of roboschool, servorobots contains realistic robots using RC servos.

With baselines and roboschool installed, run 

python3 run.py --alg=ppo2 --env=RoboschoolHalfCheetah-v1 --network=mlp --num_timesteps=1e5 \
--save_interval=20000 --play --num_env=32 \
--nsteps=512 \
--lr=3e-4 \
--noptepochs=15 \
--nminibatches=32

To simplify the install process, install this docker container:

https://github.com/eric-heiden/deep-rl-docker


Create a new venv with python 3.7

activate
in the root
>pip install ffmpeg gym
>pip install tensorflow==1.15rc2
>cd ..
>git clone https://github.com/openai/baselines
>cd baselines
>pip install -e .
>cd ../servorobots

