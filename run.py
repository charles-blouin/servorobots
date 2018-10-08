import sys
import multiprocessing
import os.path as osp
import gym
from collections import defaultdict
import tensorflow as tf
import numpy as np

from baselines.common.vec_env.vec_frame_stack import VecFrameStack
from baselines.common.cmd_util import common_arg_parser, parse_unknown_args, make_vec_env
from baselines.common.tf_util import get_session
from baselines import bench, logger
from importlib import import_module

from baselines.common.vec_env.vec_normalize import VecNormalize
from baselines.common import atari_wrappers, retro_wrappers

import servorobots

from baselines.common.models import register
from baselines.a2c.utils import conv, fc, conv_to_fc, batch_to_seq, seq_to_batch
@register("mlp2x32")
def mlp2x32(num_layers=2, num_hidden=32, activation=tf.tanh, layer_norm=False):
    """
    Simple fully connected layer policy. Separate stacks of fully-connected layers are used for policy and value function estimation.
    More customized fully-connected policies can be obtained by using PolicyWithV class directly.
    Parameters:
    ----------
    num_layers: int                 number of fully-connected layers (default: 2)

    num_hidden: int                 size of fully-connected layers (default: 64)

    activation:                     activation function (default: tf.tanh)

    Returns:
    -------
    function that builds fully connected network with a given input placeholder
    """

    def network_fn(X):
        h = tf.layers.flatten(X)
        for i in range(num_layers):
            h = activation(fc(h, 'mlp_fc{}'.format(i), nh=num_hidden, init_scale=np.sqrt(2)))
        return h, None

    return network_fn

try:
    from mpi4py import MPI
except ImportError:
    MPI = None

try:
    import pybullet_envs
except ImportError:
    pybullet_envs = None

try:
    import roboschool
except ImportError:
    roboschool = None

_game_envs = defaultdict(set)
for env in gym.envs.registry.all():
    # TODO: solve this with regexes
    env_type = env._entry_point.split(':')[0].split('.')[-1]
    _game_envs[env_type].add(env.id)

# reading benchmark names directly from retro requires
# importing retro here, and for some reason that crashes tensorflow
# in ubuntu
_game_envs['retro'] = {
    'BubbleBobble-Nes',
    'SuperMarioBros-Nes',
    'TwinBee3PokoPokoDaimaou-Nes',
    'SpaceHarrier-Nes',
    'SonicTheHedgehog-Genesis',
    'Vectorman-Genesis',
    'FinalFight-Snes',
    'SpaceInvaders-Snes',
}


def train(args, extra_args):
    env_type, env_id = get_env_type(args.env)
    print('env_type: {}'.format(env_type))

    total_timesteps = int(args.num_timesteps)
    seed = args.seed

    learn = get_learn_function(args.alg)
    alg_kwargs = get_learn_function_defaults(args.alg, env_type)
    alg_kwargs.update(extra_args)

    env = build_env(args)

    if args.network:
        alg_kwargs['network'] = args.network
    else:
        if alg_kwargs.get('network') is None:
            alg_kwargs['network'] = get_default_network(env_type)

    print('Training {} on {}:{} with arguments \n{}'.format(args.alg, env_type, env_id, alg_kwargs))

    model = learn(
        env=env,
        seed=seed,
        total_timesteps=total_timesteps,
        ent_coef=0.0,
        **alg_kwargs
    )

    return model, env


def build_env(args):
    ncpu = multiprocessing.cpu_count()
    if sys.platform == 'darwin': ncpu //= 2
    nenv = args.num_env or ncpu
    alg = args.alg
    rank = MPI.COMM_WORLD.Get_rank() if MPI else 0
    seed = args.seed

    env_type, env_id = get_env_type(args.env)

    if env_type == 'atari':
        if alg == 'acer':
            env = make_vec_env(env_id, env_type, nenv, seed)
        elif alg == 'deepq':
            env = atari_wrappers.make_atari(env_id)
            env.seed(seed)
            env = bench.Monitor(env, logger.get_dir())
            env = atari_wrappers.wrap_deepmind(env, frame_stack=True, scale=True)
        elif alg == 'trpo_mpi':
            env = atari_wrappers.make_atari(env_id)
            env.seed(seed)
            env = bench.Monitor(env, logger.get_dir() and osp.join(logger.get_dir(), str(rank)))
            env = atari_wrappers.wrap_deepmind(env)
            # TODO check if the second seeding is necessary, and eventually remove
            env.seed(seed)
        else:
            frame_stack_size = 4
            env = VecFrameStack(make_vec_env(env_id, env_type, nenv, seed), frame_stack_size)

    elif env_type == 'retro':
        import retro
        gamestate = args.gamestate or 'Level1-1'
        env = retro_wrappers.make_retro(game=args.env, state=gamestate, max_episode_steps=10000,
                                        use_restricted_actions=retro.Actions.DISCRETE)
        env.seed(args.seed)
        env = bench.Monitor(env, logger.get_dir())
        env = retro_wrappers.wrap_deepmind_retro(env)

    else:
       get_session(tf.ConfigProto(allow_soft_placement=True,
                                   intra_op_parallelism_threads=1,
                                   inter_op_parallelism_threads=1))

       env = make_vec_env(env_id, env_type, args.num_env or 1, seed, reward_scale=args.reward_scale)

       if env_type == 'mujoco':
           env = VecNormalize(env)

    return env


def get_env_type(env_id):
    if env_id in _game_envs.keys():
        env_type = env_id
        env_id = [g for g in _game_envs[env_type]][0]
    else:
        env_type = None
        for g, e in _game_envs.items():
            if env_id in e:
                env_type = g
                break
        assert env_type is not None, 'env_id {} is not recognized in env types'.format(env_id, _game_envs.keys())

    return env_type, env_id


def get_default_network(env_type):
    if env_type == 'atari':
        return 'cnn'
    else:
        return 'mlp'

def get_alg_module(alg, submodule=None):
    submodule = submodule or alg
    try:
        # first try to import the alg module from baselines
        alg_module = import_module('.'.join(['baselines', alg, submodule]))
    except ImportError:
        # then from rl_algs
        alg_module = import_module('.'.join(['rl_' + 'algs', alg, submodule]))

    return alg_module


def get_learn_function(alg):
    return get_alg_module(alg).learn


def get_learn_function_defaults(alg, env_type):
    try:
        alg_defaults = get_alg_module(alg, 'defaults')
        kwargs = getattr(alg_defaults, env_type)()
    except (ImportError, AttributeError):
        kwargs = {}
    return kwargs



def parse_cmdline_kwargs(args):
    '''
    convert a list of '='-spaced command-line arguments to a dictionary, evaluating python objects when possible
    '''
    def parse(v):

        assert isinstance(v, str)
        try:
            return eval(v)
        except (NameError, SyntaxError):
            return v

    return {k: parse(v) for k,v in parse_unknown_args(args).items()}



def main():
    # configure logger, disable logging in child MPI processes (with rank > 0)

    arg_parser = common_arg_parser()
    args, unknown_args = arg_parser.parse_known_args()
    extra_args = parse_cmdline_kwargs(unknown_args)
    unknown_a = parse_unknown_args(unknown_args)
    print('extra_args')
    print(extra_args)
    if 'progress_dir' in extra_args:
        del extra_args['progress_dir']
    print('Deleted progress_dir. new extra_arg:')
    print(extra_args)
    print(unknown_a['progress_dir'])

    if MPI is None or MPI.COMM_WORLD.Get_rank() == 0:
        rank = 0
        logger.configure(dir='/home/charles/Desktop/progress_1')
    else:
        logger.configure(format_strs=[])
        rank = MPI.COMM_WORLD.Get_rank()

    model, env = train(args, extra_args)
    env.close()

    if args.save_path is not None and rank == 0:
        save_path = osp.expanduser(args.save_path)
        model.save(save_path)

    if args.play:
        logger.log("Running trained model")
        env = build_env(args)
        obs = env.reset()
        def initialize_placeholders(nlstm=128,**kwargs):
            return np.zeros((args.num_env or 1, 2*nlstm)), np.zeros((1))
        state, dones = initialize_placeholders(**extra_args)
        while True:
            actions, _, state, _ = model.step(obs,S=state, M=dones)
            obs, _, done, _ = env.step(actions)
            env.render()
            done = done.any() if isinstance(done, np.ndarray) else done

            if done:
                obs = env.reset()

        env.close()

if __name__ == '__main__':
    main()
