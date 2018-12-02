import tensorflow as tf
from baselines.common.models import register
from baselines.a2c.utils import conv, fc, conv_to_fc, batch_to_seq, seq_to_batch
import numpy as np

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


@register("mlp2x16")
def mlp2x16(num_layers=2, num_hidden=16, activation=tf.nn.relu, layer_norm=False):
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