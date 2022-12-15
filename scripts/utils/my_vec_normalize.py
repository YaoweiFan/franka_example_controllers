import pickle
import warnings
from copy import deepcopy
from typing import Any, Dict, List, Optional, Union

import numpy as np
from typing import Tuple

import yaml


class RunningMeanStd(object):
    def __init__(self, mean, var):
        """
        Calulates the running mean and std of a data stream
        https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Parallel_algorithm

        :param epsilon: helps with arithmetic issues
        :param shape: the shape of the data stream's output
        """
        self.mean = mean
        self.var = var

class VecNormalize:
    """
    A moving average, normalizing wrapper for vectorized environment.
    has support for saving/loading moving average,

    :param venv: the vectorized environment to wrap
    :param training: Whether to update or not the moving average
    :param norm_obs: Whether to normalize observation or not (default: True)
    :param norm_reward: Whether to normalize rewards or not (default: True)
    :param clip_obs: Max absolute value for observation
    :param clip_reward: Max value absolute for discounted reward
    :param gamma: discount factor
    :param epsilon: To avoid division by zero
    :param norm_obs_keys: Which keys from observation dict to normalize.
        If not specified, all keys will be normalized.
    """

    def __init__(
        self,
        obs_rms_mean,
        obs_rms_var,
        state_rms_mean,
        state_rms_var,
        clip_obs,
        clip_state,
        epsilon
    ):
        self.obs_rms = RunningMeanStd(obs_rms_mean, obs_rms_var)
        self.state_rms = RunningMeanStd(state_rms_mean, state_rms_var)
        self.clip_obs = clip_obs
        self.clip_state = clip_state
        self.epsilon = epsilon


    def normalize_obs(self, obs):
        obsvec = np.concatenate(obs)
        # Normalize observations
        obs_normalized = np.clip((obsvec - self.obs_rms.mean) / np.sqrt(self.obs_rms.var + self.epsilon), -self.clip_obs, self.clip_obs).astype(np.float32)
        return [obs_normalized[0:obs[0].shape[0]], obs_normalized[obs[0].shape[0]:]]


    def normalize_state(self, state):
        # Normalize state
        state_normalized = np.clip((state - self.state_rms.mean) / np.sqrt(self.state_rms.var + self.epsilon), -self.clip_state, self.clip_state).astype(np.float32)
        return state_normalized


    @staticmethod
    def load(load_path):
        """
        Loads a saved VecNormalize object.

        :param load_path: the path to load from.
        :param venv: the VecEnv to wrap.
        :return:
        """
        with open(load_path, "rb") as f:
            d = pickle.load(f)
        vec_normalize = VecNormalize(d["obs_rms_mean"], d["obs_rms_var"],
                                     d["state_rms_mean"], d["state_rms_var"],
                                     d["clip_obs"], d["clip_state"], d["epsilon"])
        return vec_normalize

