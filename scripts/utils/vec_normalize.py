import pickle
import warnings
from copy import deepcopy
from typing import Any, Dict, List, Optional, Union

import numpy as np
from typing import Tuple


class RunningMeanStd(object):
    def __init__(self, epsilon = 1e-4, shape = ()):
        """
        Calulates the running mean and std of a data stream
        https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Parallel_algorithm

        :param epsilon: helps with arithmetic issues
        :param shape: the shape of the data stream's output
        """
        self.mean = np.zeros(shape, np.float64)
        self.var = np.ones(shape, np.float64)
        self.count = epsilon

    def update(self, arr):
        batch_mean = arr
        batch_var = 0
        batch_count = 1
        self.update_from_moments(batch_mean, batch_var, batch_count)

    def update_from_moments(self, batch_mean, batch_var, batch_count):
        delta = batch_mean - self.mean
        tot_count = self.count + batch_count

        new_mean = self.mean + delta * batch_count / tot_count
        m_a = self.var * self.count
        m_b = batch_var * batch_count
        m_2 = m_a + m_b + np.square(delta) * self.count * batch_count / (self.count + batch_count)
        new_var = m_2 / (self.count + batch_count)

        new_count = batch_count + self.count

        self.mean = new_mean
        self.var = new_var
        self.count = new_count


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
        obs_shape,
        state_shape,
        clip_obs = 10.0,
        clip_state = 10.0,
        epsilon = 1e-8,
        disable = False,
    ):

        self.obs_rms = RunningMeanStd(shape=obs_shape)
        self.state_rms = RunningMeanStd(shape=state_shape)
        self.clip_obs = clip_obs
        self.clip_state = clip_state
        self.epsilon = epsilon
        self.disable = disable


    def normalize_obs(self, obs, test_mode):
        if self.disable: return obs
        obsvec = np.concatenate(obs)
        if not test_mode:
            self.obs_rms.update(obsvec)
        # Normalize observations
        obs_normalized = np.clip((obsvec - self.obs_rms.mean) / np.sqrt(self.obs_rms.var + self.epsilon), -self.clip_obs, self.clip_obs).astype(np.float32)
        return [obs_normalized[0:obs[0].shape[0]], obs_normalized[obs[0].shape[0]:]]


    def normalize_state(self, state, test_mode):
        if self.disable: return state
        if not test_mode:
            self.state_rms.update(state)
        # Normalize state
        state_normalized = np.clip((state - self.state_rms.mean) / np.sqrt(self.state_rms.var + self.epsilon), -self.clip_state, self.clip_state).astype(np.float32)
        return state_normalized

    # def reset(self) -> Union[np.ndarray, Dict[str, np.ndarray]]:
    #     """
    #     Reset all environments
    #     :return: first observation of the episode
    #     """
    #     obs = self.venv.reset()
    #     self.old_obs = obs
    #     self.returns = np.zeros(self.num_envs)
    #     if self.training:
    #         if isinstance(obs, dict) and isinstance(self.obs_rms, dict):
    #             for key in self.obs_rms.keys():
    #                 self.obs_rms[key].update(obs[key])
    #         else:
    #             self.obs_rms.update(obs)
    #     return self.normalize_obs(obs)

    @staticmethod
    def load(load_path):
        """
        Loads a saved VecNormalize object.

        :param load_path: the path to load from.
        :param venv: the VecEnv to wrap.
        :return:
        """
        with open(load_path, "rb") as file_handler:
            vec_normalize = pickle.load(file_handler)
        return vec_normalize

    def save(self, save_path):
        """
        Save current VecNormalize object with
        all running statistics and settings (e.g. clip_obs)

        :param save_path: The path to save to
        """
        with open(save_path, "wb") as file_handler:
            pickle.dump(self, file_handler)

    # @property
    # def ret(self) -> np.ndarray:
    #     warnings.warn("`VecNormalize` `ret` attribute is deprecated. Please use `returns` instead.", DeprecationWarning)
    #     return self.returns
