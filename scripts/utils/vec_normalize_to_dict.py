import os
import pickle
from vec_normalize import VecNormalize


with open(os.path.join(os.path.dirname(__file__), "vec_normalize.pkl"), 'rb') as f:    
    normalizer = pickle.load(f)

d = {}

d["obs_rms_mean"] = normalizer.obs_rms.mean
d["obs_rms_var"] = normalizer.obs_rms.var
d["state_rms_mean"] = normalizer.state_rms.mean
d["state_rms_var"] = normalizer.state_rms.var
d["clip_obs"] = normalizer.clip_obs
d["clip_state"] = normalizer.clip_state
d["epsilon"] = normalizer.epsilon


with open(os.path.join(os.path.dirname(__file__), "vec_normalize_dict.pkl"), 'wb') as f:
    pickle.dump(d, f, protocol=2)
