# 此文件在 python3 环境下运行
import os
import pickle
from vec_normalize import VecNormalize

# 读取 python3 的对象
relative_path = "actor/parameters/dircrete/"
with open(os.path.join(os.path.dirname(os.path.dirname(__file__)), relative_path+"vec_normalize.pkl"), 'rb') as f:    
    normalizer = pickle.load(f)

d = {}

d["obs_rms_mean"] = normalizer.obs_rms.mean
d["obs_rms_var"] = normalizer.obs_rms.var
d["state_rms_mean"] = normalizer.state_rms.mean
d["state_rms_var"] = normalizer.state_rms.var
d["clip_obs"] = normalizer.clip_obs
d["clip_state"] = normalizer.clip_state
d["epsilon"] = normalizer.epsilon

# 提取对象中的关键数据并保存
with open(os.path.join(os.path.dirname(os.path.dirname(__file__)), relative_path+"vec_normalize_dict.pkl"), 'wb') as f:
    pickle.dump(d, f, protocol=2)
