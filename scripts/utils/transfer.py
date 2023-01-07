import numpy as np


def transfer_to_sim_frame(eef_pos, panda):
    if panda == 'panda_1':
        relative_pos = np.array([0.37687571, 0.24069445, 0.0598593]) - np.array([-0.2, -0.25, 1.0558])

    if panda == 'panda_2':
        relative_pos = np.array([0.36575212, -0.25188181, 0.05582782]) - np.array([-0.2, 0.25, 1.0558])
    
    eef_pos = eef_pos - relative_pos

    return eef_pos


def transfer_to_real_frame(eef_pos, panda):
    if panda == 'panda_1':
        relative_pos = np.array([0.37687571, 0.24069445, 0.0598593]) - np.array([-0.2, -0.25, 1.0558])

    if panda == 'panda_2':
        relative_pos = np.array([0.36575212, -0.25188181, 0.05582782]) - np.array([-0.2, 0.25, 1.0558])
    
    eef_pos = eef_pos + relative_pos

    return eef_pos
