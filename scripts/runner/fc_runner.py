import os
from actor.fc_controller import Controller
import numpy as np
import pandas as pd
from utils.my_vec_normalize import VecNormalize
from prettytable import PrettyTable

class Runner:

    def __init__(self):
        self.record = False
        self.steps = 0
        self.data_path = os.path.join(os.path.dirname(__file__), "data/state_check.csv")
        self.vn = VecNormalize.load(os.path.join(os.path.dirname(__file__), "actor/parameters/vec_normalize_dict.pkl"))
        self.controller = Controller()

    def step(self, obs, avail_actions):
        transition = {
            "avail_actions": avail_actions,
            "obs": self.vn.normalize_obs(obs)
        }
        actions = self.controller.forward(transition)

        action_table = PrettyTable(['Pactx', 'Pacty', 'Pactz', 'Qactx', 'Qacty', 'Qactz'])
        action_table.add_row([actions[0][0], actions[0][1], actions[0][2], actions[1][0], actions[1][1], actions[1][2]])
        print(action_table)

        if self.record:
            # record obs and actions
            dataframe = pd.DataFrame({
                'Pjoint_pos_1':obs[0][0], 'Pjoint_pos_2':obs[0][1], 'Pjoint_pos_3':obs[0][2], 'Pjoint_pos_4':obs[0][3], 'Pjoint_pos_5':obs[0][4], 'Pjoint_pos_6':obs[0][5], 'Pjoint_pos_7':obs[0][6],
                'Qjoint_pos_1':obs[1][0], 'Qjoint_pos_2':obs[1][1], 'Qjoint_pos_3':obs[1][2], 'Qjoint_pos_4':obs[1][3], 'Qjoint_pos_5':obs[1][4], 'Qjoint_pos_6':obs[1][5], 'Qjoint_pos_7':obs[1][6],
                'Pjoint_vel_1':obs[0][7], 'Pjoint_vel_2':obs[0][8], 'Pjoint_vel_3':obs[0][9], 'Pjoint_vel_4':obs[0][10], 'Pjoint_vel_5':obs[0][11], 'Pjoint_vel_6':obs[0][12], 'Pjoint_vel_7':obs[0][13],
                'Qjoint_vel_1':obs[1][7], 'Qjoint_vel_2':obs[1][8], 'Qjoint_vel_3':obs[1][9], 'Qjoint_vel_4':obs[1][10], 'Qjoint_vel_5':obs[1][11], 'Qjoint_vel_6':obs[1][12], 'Qjoint_vel_7':obs[1][13],
                'Peef_pos_1':obs[0][14], 'Peef_pos_2':obs[0][15], 'Peef_pos_3':obs[0][16],
                'Qeef_pos_1':obs[1][14], 'Qeef_pos_2':obs[1][15], 'Qeef_pos_3':obs[1][16],
                'Pquaternion_x':obs[0][17], 'Pquaternion_y':obs[0][18], 'Pquaternion_z':obs[0][19], 'Pquaternion_w':obs[0][20], 
                'Qquaternion_x':obs[1][17], 'Qquaternion_y':obs[1][18], 'Qquaternion_z':obs[1][19], 'Qquaternion_w':obs[1][20], 
                'Pft_1':obs[0][21], 'Pft_2':obs[0][22], 'Pft_3':obs[0][23], 'Pft_4':obs[0][24], 'Pft_5':obs[0][25], 'Pft_6':obs[0][26], 
                'Qft_1':obs[1][21], 'Qft_2':obs[1][22], 'Qft_3':obs[1][23], 'Qft_4':obs[1][24], 'Qft_5':obs[1][25], 'Qft_6':obs[1][26], 
                'Ppeg_pos_1':obs[0][27], 'Ppeg_pos_2':obs[0][28], 'Ppeg_pos_3':obs[0][29], 
                'Qpeg_pos_1':obs[1][27], 'Qpeg_pos_2':obs[1][28], 'Qpeg_pos_3':obs[1][29], 
                'Ppeg_to_hole_1':obs[0][30], 'Ppeg_to_hole_2':obs[0][31], 'Ppeg_to_hole_3':obs[0][32],
                'Qpeg_to_hole_1':obs[1][30], 'Qpeg_to_hole_2':obs[1][31], 'Qpeg_to_hole_3':obs[1][32],
                'Pactx':actions[0][0], 'Pacty':actions[0][1], 'Pactz':actions[0][2],
                'Qactx':actions[1][0], 'Qacty':actions[1][1], 'Qactz':actions[1][2]
            }, index=[self.steps])

            if self.steps:
                dataframe.to_csv(self.data_path, mode='a', header=False, columns=['Pjoint_pos_1', 'Pjoint_pos_2', 'Pjoint_pos_3', 'Pjoint_pos_4', 'Pjoint_pos_5', 'Pjoint_pos_6', 'Pjoint_pos_7',
                                                                                'Qjoint_pos_1', 'Qjoint_pos_2', 'Qjoint_pos_3', 'Qjoint_pos_4', 'Qjoint_pos_5', 'Qjoint_pos_6', 'Qjoint_pos_7',
                                                                                'Pjoint_vel_1', 'Pjoint_vel_2', 'Pjoint_vel_3', 'Pjoint_vel_4', 'Pjoint_vel_5', 'Pjoint_vel_6',  'Pjoint_vel_6',
                                                                                'Qjoint_vel_1', 'Qjoint_vel_2', 'Qjoint_vel_3', 'Qjoint_vel_4', 'Qjoint_vel_5', 'Qjoint_vel_6',  'Qjoint_vel_6',
                                                                                'Peef_pos_1', 'Peef_pos_2', 'Peef_pos_3',
                                                                                'Qeef_pos_1', 'Qeef_pos_2', 'Qeef_pos_3',
                                                                                'Pquaternion_x', 'Pquaternion_y', 'Pquaternion_z', 'Pquaternion_w', 
                                                                                'Qquaternion_x', 'Qquaternion_y', 'Qquaternion_z', 'Qquaternion_w', 
                                                                                'Pft_1', 'Pft_2', 'Pft_3', 'Pft_4', 'Pft_5', 'Pft_6', 
                                                                                'Qft_1', 'Qft_2', 'Qft_3', 'Qft_4', 'Qft_5', 'Qft_6', 
                                                                                'Ppeg_pos_1', 'Ppeg_pos_2', 'Ppeg_pos_3', 
                                                                                'Qpeg_pos_1', 'Qpeg_pos_2', 'Qpeg_pos_3', 
                                                                                'Ppeg_to_hole_1', 'Ppeg_to_hole_2', 'Ppeg_to_hole_3',
                                                                                'Qpeg_to_hole_1', 'Qpeg_to_hole_2', 'Qpeg_to_hole_3',
                                                                                'Pactx', 'Pacty', 'Pactz',
                                                                                'Qactx', 'Qacty', 'Qactz'])
            else:
                dataframe.to_csv(self.data_path, columns=['Pjoint_pos_1', 'Pjoint_pos_2', 'Pjoint_pos_3', 'Pjoint_pos_4', 'Pjoint_pos_5', 'Pjoint_pos_6', 'Pjoint_pos_7',
                                                        'Qjoint_pos_1', 'Qjoint_pos_2', 'Qjoint_pos_3', 'Qjoint_pos_4', 'Qjoint_pos_5', 'Qjoint_pos_6', 'Qjoint_pos_7',
                                                        'Pjoint_vel_1', 'Pjoint_vel_2', 'Pjoint_vel_3', 'Pjoint_vel_4', 'Pjoint_vel_5', 'Pjoint_vel_6',  'Pjoint_vel_6',
                                                        'Qjoint_vel_1', 'Qjoint_vel_2', 'Qjoint_vel_3', 'Qjoint_vel_4', 'Qjoint_vel_5', 'Qjoint_vel_6',  'Qjoint_vel_6',
                                                        'Peef_pos_1', 'Peef_pos_2', 'Peef_pos_3',
                                                        'Qeef_pos_1', 'Qeef_pos_2', 'Qeef_pos_3',
                                                        'Pquaternion_x', 'Pquaternion_y', 'Pquaternion_z', 'Pquaternion_w', 
                                                        'Qquaternion_x', 'Qquaternion_y', 'Qquaternion_z', 'Qquaternion_w', 
                                                        'Pft_1', 'Pft_2', 'Pft_3', 'Pft_4', 'Pft_5', 'Pft_6', 
                                                        'Qft_1', 'Qft_2', 'Qft_3', 'Qft_4', 'Qft_5', 'Qft_6', 
                                                        'Ppeg_pos_1', 'Ppeg_pos_2', 'Ppeg_pos_3', 
                                                        'Qpeg_pos_1', 'Qpeg_pos_2', 'Qpeg_pos_3', 
                                                        'Ppeg_to_hole_1', 'Ppeg_to_hole_2', 'Ppeg_to_hole_3',
                                                        'Qpeg_to_hole_1', 'Qpeg_to_hole_2', 'Qpeg_to_hole_3',
                                                        'Pactx', 'Pacty', 'Pactz',
                                                        'Qactx', 'Qacty', 'Qactz'])

        self.steps += 1

        return actions