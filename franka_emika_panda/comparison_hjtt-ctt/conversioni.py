import os
import numpy as np
import pandas as pd
import scipy.io
from robot_arm_utils import read_file, read_file_mod

def xls_to_mat(xls_filename, mat_filename):
    
    # Salvataggio in .mat
    data = {
        'q0': xls_filename[:, 0],
        'q1': xls_filename[:, 1],
        'q2': xls_filename[:, 2],
        'q3': xls_filename[:, 3],
    }
    scipy.io.savemat(mat_filename, data)
    return f"Dataset salvato in {mat_filename} con successo."


reale_mat = 'sim-coppelia-finali/reale_avanti_indietro_cr.mat'
ctt_mat= 'sim-coppelia-finali/ctt_avanti_indietro_cr.mat'
hjtt_mat = 'sim-coppelia-finali/hjtt_avanti_indietro_cr.mat'

file_name_reale = 'sim-coppelia-finali/cr-de/avanti_indietro.xlsx'
file_name_ctt = 'sim-coppelia-finali/cr-de/q_arm_ctt.xlsx'
file_name_hjtt = 'sim-coppelia-finali/cr-de/q_arm_hjtt.xlsx'

desired_traj = read_file(file_name_reale, 'Joint Angles ZXY', 100, False, False)
robot_traj = read_file_mod(file_name_ctt, 'arm')
modified_traj = read_file_mod(file_name_hjtt, 'arm')

print(robot_traj)

if len(robot_traj) < len(desired_traj): 
    if len(robot_traj) < len(modified_traj): # robot is the shortest
        desired_traj = desired_traj[len(desired_traj) - len(robot_traj):, :]
        modified_traj = modified_traj[len(modified_traj) - len(robot_traj):, :]
    else: # modified is the shortest
        desired_traj = desired_traj[len(desired_traj) - len(modified_traj):, :]
        robot_traj = robot_traj[len(robot_traj) - len(modified_traj):, :]
elif len(modified_traj) < len(desired_traj): 
        desired_traj = desired_traj[len(desired_traj) - len(modified_traj):, :]
        robot_traj = robot_traj[len(robot_traj) - len(modified_traj):, :]
else: # desired is the shortest
        modified_traj = modified_traj[len(modified_traj) - len(desired_traj):, :]
        robot_traj = robot_traj[len(robot_traj) - len(desired_traj):, :] 

xls_to_mat(desired_traj, reale_mat)
xls_to_mat(robot_traj, ctt_mat)
xls_to_mat(modified_traj, hjtt_mat)


print("Conversione completata.")