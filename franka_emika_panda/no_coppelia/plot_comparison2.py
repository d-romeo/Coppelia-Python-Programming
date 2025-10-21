import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from dtaidistance import dtw
from robot_arm_utils import read_file

file_name_desired = 'code/excel-files/dr/sinistra_destra_test.xlsx'
sheet_name_desired = 'Joint Angles ZXY'

file_name_robot = 'code/robot-movement/confronto/robot/robot_fake_sinistra_destra_ce2.xlsx'
sheet_name_robot = 'Joint Angles ZXY'

file_name_modified_ik = 'code/robot-movement/confronto/modified_ik/modified_ik_sinistra_destra2_ce.xlsx'
sheet_name_modified_ik = 'Joint Angles ZXY'


desired_traj = read_file(file_name_desired, sheet_name_desired, 100, False, False)
robot_traj = read_file(file_name_robot, sheet_name_robot, 100, False, False)
modified_traj = read_file(file_name_modified_ik, sheet_name_modified_ik, 100, False, False)

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

dt = 1/60  # time step
robot_traj  = np.array(robot_traj)* 180 / np.pi
desired_traj = desired_traj * 180 / np.pi
modified_traj = modified_traj * 180 / np.pi
N = min(len(robot_traj), len(desired_traj), len(modified_traj))  
time = np.arange(N) * dt  

joint_name_plot = ["Shoulder FE", "Shoulder AA", "Shoulder IE", "Elbow FE"]

plt.figure(figsize=(12, 8))
y_min = np.min([desired_traj[:N, :4], robot_traj[:N, :4], modified_traj[:N, :4]])
y_max = np.max([desired_traj[:N, :4], robot_traj[:N, :4], modified_traj[:N, :4]])
margin = 0.05 * (y_max - y_min)  # margine 5%

for j in range(4):
    plt.subplot(2, 2, j+1)
    plt.plot(time, desired_traj[:N, j], 'o-', label="Desired", markersize=3)
    plt.plot(time, robot_traj[:N, j], '-', label="Robot")
    plt.plot(time, modified_traj[:N, j], '-', label="Modified_ik")

    plt.title(f"Joint {j+1}: {joint_name_plot[j]}")
    plt.xlabel("Time [s]")
    plt.ylabel("Angle [degree]")
    plt.grid(True)
    
    # Imposta limiti y uguali per confronto coerente
    plt.ylim(y_min - margin, y_max + margin)
    
    plt.legend()

plt.tight_layout()
plt.show()

# Calcolo errori assoluti (shape: N x 4)
err_robot = np.abs(desired_traj[:N, :] - robot_traj[:N, :])
err_modified = np.abs(desired_traj[:N, :] - modified_traj[:N, :])
# Calcolo RMS per ogni giunto
rms_robot = np.sqrt(np.mean((desired_traj[:N, :] - robot_traj[:N, :])**2, axis=0))
rms_modified = np.sqrt(np.mean((desired_traj[:N, :] - modified_traj[:N, :])**2, axis=0))

# DTW per ogni giunto
dtw_robot = [dtw.distance(desired_traj[:N, j], robot_traj[:N, j]) for j in range(4)]
dtw_modified = [dtw.distance(desired_traj[:N, j], modified_traj[:N, j]) for j in range(4)]



fig, axs = plt.subplots(2, 2, figsize=(16, 10), constrained_layout=True)

y_min = min(err_robot.min(), err_modified.min())
y_max = max(err_robot.max(), err_modified.max())
margin = 0.05 * (y_max - y_min)

for j, ax in enumerate(axs.flat):
    label_robot = f"Error: Desired vs Robot (RMS: {rms_robot[j]:.2f}, DTW: {dtw_robot[j]:.2f})"
    label_modified = f"Error: Desired vs Modified_ik (RMS: {rms_modified[j]:.2f}, DTW: {dtw_modified[j]:.2f})"
    
    ax.plot(time, err_robot[:N, j], '-', label=label_robot)
    ax.plot(time, err_modified[:N, j], '-', label=label_modified)
    
    ax.set_title(f"Joint {j+1}: {joint_name_plot[j]}")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Absolute Error [degree]")
    ax.set_ylim(y_min - margin, y_max + margin)
    ax.grid(True)
    ax.legend(fontsize='small', markerscale=0.7)

plt.show()
print("end")