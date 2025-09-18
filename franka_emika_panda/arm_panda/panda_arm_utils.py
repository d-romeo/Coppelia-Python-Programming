import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from coppelia_utils import set_object_position, set_object_pose, get_pose, get_joint_positions
from robotics_utils import inverse_kinematics

"""
    Utils for the robot-arm simulation on coppelia. Work only for this simulation.
    Main: main_panda_arm.py
    Scene: panda_arm.ttt
"""

braccio_dh_params = [   
        {'a':0, 'd':0, 'alpha': -np.pi/2, 'theta':-np.pi/2}, 
        {'a':0, 'd':0, 'alpha': np.pi/2, 'theta':-np.pi/2},
        {'a':0, 'd':-0.28, 'alpha':np.pi/2, 'theta':np.pi/2},
        {'a':0.25, 'd':0, 'alpha': 0,'theta':-np.pi/2}
    ]

def read_file (file_name: str, sheet_name: str, percentage: int, back : bool) -> np.ndarray:  
    """
    percentage: percentage of the data to be used (0-100)
    """
    q_limit = np.array([0, -35, 0 , 20])  # [deg]

    if not (0 <= percentage <= 100):
        raise ValueError("percentage MUST BE in [0, 100]")

    df = pd.read_excel(file_name, sheet_name=sheet_name)
    Ntot = len(df)
    N = int(Ntot * (percentage / 100.0))
    if percentage > 0 and N == 0:
        N = 1  
    df = df.iloc[:N]  

    q0 = np.deg2rad(df['Right Shoulder Flexion/Extension'].to_numpy())
    q1 = -np.deg2rad(df['Right Shoulder Abduction/Adduction'].to_numpy())
    q2 = np.deg2rad(df['Right Shoulder Internal/External Rotation'].to_numpy())
    q3 = np.deg2rad(df['Right Elbow Flexion/Extension'].to_numpy())

    q1 = np.clip(q1, np.deg2rad(q_limit[1]), None)
    q3 = np.clip(q3, np.deg2rad(q_limit[3]), None)
    Q = np.column_stack((q0, q1, q2, q3))

    if back == True: 
        Q = np.concatenate([Q, Q[-2::-1]], axis=0) 

    return Q

""" 
Generate a fake trajectory for testing purposes 
"""
def fake_trajectory(iters: int) -> np.ndarray:
    q0 = 90 * np.pi/180 * np.ones([iters,1])
    q1 = 0 * np.pi/180 * np.ones([iters,1])
    q2 = 90 * np.pi/180 * np.ones([iters,1])
    q3 = np.linspace(90, 10, iters) * np.pi/180
    q = np.column_stack((q0, q1, q2, q3))
    
    return q

def compute_inverse_kinematics(sim, target_handle: int, franka_joint_handles: list, franka_dh_params: list, arm_trajecotry: np.array , include_orientation: bool, arm_poses : np.array, world_T_0: np.ndarray) -> np.ndarray:
    """
    include_orientation: include desired orientation, sensed usimg the sensors. 
    """
    T_arm_ef = np.array([
        [0,  0, -1, 1],
        [1,  0,  0, 1],
        [0, -1,  0, 1], 
        [0 , 0,  0, 1]
        ])
    R_arm_ef = T_arm_ef[0:3, 0:3]

    q_ik_list = []
    for i in range(0, len(arm_trajecotry)):
        set_object_position(sim, object_handle = target_handle, pos=list(arm_trajecotry[i]))
        
        # include desired orientation
        if include_orientation == True:
            desired_orientation = arm_poses[i][0:3, 0:3] @ R_arm_ef  
            T = np.eye(4)
            T[0:3, 0:3] = desired_orientation
            T[0:3, 3] = arm_trajecotry[i]
            set_object_pose(sim, target_handle, T)
        
        world_T_ee_des = get_pose(sim, target_handle, -1)
        q_start = get_joint_positions(sim, franka_joint_handles)
        success, q_ik = inverse_kinematics(dh_params=franka_dh_params, T_des=world_T_ee_des, q_first_guess=q_start,
                                       base_world_transform=world_T_0, conv_thresh=1e-3, max_iterations=1e4, 
                                       damping_factor=0.001, step_size=0.1, verbose=False)
        if not success:
            print("[WARN] IK, point did not converge to the desired solution")
        else:
            q_ik_list.append(q_ik)
    return q_ik_list


"""
Calculate the position error betweeen the end_effector traj and the desired traj
"""
def position_errors(end_effector_traj, arm_traj):
    P_ee  = end_effector_traj
    P_arm = arm_traj

    N = min(len(P_ee), len(P_arm))
    P_ee  = P_ee[:N]
    P_arm = P_arm[:N]

    E = P_ee - P_arm                # errori vettoriali per istante (N x 3)
    d = np.linalg.norm(E, axis=1)   # distanza euclidea per istante (N,)

    return d, E

"""
Plot the comparison between the desired arm angles and the simulated one
"""
def plot_joint(q, q_arm_sim, dt): 
    q_arm_sim  = np.array(q_arm_sim)* 180 / np.pi
    q_arm_real = q * 180 / np.pi
    N = min(len(q_arm_sim), len(q_arm_real))  
    time = np.arange(N) * dt  

    joint_name_plot = ["Shoulder FE", "Shoulder AA", "Shoulder IE", "Elbow FE"]

    plt.figure(figsize=(12, 8))
    for j in range(4):
        plt.subplot(2, 2, j+1)
        plt.plot(time, q_arm_real[:N, j], 'o-', label="Real", markersize=3)
        if j == 1:
            plt.plot(time, q_arm_sim[:N, j], '-', label="Sim")
        else:
            plt.plot(time, q_arm_sim[:N, j], '-', label="Sim")

        plt.title(f"Joint {j+1}: {joint_name_plot[j]}")
        plt.xlabel("Time [s]")
        plt.ylabel("Angle [rad]")
        plt.grid(True)
        plt.legend()
    plt.tight_layout()
    plt.show()

"""
Plot the position errror between the end-effector position in simulation and the desired one
"""
def plot_position_error(E,dt): 
    N = E.shape[0]
    time = np.arange(N) * dt

    fig, axs = plt.subplots(3, 1, figsize=(10, 7), sharex=True)

    axs[0].plot(time, E[:, 0], label='e_x')
    axs[0].axhline(0, color='k', lw=0.8)
    axs[0].set_ylabel('e_x')
    axs[0].grid(True)
    axs[0].legend()

    axs[1].plot(time, E[:, 1], label='e_y', color='tab:orange')
    axs[1].axhline(0, color='k', lw=0.8)
    axs[1].set_ylabel('e_y')
    axs[1].grid(True)
    axs[1].legend()

    axs[2].plot(time, E[:, 2], label='e_z', color='tab:green')
    axs[2].axhline(0, color='k', lw=0.8)
    axs[2].set_ylabel('e_z')
    axs[2].set_xlabel('Time [s]')
    axs[2].grid(True)
    axs[2].legend()

    plt.tight_layout()
    plt.show()    
