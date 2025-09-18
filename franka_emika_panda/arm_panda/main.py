import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from coppelia_utils import draw_trajectory,get_handles, get_joint_positions, get_pose, set_joint_target_positions ,get_object_position, create_drawing_object, clear_drawing_object, draw_point, set_object_position, set_object_pose
from robotics_utils import forward_kinematics, inverse_kinematics, franka_dh_params

def read_file (file_name: str, sheet_name: str, percentage: int) -> np.ndarray:  
    """
    percentage: percentage: percentage of the data to be used (0-100)
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
    return Q

def fake_trajectory() -> np.ndarray:
    """ Generate a fake trajectory for testing purposes """
    q0 = 90 * np.pi/180 * np.ones([iters,1])
    q1 = 0 * np.pi/180 * np.ones([iters,1])
    q2 = 90 * np.pi/180 * np.ones([iters,1])
    q3 = np.linspace(90, 10, iters) * np.pi/180
    q = np.column_stack((q0, q1, q2, q3))
    
    return q

def compute_inverse_kinematics(sim, target_handle: int, franka_joint_handles: list, franka_dh_params: list, arm_trajecotry: np.array , include_orientation: bool, arm_poses : np.array, world_T_0: np.ndarray) -> np.ndarray:
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

if __name__ == "__main__":
    print("[INFO] Starting the robot movement script...")
    client = RemoteAPIClient()
    sim = client.require('sim')    
    sim.setStepping(True)       

    # Time parameters
    dt = 0.500 # [s]
    total_time = 250  # [s]
    iters = int(total_time/dt)

    # File parameters
    file_name = 'code\Arm-movement\RightArmMovement-004.xlsx'
    sheet_name = 'Joint Angles ZXY'
    
   
    braccio_dh_params = [   
        {'a':0, 'd':0, 'alpha': -np.pi/2, 'theta':-np.pi/2}, 
        {'a':0, 'd':0, 'alpha': np.pi/2, 'theta':-np.pi/2},
        {'a':0, 'd':-0.28, 'alpha':np.pi/2, 'theta':np.pi/2},
        {'a':0.25, 'd':0, 'alpha': 0,'theta':-np.pi/2}
    ]

    world_ref_frame = get_handles(sim = sim, names=["/world_frame"])[0]
    franka_joint_names = [f"/joint{i+1}" for i in range(0,7)]
    franka_joint_handles = get_handles(sim = sim, names=franka_joint_names)   
    arm_joint_names = ["/q0", "/q1", "/q2", "/q3" ] 
    arm_joint_handles = get_handles(sim = sim, names=arm_joint_names)
    arm_base_reference_frame = get_handles(sim = sim, names=["/arm_base"])[0]
    q0_W = get_pose(sim = sim, object_id = arm_joint_handles[0], respect_to = world_ref_frame)
    fake_link_two = get_handles(sim = sim, names=["/fake_link_2"])[0]
    fake_link_one = get_handles(sim = sim, names=["/fake_link_1"])[0]
    base_handle = get_handles(sim = sim, names = ["/robot_base"])[0]     
    target_handle = get_handles(sim = sim, names = ["/ik_target"])[0]
    ee_handle = get_handles(sim = sim, names = ["/end_effector"])[0]
    world_T_0 = get_pose(sim = sim, object_id = base_handle, respect_to = -1)
    
    taget_position = get_object_position(sim = sim, object_id = target_handle, respect_to = -1)

    # Read file
    q = read_file (file_name=file_name, sheet_name=sheet_name, percentage=100)
    len_q = q.shape[0]
    arm_trajecotry = []
    arm_poses = []

    for i in range(0, len_q):
        T_ee_fk = forward_kinematics(braccio_dh_params, q[i] , q0_W)
        arm_trajecotry.append((T_ee_fk)[-1][0:3, 3])
        arm_poses.append(T_ee_fk[-1])
    arm_trajecotry = np.array(arm_trajecotry)
    draw_trajectory(sim, arm_trajecotry)
        
    q_arm_sim = []  # simulation pos
    q_ik_list = compute_inverse_kinematics(sim, target_handle=target_handle, franka_joint_handles=franka_joint_handles,
                                           franka_dh_params=franka_dh_params, arm_trajecotry=arm_trajecotry, 
                                           include_orientation=False, arm_poses=arm_poses, world_T_0=world_T_0)

    sim.startSimulation()
    print("[INFO] Starting simulation ...")
    for p in range(0,iters):
        if p < len(q_ik_list):
            set_joint_target_positions(sim, joint_handles=franka_joint_handles, q_des = q_ik_list[p])
            q_arm_temp = get_joint_positions(sim, arm_joint_handles)
            q_arm_sim.append(q_arm_temp)
        sim.step()
    print("[INFO] Simulation finished!")
    clear_drawing_object(sim)
    sim.stopSimulation()

    # Define values for plotting
    print("[INFO] Plotting...")
    q_arm_sim  = np.array(q_arm_sim)* 180 / np.pi
    q_arm_real = q * 180 / np.pi
    N = min(len(q_arm_sim), len(q_arm_real))    # asse dei tempi
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
    pos_fake_link_one = np.array(pos_fake_link_one)
    pos_fake_link_two = np.array(pos_fake_link_two)

    pos_one_x = pos_fake_link_one[:, 0, 3]
    pos_one_y = pos_fake_link_one[:, 1, 3]
    pos_one_z = pos_fake_link_one[:, 2, 3]

    pos_two_x = pos_fake_link_two[:, 0, 3]
    pos_two_y = pos_fake_link_two[:, 1, 3]
    pos_two_z = pos_fake_link_two[:, 2, 3]

    N = min(len(pos_one_x), len(pos_two_x))    # asse dei tempi
    time = np.arange(N) * dt

    plt.figure(figsize=(10, 6))

    # subplot 1
    plt.subplot(2, 1, 1)
    plt.plot(time, pos_one_x, 'b-', label="Link one X")
    plt.xlabel("Time [s]")
    plt.ylabel("Position X [m]")
    plt.title("Fake link one - X position")
    plt.grid(True)
    plt.legend()

    # subplot 2
    plt.subplot(2, 1, 2)
    plt.plot(time, pos_two_x, 'r-', label="Link two X")
    plt.xlabel("Time [s]")
    plt.ylabel("Position X [m]")
    plt.title("Fake link two - X position")
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.show()
    
    """

    """
        plt.figure(figsize=(8, 6))
        plt.plot(q_arm_real[:, 0], q_arm_real[:, 1], 'o-', label="Real", markersize=4)
        plt.plot(q_arm_sim[:, 0],  q_arm_sim[:, 1],  'o-', label="Sim", markersize=4)
        plt.xlabel("q0 [rad]")
        plt.ylabel("q1 [rad]")
        plt.title("q0 vs q1 (Real vs Sim)")
        plt.grid(True)
        plt.legend()
        plt.show()
    """
    
    print("[INFO] End of the program")
       
    

               