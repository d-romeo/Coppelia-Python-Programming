import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from coppelia_utils import get_handles, get_joint_positions, get_pose, set_joint_target_positions ,get_object_position, create_drawing_object, clear_drawing_object, draw_point, set_object_position
from robotics_utils import forward_kinematics, inverse_kinematics, franka_dh_params

COPPELIA_SCENE_PATH =  "code\nikolas_ik\franka_scene.ttt" 
if __name__ == "__main__":
    print("[INFO] Starting the robot movement script...")
    client = RemoteAPIClient()
    sim = client.require('sim')    
    sim.setStepping(True)       

    # time parameters
    dt = 0.100 # [s]
    total_time = 100  # [s]
    iters = int(total_time/dt)
   
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

    base_handle = get_handles(sim = sim, names = ["/robot_base"])[0]     
    target_handle = get_handles(sim = sim, names = ["/ik_target"])[0]
    ee_handle = get_handles(sim = sim, names = ["/end_effector"])[0]
    world_T_0 = get_pose(sim = sim, object_id = base_handle, respect_to = -1)
    
    taget_position = get_object_position(sim = sim, object_id = target_handle, respect_to = -1)

    max_points = 1000  # drawing obj
    create_drawing_object(sim, max_points, size=5.0, color=[255, 0, 0])

    # Move to initial position
    q = [0,0,0,0]
    df = pd.read_excel('code\Arm-movement\RightArmMovement-004.xlsx', sheet_name = 'Joint Angles ZXY')
    q[1] = - np.pi/180*df.loc[0,'Right Shoulder Abduction/Adduction']
    q[0] = np.pi/180*df.loc[0,'Right Shoulder Flexion/Extension']
    q[2] = np.pi/180*df.loc[0,'Right Shoulder Internal/External Rotation']
    q[3] = np.pi/180*df.loc[0,'Right Elbow Flexion/Extension']
    
    T_ee_fk = forward_kinematics(braccio_dh_params, q , q0_W)[-1][0:3, 3]
    draw_point(sim , T_ee_fk.tolist())
    
    set_object_position(sim, object_handle = target_handle, pos=list(T_ee_fk))

    # Define the desired pose
    world_T_ee_des = get_pose(sim, target_handle, -1)
    q_first = get_joint_positions(sim, franka_joint_handles)
    success, q_ik_first = inverse_kinematics(dh_params=franka_dh_params, T_des=world_T_ee_des, q_first_guess=q_first,
                                       base_world_transform=world_T_0, conv_thresh=1e-3, max_iterations=1e4, 
                                       damping_factor=0.001, step_size=0.1, verbose=False)
    q_ik_list = []
    q_arm_sim = []  # simulation pos
    q_arm_real = [] # real pos from sensors

    if not success:
        print("[WARN] IK, first point did not converge to the desired solution")
    else:
        q_ik_list.append(q_ik_first)

    print("[INFO] Starting the reading procedure...")
    for i in range(1,len(df)):
        q[1] = - np.pi/180*df.loc[i,'Right Shoulder Abduction/Adduction']
        q[0] = np.pi/180*df.loc[i,'Right Shoulder Flexion/Extension']
        q[2] = np.pi/180*df.loc[i,'Right Shoulder Internal/External Rotation']
        q[3] = np.pi/180*df.loc[i,'Right Elbow Flexion/Extension']

        T_ee_fk = forward_kinematics(braccio_dh_params, q , base_world_transform=q0_W)[-1][0:3, 3] 
        set_object_position(sim, target_handle, pos=list(T_ee_fk))
        world_T_ee_des = get_pose(sim, target_handle, -1)
        draw_point(sim , T_ee_fk.tolist())        

        if i == 1:
            q_start = q_ik_first  
        else : 
            q_start = q_ik_list[-1]

        success, q_ik_temp = inverse_kinematics(dh_params=franka_dh_params, T_des=world_T_ee_des, q_first_guess=q_start,
                                                base_world_transform=world_T_0, conv_thresh=1e-3, max_iterations=1e4, 
                                                damping_factor=0.001, step_size=0.1, verbose=False)
        if not success:
            print("[WARN] IK did not converge to the desired solution")
        else:
            q_ik_list.append(q_ik_temp)
            q_arm_real.append(q.copy())

    sim.startSimulation()
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
    q_arm_sim  = np.array(q_arm_sim)
    q_arm_real = np.array(q_arm_real)
    N = min(len(q_arm_sim), len(q_arm_real))    # asse dei tempi
    time = np.arange(N) * dt  

    joint_name_plot = ["Shoulder FE", "Shoulder AA", "Shoulder IE", "Elbow FE"]

    plt.figure(figsize=(12, 8))
    for j in range(4):
        plt.subplot(2, 2, j+1)
        plt.plot(time, q_arm_real[:N, j], 'o-', label="Real", markersize=3)
        plt.plot(time, q_arm_sim[:N, j], '-', label="Sim")
        plt.title(f"Joint {j+1}: {joint_name_plot[j]}")
        plt.xlabel("Time [s]")
        plt.ylabel("Angle [rad]")
        plt.grid(True)
        plt.legend()
    plt.tight_layout()
    plt.show()

    plt.figure(figsize=(8, 6))
    plt.plot(q_arm_real[:, 0], q_arm_real[:, 1], 'o-', label="Real", markersize=4)
    plt.plot(q_arm_sim[:, 0],  q_arm_sim[:, 1],  'o-', label="Sim", markersize=4)
    plt.xlabel("q0 [rad]")
    plt.ylabel("q1 [rad]")
    plt.title("q0 vs q1 (Real vs Sim)")
    plt.grid(True)
    plt.legend()
    plt.show()
    print("[INFO] End of the program")
       