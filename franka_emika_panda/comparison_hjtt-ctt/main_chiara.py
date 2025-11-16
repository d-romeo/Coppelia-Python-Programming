import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from coppelia_utils import draw_trajectory,get_handles, get_joint_positions, get_pose, set_joint_target_positions ,get_object_position, clear_drawing_object
from robotics_utils import forward_kinematics, forward_kinematics_panda, franka_dh_params
from robot_arm_utils import save_traj_to_excel,save_q_to_excel_arm,plot_joint, read_file, compute_arm_error_minimization_inverse_kinematics, compute_inverse_kinematics, braccio_dh_params_chiara, braccio_dh_params_daniele

""" Use different IK methods to move the robot arm along a desired trajectory. 
    The optimization problem take in account the manipulator and the arm attached to it. 
    The new function compute_arm_error_minimization_inverse_kinematics is used.
"""
if __name__ == "__main__":
    print("[INFO] Starting the robot movement script...")
    client = RemoteAPIClient()
    sim = client.require('sim')    
    sim.setStepping(True)       

    # Time parameters
    dt = 0.500 # [s]
    total_time = 400  # [s]
    iters = int(total_time/dt)

    # File parameters
    file_name = 'sim-coppelia-finali/dr-ce/flessione_spalla_test.xlsx'
    sheet_name = 'Joint Angles ZXY'

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

    # Read file
    q = read_file (file_name=file_name, sheet_name=sheet_name, percentage=100, back=False, limits = False)
    arm_trajecotry_daniele = []           # desired trajectory 
    arm_trajecotry_chiara = []
    arm_poses = []                  # desired arm pose
    q_arm_sim_hjtt= []  
    q_arm_sim_ctt= []                 # simulation pos


    for i in range(0, q.shape[0]):
        T_ee_fk = forward_kinematics(braccio_dh_params_daniele, q[i] , q0_W) # traiettoria di chiara 
        arm_trajecotry_daniele.append((T_ee_fk)[-1][0:3, 3])
        arm_poses.append(T_ee_fk[-1])

    arm_trajecotry_daniele = np.array(arm_trajecotry_daniele)
    draw_trajectory(sim, arm_trajecotry_daniele, [255, 0, 0]) # plot the desired trajectory

    # Compute the forward kinematics
    for i in range(0, q.shape[0]):
        T_ee_fk = forward_kinematics(braccio_dh_params_chiara, q[i] , q0_W) # traiettoria di chiara 
        arm_trajecotry_chiara.append((T_ee_fk)[-1][0:3, 3])
        arm_poses.append(T_ee_fk[-1])

    arm_trajecotry_chiara = np.array(arm_trajecotry_chiara)
    draw_trajectory(sim, arm_trajecotry_chiara, [0, 255, 0]) # plot the desired trajectory

    

    end_effector_trajectory = []

    # Performe the inverse kinematics
    q_ik_list_hjtt, end_effector_human_list = compute_arm_error_minimization_inverse_kinematics(sim, franka_joint_handles, franka_dh_params,  world_T_0, q0_W, q)

    q_ik_list_ctt = compute_inverse_kinematics(sim, target_handle=target_handle, franka_joint_handles=franka_joint_handles,franka_dh_params=franka_dh_params, arm_trajecotry=arm_trajecotry_daniele, include_orientation=False, arm_poses=arm_poses, world_T_0=world_T_0)
    
    hjtt = True
    if hjtt == True: 
        sim.startSimulation()
        print("[INFO] Starting simulation ...")
        for p in range(0,iters):
            if p < len(q_ik_list_hjtt):
                set_joint_target_positions(sim, joint_handles=franka_joint_handles, q_des = q_ik_list_hjtt[p])
                q_arm_sim_hjtt.append(get_joint_positions(sim, arm_joint_handles))     
                end_effector_trajectory.append(get_object_position(sim = sim, object_id = ee_handle, respect_to = -1))                                     # save the q values of the arm in simulation  
            sim.step()
        print("[INFO] Simulation finished!")
        save_traj_to_excel(
            traj=end_effector_trajectory,
            path="sim-coppelia-finali/risultati/efm/traj_efm_hjtt.xlsx"
        )
        #plot_joint(q, q_arm_sim_hjtt, dt)
        #save_q_to_excel_arm(q_arm_sim_hjtt, "sim-coppelia-finali/dr-ce/q_arm_hjtt.xlsx", "arm")
        print("[INFO] Save Complete")
        
    else:
        sim.startSimulation()
        print("[INFO] Starting simulation ...")
        for p in range(0,iters):
            if p < len(q_ik_list_ctt):
                set_joint_target_positions(sim, joint_handles=franka_joint_handles, q_des = q_ik_list_ctt[p])
                q_arm_sim_ctt.append(get_joint_positions(sim, arm_joint_handles))       
                end_effector_trajectory.append(get_object_position(sim = sim, object_id = ee_handle, respect_to = -1))                                   # save the q values of the arm in simulation  
            sim.step()
        print("[INFO] Simulation finished!")
        #plot_joint(q, q_arm_sim_ctt, dt)
        #save_q_to_excel_arm(q_arm_sim_ctt, "sim-coppelia-finali/dr-ce/q_arm_ctt.xlsx", "arm")
        
        save_traj_to_excel(
            traj=end_effector_trajectory,
            path="sim-coppelia-finali/risultati/efm/traj_efm_ctt.xlsx"
        )
        print("[INFO] Save Complete")
    sim.stopSimulation()

    