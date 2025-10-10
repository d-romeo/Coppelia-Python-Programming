import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from coppelia_utils import draw_trajectory,get_handles, get_joint_positions, get_pose, set_joint_target_positions ,get_object_position, clear_drawing_object
from robotics_utils import forward_kinematics, forward_kinematics_panda, franka_dh_params
from robot_arm_utils import plot_panda_traj_comparison,read_file,compute_arm_error_minimization_inverse_kinematics, position_errors, plot_joint, plot_position_error, save_q_to_excel, plot_q_panda_comparison, braccio_dh_params_daniele, braccio_dh_params_chiara

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
    total_time = 700  # [s]
    iters = int(total_time/dt)

    # File parameters
    file_name = 'code/excel-files/dr/flessione_spalla_test.xlsx'
    sheet_name = 'Joint Angles ZXY'

    braccio_dh_params = braccio_dh_params_daniele


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
    q_h = read_file (file_name=file_name, sheet_name=sheet_name, percentage = 100, back=False, limits = False)
    arm_trajecotry = []             # desired trajectory 
    arm_poses = []                  # desired arm pose
    end_effector_trajectory = []    # end_effector pos in simulation
    q_arm_sim = []                  # simulation pos
    q_panda_sim = []

    # Compute the forward kinematics
    for i in range(0, q_h.shape[0]):
        T_ee_fk = forward_kinematics(braccio_dh_params_chiara, q_h[i] , q0_W) # traiettoria di chiara 
        arm_trajecotry.append((T_ee_fk)[-1][0:3, 3])
        arm_poses.append(T_ee_fk[-1])

    arm_trajecotry = np.array(arm_trajecotry)
    draw_trajectory(sim, arm_trajecotry, [255, 0, 0]) # plot the desired trajectory
        
    # Performe the inverse kinematics
    q_ik_list, end_effector_human_list = compute_arm_error_minimization_inverse_kinematics(sim, franka_joint_handles=franka_joint_handles,franka_dh_params=franka_dh_params, 
                                                                  world_T_0=world_T_0, q0_W = q0_W, q_des_list=q_h)
    
    ee_panda_fk = []           
    for i in range(0, len(q_ik_list)): 
        temp = forward_kinematics_panda(franka_dh_params, q_ik_list[i], world_T_0)[-1][0:3, 3]
        ee_panda_fk.append(temp)
    draw_trajectory(sim, np.array(ee_panda_fk), [0, 255, 0])


    # Simulation
    sim.startSimulation()
    print("[INFO] Starting simulation ...")
    for p in range(0,iters):
        if p < len(q_ik_list):
            set_joint_target_positions(sim, joint_handles=franka_joint_handles, q_des = q_ik_list[p])
            q_arm_sim.append(get_joint_positions(sim, arm_joint_handles))  
            q_panda_sim.append(get_joint_positions(sim,franka_joint_handles))                                         # save the q values of the arm in simulation  
            end_effector_trajectory.append(get_object_position(sim = sim, object_id = ee_handle, respect_to = -1))    # save the end_effector trajectory in simulation
        sim.step()
    print("[INFO] Simulation finished!")
    clear_drawing_object(sim)
    clear_drawing_object(sim)
    sim.stopSimulation()

    
    print("[INFO] Plotting...")
    # Plot of the q values simulated vs detected
    #plot_joint(q_h, q_arm_sim, dt)
    
    # Plot comparison between the q of the panda given by ik func and the simulated one
    #plot_q_panda_comparison(q_ik_list, q_panda_sim, dt)

    # Plot of the position error
    #d, e = position_errors(end_effector_traj=end_effector_trajectory, arm_traj=arm_trajecotry)
    #plot_position_error(E=e, dt = dt, d = d)

    d, e = position_errors(end_effector_traj=end_effector_human_list, arm_traj=arm_trajecotry)
    #plot_position_error(E=e, dt = dt, d = d)

    # PLot end effector traj in sim from forword kin
    #plot_panda_traj_comparison(np.array(end_effector_trajectory), np.array(ee_panda_fk), dt)

    print("[INFO] Saving q in Excel")
    save_q_to_excel(q_ik_list, "code/robot-movement/result_coppelia/result_con_robot_fake/q_ik_flessione_spalle_robot_fake.xlsx", "IK")
        
    print("[INFO] End of the program")