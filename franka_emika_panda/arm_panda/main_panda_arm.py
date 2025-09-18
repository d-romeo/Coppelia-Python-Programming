import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from coppelia_utils import draw_trajectory,get_handles, get_joint_positions, get_pose, set_joint_target_positions ,get_object_position, clear_drawing_object
from robotics_utils import forward_kinematics, franka_dh_params
from robot_arm_utils import read_file,compute_inverse_kinematics, position_errors, plot_joint, plot_position_error, braccio_dh_params


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
    q = read_file (file_name=file_name, sheet_name=sheet_name, percentage=40, back=True)
    arm_trajecotry = []             # desired trajectory 
    arm_poses = []                  # desired arm pose
    end_effector_trajectory = []    # end_effector pos in simulation
    q_arm_sim = []                  # simulation pos

    # Compute the forward kinematics
    for i in range(0, q.shape[0]):
        T_ee_fk = forward_kinematics(braccio_dh_params, q[i] , q0_W)
        arm_trajecotry.append((T_ee_fk)[-1][0:3, 3])
        arm_poses.append(T_ee_fk[-1])

    arm_trajecotry = np.array(arm_trajecotry)
    draw_trajectory(sim, arm_trajecotry) # plot the desired trajectory
        
    # Performe the inverse kinematics
    q_ik_list = compute_inverse_kinematics(sim, target_handle=target_handle, franka_joint_handles=franka_joint_handles,
                                           franka_dh_params=franka_dh_params, arm_trajecotry=arm_trajecotry, 
                                           include_orientation=False, arm_poses=arm_poses, world_T_0=world_T_0)
    
    # Simulation
    sim.startSimulation()
    print("[INFO] Starting simulation ...")
    for p in range(0,iters):
        if p < len(q_ik_list):
            set_joint_target_positions(sim, joint_handles=franka_joint_handles, q_des = q_ik_list[p])
            q_arm_sim.append(get_joint_positions(sim, arm_joint_handles))                                           # save the q values of the arm in simulation  
            end_effector_trajectory.append(get_object_position(sim = sim, object_id = ee_handle, respect_to = -1))  # save the end_effector trajectory in simulation
        sim.step()
    print("[INFO] Simulation finished!")
    clear_drawing_object(sim)
    sim.stopSimulation()

    # Plot of the q values simulated vs detected
    print("[INFO] Plotting...")
    plot_joint(q, q_arm_sim, dt)
    
    # Plot of the position error
    d, e = position_errors(end_effector_traj=end_effector_trajectory, arm_traj=arm_trajecotry)
    plot_position_error(E=e, dt = dt)

    print("[INFO] End of the program")