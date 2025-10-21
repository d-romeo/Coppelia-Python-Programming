import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from robotics_utils import forward_kinematics, forward_kinematics_panda, franka_dh_params
from robot_arm_utils import plot_panda_traj_comparison,read_file,compute_arm_error_minimization_inverse_kinematics_no_coppelia, position_errors, plot_joint, plot_position_error, save_q_to_excel, plot_q_panda_comparison, braccio_dh_params_daniele, braccio_dh_params_chiara

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
    file_name = 'code/excel-files/dr/cerchio_avanti_test.xlsx'
    sheet_name = 'Joint Angles ZXY'

    braccio_dh_params = braccio_dh_params_daniele

    #Parametri di scenario con distanza sedia 90 e altezza spalla chiara 98 e daniele 102,5
    
    #parametri chiara 
    q0_W = np.array([[ 3.81097684e-18, -1.67563409e-05,  1.00000000e+00, -4.43983579e-01],
                      [ 1.00000000e+00,  2.27434908e-13, -0.00000000e+00, -7.20000000e-02],
                      [-2.27434908e-13,  1.00000000e+00,  1.67563409e-05, 9.80007440e-01],
                      [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 1.00000000e+00]])
    
    world_T_0 = np.array([[-1.02967464e-13,  1.00000000e+00, -1.42518816e-32, -4.44000000e-01],
                          [-1.00000000e+00, -1.02967464e-13,  3.47289490e-18, 8.28000000e-01],
                          [ 3.47289490e-18,  3.71847063e-31,  1.00000000e+00, 9.00000000e-01],
                          [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 1.00000000e+00]])
    
    q_first_guess = [0.08726646259972222, 1.0471975511966667, 0.0, -1.9198621771938889, 0.0, 3.14159265359, -1.3322676295501878e-15]

    end_effector_des_start_pose = np.array([[ 0.02298116, -0.15364587,  0.9878587 ],
                                            [ 0.98590257,  0.16729199,  0.00308398],
                                            [-0.16573469,  0.97386156,  0.15532443]])
    
    # Read file
    q_h = read_file (file_name=file_name, sheet_name=sheet_name, percentage = 100, back=False, limits = False)
    arm_trajecotry = []             # desired trajectory 
    arm_poses = []                  # desired arm pose
    end_effector_trajectory = []    # end_effector pos in simulation

    # Compute the forward kinematics
    for i in range(0, q_h.shape[0]):
        T_ee_fk = forward_kinematics(braccio_dh_params_chiara, q_h[i] , q0_W) # traiettoria di chiara 
        arm_trajecotry.append((T_ee_fk)[-1][0:3, 3])
        arm_poses.append(T_ee_fk[-1])

    arm_trajecotry = np.array(arm_trajecotry)
        
    


    # Performe the inverse kinematics
    q_ik_list, end_effector_human_list = compute_arm_error_minimization_inverse_kinematics_no_coppelia(q_first_guess =  q_first_guess ,end_effector_des_start_pose = end_effector_des_start_pose, 
                                                                                                      franka_dh_params=franka_dh_params, 
                                                                                                      world_T_0=world_T_0, q0_W = q0_W, q_des_list=q_h)

    
    
    ee_panda_fk = []           
    for i in range(0, len(q_ik_list)): 
        temp = forward_kinematics_panda(franka_dh_params, q_ik_list[i], world_T_0)[-1][0:3, 3]
        ee_panda_fk.append(temp)
   


    
    


    
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
    save_q_to_excel(q_ik_list, "code/robot-movement/result_coppelia/result_con_robot_fake/q_ik_cerchio_avanti_robot_fake.xlsx", "IK")
        
    print("[INFO] End of the program")