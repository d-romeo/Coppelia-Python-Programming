import numpy as np

# DH parameters of the UR5 robot
franka_dh_params = [
        {'a': 0,       'alpha': 0,        'd': 0.333, 'theta': 0},  # q1
        {'a': 0,       'alpha': -np.pi/2, 'd': 0,     'theta': 0},  # q2
        {'a': 0,       'alpha':  np.pi/2, 'd': 0.316, 'theta': 0},  # q3
        {'a': 0.0825,  'alpha':  np.pi/2, 'd': 0,     'theta': 0},  # q4
        {'a': -0.0825, 'alpha': -np.pi/2, 'd': 0.384, 'theta': 0},  # q5
        {'a': 0,       'alpha':  np.pi/2, 'd': 0,     'theta': 0},  # q6
        {'a': 0.088,   'alpha':  np.pi/2, 'd': 0,     'theta': 0},  # q7
        {'a': 0,       'alpha':        0, 'd': 0.107, 'theta': 0},  # q7
    ]

def forward_kinematics( dh_params:list, q:list, base_world_transform:np.ndarray=np.identity(4))->list:
    kinematic_chain = [base_world_transform]
    for i in range(0, len(dh_params)):
        a_i, d_i, alpha_i = (dh_params[i]["a"], dh_params[i]["d"], dh_params[i]["alpha"])

        theta_tot = q[i] + dh_params[i]["theta"]
        
        c_theta_i, s_theta_i = np.cos(theta_tot), np.sin(theta_tot) 
        c_alpha_i, s_alpha_i = np.cos(alpha_i), np.sin(alpha_i)

        T_i = kinematic_chain[-1]@np.array([[c_theta_i, -s_theta_i*c_alpha_i, s_theta_i*s_alpha_i, a_i*c_theta_i],
                                            [s_theta_i, c_theta_i*c_alpha_i, -c_theta_i*s_alpha_i, a_i*s_theta_i],
                                            [0, s_alpha_i, c_alpha_i, d_i],
                                            [0, 0, 0, 1]
                                            ])
        kinematic_chain.append(T_i)
    
    return kinematic_chain   

def forward_kinematics_panda( dh_params:list, q:list, base_world_transform:np.ndarray=np.identity(4))->list:
    kinematic_chain = [base_world_transform]
    for i in range(0, len(dh_params)):
        a_i, d_i, alpha_i = (dh_params[i]["a"], dh_params[i]["d"], dh_params[i]["alpha"])
        
        if i == len(dh_params)-1:   # last joint has a different convention
            theta_tot = 0
            c_theta_i, s_theta_i = np.cos(0), np.sin(0)
        else:
            theta_tot = q[i]
            c_theta_i, s_theta_i = np.cos(theta_tot), np.sin(theta_tot)
            

        c_alpha_i, s_alpha_i = np.cos(alpha_i), np.sin(alpha_i)

        T_i = kinematic_chain[-1]@np.array([[c_theta_i, -s_theta_i, 0, a_i],
                                            [s_theta_i *c_alpha_i, c_theta_i*c_alpha_i, -s_alpha_i, -d_i*s_alpha_i],
                                            [s_theta_i * s_alpha_i, c_theta_i * s_alpha_i, c_alpha_i, d_i * c_alpha_i],
                                            [0, 0, 0, 1]
                                            ])
        kinematic_chain.append(T_i)
    
    return kinematic_chain  


def geometric_jacobian(kinematic_chain:list, q:list) -> np.ndarray:
    p_ee = kinematic_chain[-1][0:3, 3]   # Reale o aumentato

    J = np.zeros((6,len(q)))

    for i in range(0, len(q)):    # I giunti reali sono rotoidali
        z_iminus1 = kinematic_chain[i+1][0:3, 2]
        p_iminus1 = kinematic_chain[i+1][0:3, 3]
        J[0:3, i] = np.cross(z_iminus1, (p_ee-p_iminus1))
        J[3:6, i] = z_iminus1

    return J
    

def rotmat_to_angle_axis(R:np.ndarray):

    if np.allclose(R, np.eye(3), rtol=1e-3, atol=1e-3):
        return 0, np.array([1, 0, 0])
    
    elif np.isclose(np.trace(R), -1): # Check if the trace is equal to -1
        angle = np.pi
        if not np.isclose(R[2,2], -1):    # if r33 is different from -1
            axis = 1/(np.sqrt(2*(1+R[2,2]))) * np.array([R[0,2], R[1,2], 1 + R[2,2]])
            return angle, axis 

        if not np.isclose(R[1,1], -1):    # if r22 is different from -1
            axis = 1/(np.sqrt(2*(1+R[1,1]))) * np.array([R[0,1], 1+ R[1,1], R[2,1]])
            return angle, axis 
        
        if not np.isclose(R[0,0], -1):    # if r11 is different from -1
            axis = 1/(np.sqrt(2*(1+R[0,0]))) * np.array([1+R[0,0], R[1,0], R[2,0]])
            return angle, axis 
        
    else:
        angle = np.arccos(0.5*(np.trace(R)-1))
        axis_cross_prod = (1/(2*np.sin(angle)))*(R - R.T)
        axis = np.array([axis_cross_prod[2,1], axis_cross_prod[0,2], axis_cross_prod[1,0]])

        return angle, axis
    

def inverse_kinematics(dh_params:list, T_des:np.ndarray, q_first_guess:list, base_world_transform:np.ndarray, 
                       conv_thresh:float=1e-4, max_iterations:int=1e4, damping_factor:float=0.001, step_size:float=0.05, 
                       verbose:bool=True):
    b_p_des = T_des[0:3,3]
    b_R_des = T_des[0:3,0:3]

    q = np.array(q_first_guess)
    

    it = 0
    stop = False
    while not stop:
        kin_chain = forward_kinematics_panda(dh_params=dh_params, q=q, base_world_transform=base_world_transform)
        J = geometric_jacobian(kinematic_chain=kin_chain, q=q.tolist())
        b_p_curr = kin_chain[-1][0:3,3]
        b_R_curr = kin_chain[-1][0:3, 0:3]

        angle, ee_axis = rotmat_to_angle_axis(b_R_curr.T@b_R_des)   # Angle-axis error in the {ee} frame
        pose_err = np.array((b_p_des-b_p_curr).tolist()+(angle*b_R_curr@ee_axis).tolist())
        
        J_pinv = J.T @ np.linalg.inv(J@J.T + damping_factor*np.eye(J.shape[0]))
        q = q + step_size*J_pinv@pose_err 

        success = np.linalg.norm(pose_err)<conv_thresh
        stop = (it == max_iterations) or success

        if it%100 ==0 and verbose: 
            print(f"It: {it} Err:{np.linalg.norm(pose_err):.5}" )
        it += 1

    return success, q.tolist()