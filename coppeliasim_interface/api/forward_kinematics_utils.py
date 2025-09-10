import numpy as np

# Define elementary rotation matrix around x-axis
def rotation_x(theta):
    return np.array([
        [1, 0, 0],
        [0, np.cos(theta), -np.sin(theta)],
        [0, np.sin(theta), np.cos(theta)]
    ])

# Define elementary rotation matrix around y-axis
def rotation_y(theta):
    return np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])

# Define elementary rotation matrix around z-axis
def rotation_z(theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ])

# Get the omogeneus transformation matrix A from the rotational matrix R and the translation vector d
def transformation_matrix(R, d):
    T = np.hstack((R, d.reshape(-1, 1)))  
    T = np.vstack((T, np.array([0, 0, 0, 1])))  
    return T

# Perform forward kinematics with the geometric approach
def compute_hand_position(q,ua_length,fa_length):
    R_x_s = rotation_x(-q[0])
    R_y_s = rotation_y(q[2])
    R_z_s = rotation_z(q[1])  
    R_shoulder = R_z_s @ R_x_s @ R_y_s
    #ee_position = R_shoulder @ np.array([0,-ua_length,0]) + R_shoulder @ rotation_z(q[3]) @ np.array([0,-fa_length,0])
    ee_position = R_z_s @ R_x_s @ np.array([0,-ua_length,0]) + R_shoulder @ rotation_z(q[3]) @ np.array([0,-fa_length,0])
    return ee_position

def compute_elbow_position(q,ua_length):
    R_x_s = rotation_x(-q[0])
    R_z_s = rotation_z(q[1])  
    elbow_position = np.array(R_z_s @ R_x_s @ np.array([0,-ua_length,0]))
    
    return elbow_position

def dh_matrix(theta, d, a, alpha):
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,     sa,     ca,    d],
        [0,      0,      0,    1]
    ])


def compute_hand_position_dh(q, ua_length, fa_length):
    # theta​	di	    ai​	    alpha​
    T1 = dh_matrix(q[1] -np.pi/2 , 0,       0,      np.pi/2)      # shoulder flex/ext
    T2 = dh_matrix(q[0] -np.pi/2 , 0,       0,     -np.pi/2)      # shoulder abd/add
    T3 = dh_matrix(np.pi/2 + q[2], -ua_length, 0,    np.pi/2)       # shoulder rot int/ext
    T4 = dh_matrix(q[3] -np.pi/2 , 0, fa_length,   0)             # elbow flexion
    
    T = T1 @ T2 @ T3 @ T4
    hand_position = T[0:3, 3]
    return hand_position


def compute_elbow_position_dh(q, ua_length):
    T1 = dh_matrix( q[1], 0,       0,      np.pi/2)
    T2 = dh_matrix(-q[0], 0,       0,     -np.pi/2)
    T3 = dh_matrix(q[2], 0, ua_length,    np.pi/2)
    
    T = T1 @ T2 @ T3
    elbow_position = T[0:3, 3]
    return elbow_position


def compute_hand_position_hom(q, ua_length, fa_length):
    R_x_s = rotation_x(-q[0])            
    R_y_s = rotation_y(q[2])               
    R_z_s = rotation_z(q[1])               

    R_shoulder = R_z_s @ R_x_s @ R_y_s     

    T_shoulder = transformation_matrix(R_shoulder, np.array([0, 0, 0]))
    T_upperarm = transformation_matrix(np.eye(3), np.array([0, -ua_length, 0]))
    T_elbow_flex = transformation_matrix(rotation_z(q[3]), np.array([0, 0, 0]))
    T_forearm = transformation_matrix(np.eye(3), np.array([0, -fa_length, 0]))

    T_total = T_shoulder @ T_upperarm @ T_elbow_flex @ T_forearm
    
    hand_hom = T_total @ np.array([0, 0, 0, 1])
    hand_pos = hand_hom[0:3]  # Converti da omogenea a cartesiana

    return hand_pos

def compute_elbow_position_hom(q, ua_length):
    R_x_s = rotation_x(-q[0])
    R_y_s = rotation_y(q[2])
    R_z_s = rotation_z(q[1])
    R_shoulder = R_z_s @ R_x_s @ R_y_s

    T_shoulder = transformation_matrix(R_shoulder, np.array([0, 0, 0]))
    T_upperarm = transformation_matrix(np.eye(3), np.array([0, -ua_length, 0]))

    T_elbow = T_shoulder @ T_upperarm
    elbow_hom = T_elbow @ np.array([0, 0, 0, 1])
    elbow_pos = elbow_hom[0:3]

    return elbow_pos

def forward_kinematics( dh_params:list, q:list, base_world_transform:np.ndarray=np.identity(4))->list:
    kinematic_chain = [base_world_transform]
    for i in range(0, len(dh_params)):
        a_i, d_i, alpha_i = (dh_params[i]["a"], dh_params[i]["d"], dh_params[i]["alpha"])

        theta_tot = q[i] +  dh_params[i]["theta"]
        
        c_theta_i, s_theta_i = np.cos(theta_tot), np.sin(theta_tot) 
        c_alpha_i, s_alpha_i = np.cos(alpha_i), np.sin(alpha_i)

        T_i = kinematic_chain[-1]@np.array([[c_theta_i, -s_theta_i*c_alpha_i, s_theta_i*s_alpha_i, a_i*c_theta_i],
                                            [s_theta_i, c_theta_i*c_alpha_i, -c_theta_i*s_alpha_i, a_i*s_theta_i],
                                            [0, s_alpha_i, c_alpha_i, d_i],
                                            [0, 0, 0, 1]
                                            ])
        kinematic_chain.append(T_i)
    return kinematic_chain  
