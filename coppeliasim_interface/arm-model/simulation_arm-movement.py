from typing import Tuple
from coppeliasim_interface import CoppeliaSimInterface
import numpy as np
import time
import pandas as pd
from forward_kinematics_utils import forward_kinematics


if __name__ == "__main__":

    coppelia_int = CoppeliaSimInterface(real_time=False)
    coppelia_int.start_simulation()
    joint_names = ["/q0", "/q1", "/q2", "/q3" ] 
    joint_handles = coppelia_int.get_handles(names=joint_names)
    world_ref_frame = coppelia_int.get_handles(names=["/base_reference_frame"])[0]
    ee_handle = coppelia_int.get_handles(names=["/wrist"])[0]
    world_T_0 = coppelia_int.get_pose_update(joint_handles[0], world_ref_frame)

    max_points = 1000  # drawing obj
    coppelia_int.create_drawing_object(max_points, size=5.0, color=[255, 0, 0])
                                                                    
    print("Joint handles:", joint_handles)
    braccio_dh_params = [   
    {'a':0, 'd':0, 'alpha': -np.pi/2, 'theta':-np.pi/2}, 
    {'a':0, 'd':0, 'alpha': np.pi/2, 'theta':-np.pi/2},
    {'a':0, 'd':-0.28, 'alpha':np.pi/2, 'theta':np.pi/2},
    {'a':0.25, 'd':0, 'alpha': 0,'theta':-np.pi/2}
    ]
    
    q = [0,0,0,0]
    df = pd.read_excel('code\RightArmMovement-004.xlsx', sheet_name = 'Joint Angles ZXY')
    for i in range(0,len(df)):
        q[1] = - np.pi/180*df.loc[i,'Right Shoulder Abduction/Adduction']
        q[0] = np.pi/180*df.loc[i,'Right Shoulder Flexion/Extension']
        q[2] = np.pi/180*df.loc[i,'Right Shoulder Internal/External Rotation']
        q[3] = np.pi/180*df.loc[i,'Right Elbow Flexion/Extension']


        print(f"Step {i}: q = {q}")  # Debug: stampa i valori
        coppelia_int.step()
        time.sleep(0.05)

        T_ee_fk = forward_kinematics(braccio_dh_params, q , base_world_transform=world_T_0)[-1][0:3, 3]
        world_T_ee = coppelia_int.get_pose_update(ee_handle, world_ref_frame)

        coppelia_int.set_joint_target_positions(joint_handles, q)
        coppelia_int.draw_point(T_ee_fk.tolist())

    coppelia_int.clear_drawing_object()
    
    coppelia_int.stop_simulation()

       
    

               