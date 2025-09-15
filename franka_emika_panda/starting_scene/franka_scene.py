import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from coppelia_utils import get_handles, get_joint_positions, get_pose, set_joint_target_positions, get_object_position
from robotics_utils import forward_kinematics, inverse_kinematics, franka_dh_params, forward_kinematics_panda

COPPELIA_SCENE_PATH =  "code\nikolas_ik\franka_scene.ttt" 
if __name__ == "__main__":
    client = RemoteAPIClient()
    sim = client.require('sim') # This is the object that represents the simulation
    sim.setStepping(True)       # In this way the simulation is controlled via the step() method

    franka_joint_names = [f"/joint{i+1}" for i in range(0,7)]
    franka_joint_handles = get_handles(sim=sim,names=franka_joint_names)
    print("Franka joint handles:", franka_joint_handles)

    base_handle = get_handles(sim = sim, names = ["/robot_base"])[0]     
    target_handle = get_handles(sim = sim, names = ["/ik_target"])[0]
    ee_handle = get_handles(sim = sim, names = ["/ee_frame"])[0]
    world_T_0 = get_pose(sim = sim, object_id = base_handle, respect_to = -1)
    
    taget_position = get_object_position(sim = sim, object_id = target_handle, respect_to = -1)
    print("Target position:", taget_position)

    # Get the current joint position, useful for the initialization of the IK algorithm
    q = get_joint_positions(sim = sim, joint_handles = franka_joint_handles)

    # Define the desired pose 
    world_T_ee_des = get_pose(sim, target_handle, -1)

     # current end-effector pose from forward kinematics (usando la q corrente)
    kin_chain_curr = forward_kinematics_panda(dh_params=franka_dh_params, q=q, base_world_transform=world_T_0)
   

    # Compute the configuration associated with the desired pose with IK
    success, q_ik = inverse_kinematics(dh_params=franka_dh_params, T_des=world_T_ee_des, q_first_guess=q,
                                       base_world_transform=world_T_0, conv_thresh=1e-3, max_iterations=1e4, 
                                       damping_factor=0.001, step_size=0.1, verbose=False)
    
    if not success:
         print("[WARN] IK did not converge to the desired solution")
    else:
        print("Desidered ik (deg):", [(q_ik/np.pi)*180 for q_ik in list(q_ik)])

    print("[INFO] in loop")
    sim.startSimulation()
    
    for it in range(0, 1000):
        set_joint_target_positions(sim=sim, joint_handles=franka_joint_handles, q_des = q_ik)
        sim.step()

    sim.stopSimulation()
    print("[INFO] Simulation finished!")
