import os
from zmqRemoteApi import RemoteAPIClient
from coppeliaUtils import (get_handles,set_joint_target_positions,get_camera_handle,get_visionSensorImg)
import numpy as np
import time
import cv2

# connessione con coppelia
client = RemoteAPIClient()
sim = client.getObject('sim')
client.setStepping(True)
 
joint_names = [f"/joint{i+1}" for i in range(0,7)]
joint_handles = get_handles(sim, joint_names)
camera_handle = get_camera_handle(sim,'/camera')
timestep = 0.01
t_sim = 0


sim.startSimulation()
while (t := sim.getSimulationTime()) < 1:
    img = get_visionSensorImg(sim,camera_handle)
    # Filename 
    filename = 'capture' + str(int(t*10)+218) + '.jpg'
    cv2.imwrite(filename, img) 
    cv2.imshow("full", img)     
    cv2.waitKey(1) 

    """
    joint_reference = [ np.deg2rad(45),
                        0, 
                        0,
                        np.deg2rad(-40),
                        0,
                        np.deg2rad(45),
                        0   ]
    set_joint_target_positions(sim, joint_handles, joint_reference)        
    """
    """
    joint_reference = [ np.deg2rad(45),
                        0, 
                        0,
                        np.deg2rad(-40),
                        np.deg2rad(-45),
                        np.deg2rad(45),
                        0   ]
    set_joint_target_positions(sim, joint_handles, joint_reference)
    """
    joint_reference = [ np.deg2rad(-44),
                        0, 
                        0,
                        np.deg2rad(-40),
                        np.deg2rad(45),
                        np.deg2rad(45),
                        0  ]
    set_joint_target_positions(sim, joint_handles, joint_reference)

    t_sim += timestep
    client.step()
    

    s = f'Simulation time: {t:.2f} [s]'
    print(s)
    time.sleep(timestep)

    
sim.stopSimulation()

