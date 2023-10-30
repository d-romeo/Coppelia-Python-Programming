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

    
    cv2.imshow("full", img)     
    cv2.waitKey(1)

    joint_reference = [0 + 0.1*np.sin(2*np.pi*t_sim),
                       0 + 0.1*np.sin(2*np.pi*t_sim),
                       0 + 0.1*np.sin(2*np.pi*t_sim),
                       np.deg2rad(-90) + 0.1*np.sin(2*np.pi*t_sim),
                       0 + 0.1*np.sin(2*np.pi*t_sim),
                       np.deg2rad(90) + 0.1*np.sin(2*np.pi*t_sim),
                       0 + 0.1*np.sin(2*np.pi*t_sim),
                       ]
    set_joint_target_positions(sim, joint_handles, joint_reference)    
    t_sim += timestep
    client.step()

    s = f'Simulation time: {t:.2f} [s]'
    print(s)
    time.sleep(timestep)

    
sim.stopSimulation()

