from cmath import pi
import sim
import time
import sys
import numpy

#create connection 
print("Program Started")
sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

#check connection
if(clientID != -1):
    print('Connected Successfully.')
else:
    sys.exit('Failed To connect.')
time.sleep(1)

# handles
jointHandles=[-1,-1,-1,-1,-1,-1,-1]

error_code,jointHandles[0]=sim.simxGetObjectHandle(clientID, '/Franka/joint' ,sim.simx_opmode_oneshot_wait) 
error_code,jointHandles[1]=sim.simxGetObjectHandle(clientID, '/Franka/link2_resp/joint' ,sim.simx_opmode_oneshot_wait)
error_code,jointHandles[2]=sim.simxGetObjectHandle(clientID, '/Franka/link3_resp/joint' ,sim.simx_opmode_oneshot_wait)
error_code,jointHandles[3]=sim.simxGetObjectHandle(clientID, '/Franka/link4_resp/joint' ,sim.simx_opmode_oneshot_wait)
error_code,jointHandles[4]=sim.simxGetObjectHandle(clientID, '/Franka/link5_resp/joint' ,sim.simx_opmode_oneshot_wait)
error_code,jointHandles[5]=sim.simxGetObjectHandle(clientID, '/Franka/link6_resp/joint' ,sim.simx_opmode_oneshot_wait)
error_code,jointHandles[6]=sim.simxGetObjectHandle(clientID, '/Franka/link7_resp/joint' ,sim.simx_opmode_oneshot_wait)

print(jointHandles) 




