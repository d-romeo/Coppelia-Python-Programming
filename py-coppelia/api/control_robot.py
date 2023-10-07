from cmath import pi
import sim
import time
import sys
import numpy

#definition of gripper operation, false -> close, true -> open

def gripperOp(op): 
    if op == (False):
        sim.simxSetJointTargetVelocity(clientID,frankaGripper[1],-0.3, sim.simx_opmode_oneshot_wait)
    if op == (True):
        sim.simxSetJointTargetVelocity(clientID,frankaGripper[1], 0.3, sim.simx_opmode_oneshot_wait)
        


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

# franka handles
jointHandles=[-1,-1,-1,-1,-1,-1,-1]

error_code,jointHandles[0]=sim.simxGetObjectHandle(clientID, '/Franka/joint' ,sim.simx_opmode_oneshot_wait) 
error_code,jointHandles[1]=sim.simxGetObjectHandle(clientID, '/Franka/link2_resp/joint' ,sim.simx_opmode_oneshot_wait)
error_code,jointHandles[2]=sim.simxGetObjectHandle(clientID, '/Franka/link3_resp/joint' ,sim.simx_opmode_oneshot_wait)
error_code,jointHandles[3]=sim.simxGetObjectHandle(clientID, '/Franka/link4_resp/joint' ,sim.simx_opmode_oneshot_wait)
error_code,jointHandles[4]=sim.simxGetObjectHandle(clientID, '/Franka/link5_resp/joint' ,sim.simx_opmode_oneshot_wait)
error_code,jointHandles[5]=sim.simxGetObjectHandle(clientID, '/Franka/link6_resp/joint' ,sim.simx_opmode_oneshot_wait)
error_code,jointHandles[6]=sim.simxGetObjectHandle(clientID, '/Franka/link7_resp/joint' ,sim.simx_opmode_oneshot_wait)
print("joint Handles: ", jointHandles) 

# gripper handles
frankaGripper =sim.simxGetObjectHandle(clientID, '/Franka/FrankaGripper/openCloseJoint' ,sim.simx_opmode_oneshot_wait)
frankaCenterJoint = sim.simxGetObjectHandle(clientID, '/Franka/FrankaGripper/centerJoint' ,sim.simx_opmode_oneshot_wait)
print("gripper Handles: ", frankaGripper) 



 
"""
while (sim.simxGetLastCmdTime(clientID)/1000)<100 : 
    for i in range(7):
        sim.simxSetJointTargetPosition(clientID, jointHandles[i], numpy.sin(sim.simxGetLastCmdTime(clientID)/1000) * numpy.pi/180,sim.simx_opmode_oneshot_wait)
    print("simulation time: ", sim.simxGetLastCmdTime(clientID)/1000)

"""


gripperOp(False)
while (sim.simxGetLastCmdTime(clientID)/1000)< 20 :
    first = sim.simxGetLastCmdTime(clientID)/1000
    sim.simxSetJointTargetPosition(clientID, jointHandles[0], numpy.sin(sim.simxGetLastCmdTime(clientID)/1000) * numpy.pi/180,sim.simx_opmode_oneshot_wait)
    sim.simxSetJointTargetPosition(clientID, jointHandles[1], numpy.sin(sim.simxGetLastCmdTime(clientID)/1000) * numpy.pi/180,sim.simx_opmode_oneshot_wait)
    sim.simxSetJointTargetPosition(clientID, jointHandles[2], numpy.sin(sim.simxGetLastCmdTime(clientID)/1000) * numpy.pi/180,sim.simx_opmode_oneshot_wait)
    print("simulation time: ", sim.simxGetLastCmdTime(clientID)/1000)
    if (first == sim.simxGetLastCmdTime(clientID)/1000): 
        sys.exit('Failed To connect. PLease restart the simulation.')
        
gripperOp(True)
sys.exit('End')


    





