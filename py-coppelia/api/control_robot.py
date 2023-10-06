from cmath import pi
import sim
import time
import sys
import numpy

#create connection 
print("Program Started")
sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

#check connection
if(clientID != -1):
    print('Connected Successfully.')
else:
    sys.exit('Failed To connect.')
time.sleep(1)


# handles
jointHandles={-1,-1,-1,-1,-1,-1,-1}
for i in range(7):        
    jointHandles[i]=sim.simxGetObjectHandle('./joint', 0)







