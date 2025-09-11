import numpy as np
from typing import Tuple



def get_handles(sim, names:list) -> list:
    handles = []
    for name in names:
        handle = sim.getObject(name)
        handles.append(handle)
    return handles

def get_joint_positions(sim, joint_handles:list) -> list:
    joint_positions = []
    for handle in joint_handles:
        pos = sim.getJointPosition(handle)
        joint_positions.append(pos)
    return joint_positions

def get_joint_velocities(sim, joint_handles:list) -> list:
    joint_velocities = []
    for handle in joint_handles:
        vel = sim.getJointVelocity(handle)
        joint_velocities.append(vel)
    return joint_velocities

def set_joint_target_positions(sim, joint_handles:list, q_des:list):
    assert len(joint_handles) == len(q_des)
    for i in range(0, len(joint_handles)):
        sim.setJointTargetPosition(joint_handles[i], q_des[i])
        

def set_joint_target_velocities(sim, joint_handles:list, dq_des:list):
    assert len(joint_handles) == len(dq_des)
    for i in range(0, len(joint_handles)):
        sim.setJointTargetVelocity(joint_handles[i], dq_des[i])

def get_object_velocity(sim, handle:int) -> np.ndarray:
    linear, angular = sim.getVelocity(handle)
    return np.array(linear+angular)    

def get_object_position(sim, object_id:int, respect_to:int) -> np.ndarray:
    position = sim.getObjectPosition(object_id, respect_to)
    return np.array(position)


"""
Function that returns the 4x4 pose matrix.
"""
def get_pose(sim, object_id:int, respect_to:int) -> np.ndarray:
    position = sim.getObjectPosition(object_id, respect_to)
    orientation = sim.getObjectOrientation(object_id, respect_to)

    matrix_list = sim.buildMatrix(position, orientation) + [0, 0, 0, 1]
    matrix = np.reshape(matrix_list, (4,4))

    return matrix

def matrix_to_euler(sim, T:np.ndarray) -> Tuple[int, int, int]:
    return sim.getEulerAnglesFromMatrix(T[0:3, 0:4].flatten().tolist())

def set_object_pose(sim, object_id:int, T:np.ndarray):
    sim.setObjectMatrix(object_id, T[0:3, 0:4].flatten().tolist())