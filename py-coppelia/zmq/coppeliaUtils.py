import numpy as np

# handel for robot
def get_handles(sim, names:list) -> list:
    handles = []
    for name in names:
        handle = sim.getObject(name)
        handles.append(handle)
    return handles

# handle for any camera
def get_camera_handle(sim, name):
    handle = sim.getObject(name)
    return handle

def get_joint_positions(sim, joint_handles:list):
    joint_positions = []
    for handle in joint_handles:
        pos = sim.getJointPosition(handle)
        joint_positions.append(pos)
    return joint_positions

def set_joint_positions(sim, joint_handles:list, joint_positions:list):
    for i in range(len(joint_positions)) :
        pos = sim.setJointPosition(joint_handles[i], joint_positions[i])

def get_joint_velocities(sim, joint_handles:list):
    joint_velocities = []
    for handle in joint_handles:
        vel = sim.getJointVelocity(handle)
        joint_velocities.append(vel)
    return joint_velocities

def set_joint_target_velocities(sim, joint_handles:list, dq_target:list):
    for i in range(len(joint_handles)):
        sim.setJointTargetVelocity(joint_handles[i], dq_target[i])

def set_joint_target_positions(sim, joint_handles:list, q_target:list):
    for i in range(len(joint_handles)):
        sim.setJointTargetPosition(joint_handles[i], q_target[i])

def get_obstacle_min_distance(sim, links_handles:list, obstacle_handle:int):
    distances = []
    for link_h in links_handles:
        result = sim.checkDistance(link_h, obstacle_handle)
        distances.append(result[1][6])
    return min(distances)

def get_cps_static_transforms(sim, cp_handles:list, tip_handle:int):
    cp_transforms = [np.eye(4) for i in range(len(cp_handles))] # rispetto all' End Effector
    for i in range(len(cp_handles)):
        cp_pos = sim.getObjectPosition(cp_handles[i], tip_handle)
        cp_transforms[i][0:3, 3] = np.array(cp_pos)
    return cp_transforms

def get_object_pos(sim, object_handle:int):
    return sim.getObjectPosition(object_handle, -1)

def get_visionSensorImg(sim, camera_handle:int):
    #image, resolution = sim.getVisionSensorImg(camera_handle,0,0.0,[0,0],[0,0])
    img, resX, resY = sim.getVisionSensorCharImage(camera_handle)
    #uint8Numbers=sim.unpackUInt8Table(img,0,0)
    #im = np.array(uint8Numbers, dtype=np.uint8) 
    #im.resize([resolution [0], resolution[1],3])
    return np.frombuffer(img, dtype=np.uint8).reshape(resY, resX, 3)
     