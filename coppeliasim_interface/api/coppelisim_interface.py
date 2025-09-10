from typing import Tuple
import numpy as np
from coppeliasim_zmqremoteapi_client  import RemoteAPIClient 

class CoppeliaSimInterface:
    def __init__(self, real_time:bool=False):
        self.client = RemoteAPIClient()
        self.sim = self.client.require("sim")
        self.simIK = self.client.require('simIK')
        self.client.setStepping(not real_time)
        self.drawing_object_handle = None 
    
    def start_simulation(self):
        self.sim.startSimulation()

    def stop_simulation(self):
        self.sim.stopSimulation()

    def step(self):
        self.client.step()

    def get_simulation_time(self):
        return self.sim.getSimulationTime()
    
    def get_simulation_timestep(self) -> float:
        return self.sim.getSimulationTimeStep()

    def create_drawing_object(self, max_items:int, size:float=10.0, color:list=[255,0,0]):
        self.drawing_object_handle = self.sim.addDrawingObject(self.sim.drawing_points,
                                                        size, 0, self.sim.handle_world, max_items, 
                                                        color)
    def clear_drawing_object(self):
        if self.drawing_object_handle != None:
            self.sim.removeDrawingObject(self.drawing_object_handle)
            self.drawing_object_handle = None
    
    def draw_point(self, coord_world:list):
        """
        Draw a point in the simulator
        :param: coord_world -> position of the point to draw wrt to the world frame 
        """
        if self.drawing_object_handle == None:
            raise Exception("[ERROR] Drawing object not initialized.")
        self.sim.addDrawingObjectItem(self.drawing_object_handle, coord_world)

    def get_object_pose_matrix(self, object_a_handle:int, object_b_handle:int=-1):
        """
        Calculate the homogeneous transformation between object_A and object_B.
        :param: object_a_handle -> handle of object_a
        :param: object_b_handle -> handle of object_b. Default is world handle (-1)
        :return: b_T_a the 4x4 matrix which relates the pose of object_a wrt object_b
        """
        tmp = np.array(self.sim.getObjectMatrix(object_a_handle, object_b_handle)).reshape((3,4))
        b_T_a = np.vstack([tmp, np.array([[0,0,0,1]])])
        return b_T_a

    def get_handles(self, names:list) -> list:
        """
        Retrieves the handles associated to the object names. 
        :param: names -> list conaining the names of the objects of interest
        :return: handles -> list containing the handles (integer numbers)
        """
        if len(names) == 0:
            raise Exception("[ERROR] The list of the object names is empty.")
        handles = []
        for name in names:
            handle = self.sim.getObject(name)
            handles.append(handle)
        return handles

    def get_joint_positions(self, joint_handles:list) -> np.ndarray:
        """
        Retrieves the joint position given certain handles. 
        :param: joint_handles -> list conaining the handles ofthe joints
        :return: joint_positions -> numpy array containing the robot joint positions
        """
        if len(joint_handles) == 0:
            raise Exception("[ERROR] The list of the joint handles is empty.")
        joint_positions = []
        for handle in joint_handles:
            pos = self.sim.getJointPosition(handle)
            joint_positions.append(pos)
        return np.array(joint_positions)
    

    def get_joint_velocities(self, joint_handles:list) -> np.ndarray:
        """
        Retrieves the joint velocities given certain handles. 
        :param: joint_handles -> list conaining the handles ofthe joints
        :return: joint_velocities -> numpy array containing the robot joint velocities
        """
        if len(joint_handles) == 0:
            raise Exception("[ERROR] The list of the joint handles is empty.")
        joint_velocities = []
        for handle in joint_handles:
            pos = self.sim.getJointVelocity(handle)
            joint_velocities.append(pos)
        return np.array(joint_velocities)
    

    def set_joint_target_velocities(self, joint_handles:list, target_velocities:list):
        """
        Set a desired velocity for the joint. 
        :param: joint_handles -> list containing the handles ofthe joints
        :param: target_velocities -> list containing the target for the joint velocities
        
        """
        if len(joint_handles) == 0:
            raise Exception("[ERROR] The list of the joint handles is empty.")
        if len(joint_handles) != len(target_velocities):
            raise Exception("[ERROR] joint handles and target velocity lists have incoherent dimensions.")
        for i in range(len(joint_handles)):
            self.sim.setJointTargetVelocity(joint_handles[i], target_velocities[i])


    def set_joint_target_positions(self, joint_handles: list, target_positions: list):
        """
        Set a desired position for the joints. 
        :param: joint_handles -> list containing the handles of the joints
        :param: target_positions -> list containing the target positions for the joints
        """
        if len(joint_handles) == 0:
            raise Exception("[ERROR] The list of the joint handles is empty.")
        if len(joint_handles) != len(target_positions):
            raise Exception("[ERROR] joint handles and target positions lists have incoherent dimensions.")
        for i in range(len(joint_handles)):
            self.sim.setJointTargetPosition(joint_handles[i], target_positions[i])

    def set_object_position(self, object_handle:int, pos:list):
        self.sim.setObjectPosition(object_handle, pos, self.sim.handle_world)

    def get_pose_update(self, object_id:int, respect_to:int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Function that returns the rotation matrix and the position vector of an object w.r.t. the world 
        reference frame.
        """
        position = self.sim.getObjectPosition(object_id, respect_to)
        orientation = self.sim.getObjectOrientation(object_id, respect_to)

        matrix_list = self.sim.buildMatrix(position, orientation) + [0, 0, 0, 1]
        matrix = np.reshape(matrix_list, (4,4))

        return matrix

    def enable_vision_sensor(self, vision_sensor_handle:int):
        self.sim.handleVisionSensor(vision_sensor_handle)


    def get_vision_sensor_params(self, vision_sensor_handle:int) -> (int, int, float, float): # type: ignore
        """
        Retrieve vision sensor parameters, useful for image manipulation.
        :param: vision_sensor_handle -> handle of the vision sensot
        :return: res_x -> image width in pixels
        :return: res_y -> image height in pixel
        :return: angle_x - > the maximum opening angle of the view trapezoid along camera x axis 
        :return: angle_y - > the maximum opening angle of the view trapezoid along camera y axis 
         """
        res_x = self.sim.getObjectInt32Param(vision_sensor_handle, self.sim.visionintparam_resolution_x)
        res_y = self.sim.getObjectInt32Param(vision_sensor_handle, self.sim.visionintparam_resolution_y)
        ratio = res_x / res_y 
        
        # the code below comes from the script attached to the "Camera pixel to 3D positions.ttm" model inside CoppeliaSim
        angle_x = self.sim.getObjectFloatParam(vision_sensor_handle,self.sim.visionfloatparam_perspective_angle)
        angle_y = angle_x
        if (res_x>res_y): 
            angle_y = 2*np.arctan(np.tan(angle_x/2)/ratio)        
        else: 
            angle_x = 2*np.arctan(np.tan(angle_x/2)/ratio)

        return res_x, res_y, angle_x ,angle_y

    def get_rgb_image(self, vision_sensor_handle:int)->np.ndarray:
        """
        Capure RGB data from a vision sensor and manipulates it to obtain RGB image
        :param: vision_sensor_handle -> handle of the virtual sensor
        :return: rgb_img -> numpy tensor containing the three channels (res_y, res_x, 3)
        """
        buffer_img, res_x, res_y = self.sim.getVisionSensorCharImage(vision_sensor_handle)   # retrieve the encoded image
        rgb_img = np.frombuffer(buffer_img, dtype=np.uint8).reshape(res_y, res_x, 3)
        return rgb_img
    
    def get_depth_image(self, vision_sensor_handle:int, res_x:int, res_y:int, distenance_in_meters:bool=True)->np.ndarray:
        """
        Capure depth data from a vision sensor and manipulates it to obtain depth image
        :param: vision_sensor_handle -> handle of the virtual sensor
        :param: res_x -> image width in pixels
        :param: res_y -> image height in pixels
        :param: distence_in_meters -> distance is directly measured in meters
        :return: depth_img -> numpy tensor containing the depth channel (res_y, res_x)
        """ 
        options = 1 if distenance_in_meters else 0
        depth_buffer = self.sim.getVisionSensorDepth(vision_sensor_handle, options) 
        depth_img = np.array(self.sim.unpackFloatTable(depth_buffer[0])).reshape((res_y, res_x))
        return depth_img

    def draw_full_depth_point_cloud(self, vision_sensor_handle:int, depth_image:np.ndarray, 
                                res_x:int, res_y:int, angle_x:float, angle_y:float, 
                                ):
        world_T_camera = self.get_object_pose_matrix(vision_sensor_handle, self.sim.handle_world)  

        for y in range(0,res_y):
            for x in range (0,res_x):
                p_world = self.pixel_to_3d_world([x,y], depth_image, world_T_camera, res_x, res_y, angle_x, angle_y)
                self.draw_point(p_world)

    def pixel_to_3d_world(self, pixel_coord:list, depth_image:np.ndarray, world_T_camera:np.ndarray, res_x:int,
                          res_y:int, angle_x:float, angle_y:float ) -> list:
        x = pixel_coord[0]
        y = pixel_coord[1]

        depth = depth_image[y, x]
        p_camera = np.array([depth*np.tan(angle_x*0.5)*(0.5-(x/(res_x-1)))/0.5, 
                                     depth*np.tan(angle_y*0.5)*((y/(res_y-1))-0.5)/0.5,
                                     depth,
                                     1])
        # Express the position above in the world reference frame
        p_world= world_T_camera@p_camera 

        return [p_world[0], p_world[1], p_world[2]]


    def transform_pixel_coord(self, pixel_coord_a:list, b_T_a:np.ndarray) -> list:
        """
        Transforms a pixel coordinate from one fram to another using a given transformation matrix.
        :param: pixel_coord_a -> the pixel expressed wrt frame a
        :param: b_T_a -> 3x3 matrix which maps frame a on frame b
        :return: pixel_coord_b -> the pixel expressed wrt frame b
        """
        p_a = np.array(pixel_coord_a + [1])
        p_b = b_T_a @ p_a
        return [p_b[0], p_b[1]]

    def init_inverse_kinematics(self, joint_handles:list, tip_handle:int, target_handle:int, base_handle:int):
        
        self.ik_elements = {"joint_handles": [],
                            "base": None,
                            "target": None,
                            "tip":None}
        
        self.ik_env = self.simIK.createEnvironment()
        self.ik_elements["base"] = self.simIK.createDummy(self.ik_env)
        self.simIK.setObjectMatrix(self.ik_env, self.ik_elements["base"], self.simIK.handle_world, self.sim.getObjectMatrix(base_handle)) 
    
        # Reconstruct the robot kinematic chain in the IK environment
        parent = self.ik_elements["base"]
        for i in range(0, len(joint_handles)):
            self.ik_elements["joint_handles"].append(self.simIK.createJoint(self.ik_env, self.simIK.jointtype_revolute))
            self.simIK.setJointMode(self.ik_env, self.ik_elements["joint_handles"][i], self.simIK.jointmode_ik)
            cyclic, interv = self.sim.getJointInterval(joint_handles[i])
            self.simIK.setJointInterval(self.ik_env, self.ik_elements["joint_handles"][i], cyclic, interv)
            self.simIK.setJointPosition(self.ik_env, self.ik_elements["joint_handles"][i], self.sim.getJointPosition(joint_handles[i]))
            self.simIK.setObjectMatrix(self.ik_env, self.ik_elements["joint_handles"][i], self.simIK.handle_world, self.sim.getObjectMatrix(joint_handles[i]))
            self.simIK.setObjectParent(self.ik_env, self.ik_elements["joint_handles"][i], parent, True) 
            parent = self.ik_elements["joint_handles"][i]
        
        self.ik_elements["tip"] = self.simIK.createDummy(self.ik_env)
        self.simIK.setObjectMatrix(self.ik_env, self.ik_elements["tip"], self.simIK.handle_world, self.sim.getObjectMatrix(tip_handle))
        self.simIK.setObjectParent(self.ik_env, self.ik_elements["tip"], parent, True)

        self.ik_elements["target"] = self.simIK.createDummy(self.ik_env)
        self.simIK.setObjectMatrix(self.ik_env, self.ik_elements["target"], self.simIK.handle_world, self.sim.getObjectMatrix(target_handle))
        
        # Link the tip and the target
        self.simIK.setTargetDummy(self.ik_env, self.ik_elements["tip"], self.ik_elements["target"])
        # Setup undumped IK group
        self.ik_group_undamped = self.simIK.createGroup(self.ik_env)
        self.simIK.setGroupCalculation(self.ik_env, self.ik_group_undamped, self.simIK.method_pseudo_inverse, 0, 6)
        self.simIK.setGroupFlags(self.ik_env,  self.ik_group_undamped, self.simIK.group_enabled | 
            self.simIK.group_ignoremaxsteps | self.simIK.group_restoreonbadlintol | self.simIK.group_restoreonbadangtol)
        ik_element_handle = self.simIK.addElement(self.ik_env, self.ik_group_undamped, self.ik_elements["tip"])
        self.simIK.setElementBase(self.ik_env, self.ik_group_undamped, ik_element_handle, self.ik_elements["base"])
        self.simIK.setElementConstraints(self.ik_env, self.ik_group_undamped, ik_element_handle, self.simIK.constraint_pose)

        # Setup damped IK group
        self.ik_group_damped = self.simIK.createGroup(self.ik_env)
        self.simIK.setGroupCalculation(self.ik_env, self.ik_group_damped, self.simIK.method_damped_least_squares, 1, 1000)
        ik_element_handle = self.simIK.addElement(self.ik_env, self.ik_group_damped, self.ik_elements["tip"])
        self.simIK.setElementBase(self.ik_env, self.ik_group_damped, ik_element_handle, self.ik_elements["base"])
        self.simIK.setElementConstraints(self.ik_env, self.ik_group_damped, ik_element_handle, self.simIK.constraint_pose)

    def perform_inverse_kinematics(self, joint_handles:list, tip_handle:int, target_handle:int, base_handle:int) -> list:
        self.simIK.setObjectMatrix(self.ik_env, self.ik_elements["target"], self.ik_elements["base"], self.sim.getObjectMatrix(target_handle, base_handle)) 

        res, *_ = self.simIK.handleGroup(self.ik_env, self.ik_group_undamped)
        if res != self.simIK.result_success:
            self.simIK.handleGroup(self.ik_env, self.ik_group_damped)
            print("[WARNING] Solver failed")

        q_ik = []
        for i in range(0, len(joint_handles)):
            q_ik.append(self.simIK.getJointPosition(self.ik_env, self.ik_elements["joint_handles"][i]))
        return q_ik
    
    def load_image(self,abs_path): 
        img, res = self.sim.loadImage(0,abs_path)
        return img, res

    def remove_obj(self,delate_obj:str):
        delate_obj= delate_obj.lower()
        delate_obj = "/"+ delate_obj
        handle = self.sim.getObject(delate_obj)
        self.sim.removeObjects([handle])
    
    def wait_sim(self, deltaTime:float, simulationTime:bool = True): 
        return self.sim.wait(deltaTime)


