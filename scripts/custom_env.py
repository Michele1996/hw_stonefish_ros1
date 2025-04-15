#!/usr/bin/env python3
import rospy
import gymnasium as gym
from gymnasium import spaces
import numpy as np
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
import time
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3
from stonefish_ros.msg import INS
import pyproj
import scipy.spatial.transform

def geodetic2enu(lat, lon, alt, lat_org, lon_org, alt_org):
    transformer = pyproj.Transformer.from_crs(
        {"proj":'latlong', "ellps":'WGS84', "datum":'WGS84'},
        {"proj":'geocent', "ellps":'WGS84', "datum":'WGS84'},
        )
    x, y, z = transformer.transform( lon,lat,  alt,radians=False)
    x_org, y_org, z_org = transformer.transform( lon_org,lat_org,  alt_org,radians=False)
    vec=np.array([[ x-x_org, y-y_org, z-z_org]]).T

    rot1 =  scipy.spatial.transform.Rotation.from_euler('x', -(90-lat_org), degrees=True).as_matrix()#angle*-1 : left handed *-1
    rot3 =  scipy.spatial.transform.Rotation.from_euler('z', -(90+lon_org), degrees=True).as_matrix()#angle*-1 : left handed *-1

    rotMatrix = rot1.dot(rot3)    
   
    enu = rotMatrix.dot(vec).T.ravel()
    return enu.T


class CustomEnv(gym.Env):
    metadata = {'render_modes': ['human']}

    def __init__(self):
        
        self.thruster_pubs = rospy.Publisher(
            '/bluerov/controller/thruster_setpoints_sim', Float64MultiArray, queue_size=10)
        # Action and Observation Space Definitions
        self.action_space = spaces.Box(
            low=-2, high=2, shape=(6,), dtype=np.float32)
        self.observation_space = spaces.Dict({
            'position': spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32),
            'orientation': spaces.Box(low=-np.inf, high=np.inf, shape=(4,), dtype=np.float32),
            'thrust': spaces.Box(low=-np.inf, high=np.inf, shape=(6,), dtype=np.float32)
        })

        self.pose = np.zeros((3,), dtype=np.float32)
        self.auv_move_sim_time = 1
        self.auv_data = None
        self.auv_pose = np.zeros((7), dtype=np.float32)
        self.thrust_vector = np.zeros(6, dtype=np.float32)
        rospy.Subscriber("/bluerov/navigator/ins",INS,
                         self._auv_data_callback)
        # Goal Position
        self.goal = np.array([4, 0, -20], dtype=np.float32)

    # Example function to check for NaN values
    def _sanitize_value(self, value):
        return 0 if np.isnan(value) else value

    # In the _auv_data_callback
    def _auv_data_callback(self, data):
        # Sanitize the pose values
        res1 = geodetic2enu(data.latitude, data.longitude, data.altitude, data.origin_latitude, data.origin_longitude, data.altitude)

        self.auv_pose[0] = self._sanitize_value(res1[0])
        self.auv_pose[1] = self._sanitize_value(res1[1])
        self.auv_pose[2] = self._sanitize_value(res1[2])
        self.auv_pose[3] = self._sanitize_value(data.pose.roll)
        self.auv_pose[4] = self._sanitize_value(data.pose.pitch)
        self.auv_pose[5] = self._sanitize_value(data.pose.yaw)
        self.auv_pose[6] = 1.




    def step(self, action):

        rospy.wait_for_service('/bluerov/stop_simulation', timeout=5)
        rospy.wait_for_service('//bluerov/resume_simulation', timeout=5)
        rospy.wait_for_service('/bluerov/restart_simulation', timeout=5)
        rospy.loginfo("[CustomEnv] Stonefish services available.")


        # Limit the action (thrust) values to prevent extreme values
        limited_action = np.clip(action, -1.0, 1.0)  # Adjust the range as needed
        print(limited_action)
        # Unpause, apply thrust, and then pause the simulation
        self.pause_stonefish()
        self.publish_thrust(limited_action) 
        self.resume_stonefish()
        time.sleep(self.auv_move_sim_time)
        # Compute reward, check if episode is done
        reward, terminated, truncated, info = self.compute_reward_and_done()
        # Get the next observation
        next_observation = self._get_observation()
        #print(next_observation)
        # Determine if the episode is finished
        done = terminated or truncated
        return next_observation, reward, done, info

    def pause_stonefish(self):
        # Create a service proxy
        pause_stonefish_service = rospy.ServiceProxy('/bluerov/stop_simulation', Empty)
        # Call the service
        try:
            pause_stonefish_service()
        except rospy.ServiceException as e:
                rospy.logerr("[CustomEnv] Failed to call pause Stonefish service: %s" % e)

    def resume_stonefish(self):
        # Create a service proxy
        resume_stonefish_service = rospy.ServiceProxy('/bluerov/resume_simulation', Empty)
        # Call the service
        try:
            response = resume_stonefish_service()
        except rospy.ServiceException as e:
                rospy.logerr("[CustomEnv] Failed to call resume Stonefish service: %s" % e)


    def publish_thrust(self, action):
        msg = Float64MultiArray()
        val=[]
        for i, thrust in enumerate(action):
            if not np.isnan(thrust):
                val.append(thrust)
            else:
                rospy.logwarn("NaN detected in thruster command.")
        msg.data = val
        self.thruster_pubs.publish(msg)


    def restart_stonefish(self):
        # Create a service proxy
        restart_stonefish_service = rospy.ServiceProxy('/bluerov/restart_simulation', Empty)
        # Call the service
        try:
           response = restart_stonefish_service()
        except rospy.ServiceException as e:
                rospy.logerr("[CustomEnv] Failed to call restart Stonefish service: %s" % e)


    def _get_observation(self):
        # Construct observation from AUV data and thrust vector
        if self.auv_data:
            position = np.array([self.auv_pose[0],
                                self.auv_data.pose[1],
                                self.auv_data.pose[2]], dtype=np.float32)
            orientation = np.array([self.auv_data.pose[3],
                                    self.auv_data.pose[4],
                                    self.auv_data.pose[5],
                                    self.auv_data.pose[6]], dtype=np.float32)
        else:
            # Provide default values if auv_data is None
            position = np.zeros((3,), dtype=np.float32)
            orientation = np.zeros((4,), dtype=np.float32)
        
        #print(position)
        return {'position': position, 'orientation': orientation, 'thrust': self.thrust_vector.astype(np.float32)}



    def prepare_for_reset(self):
        rospy.loginfo("Preparing for reset.")

        # Example: publish zero velocity commands to the thrusters to stop the AUV
        zero_thrust = Float64MultiArray()
        zero_thrust.data = [0.0] * len(self.thruster_pubs)  # Assuming thruster_pubs is a list of thruster publishers
        for pub in self.thruster_pubs:
            pub.publish(zero_thrust)
        # Wait for the AUV to stabilize
        # rospy.loginfo("Preparing for reset ------sleeping.")
        rospy.sleep(2)  # Adjust the time as needed
        #rospy.loginfo("after sleeping ------sleeping.")

    def reset(self, seed=None, options=None,sim_reset=True):
        rospy.loginfo("[CustomEnv] Reset function called.")
        self.controlFlag = 0
        # Reset the simulation
        if(sim_reset):
           self.restart_stonefish() 
        time.sleep(1)
        
        # Reinitialize physics and other post-reset conditions
        #self.init_physics_parameters()

        # Reset observation data
        self.auv_data = None
        self.thrust_vector = np.zeros(6, dtype=np.float32)

        # Return the initial observation
        initial_observation = self._get_observation()
        rospy.loginfo("[CustomEnv] Reset function completed. Initial observation: {}".format(initial_observation))
        return initial_observation, {}

        
    def _is_done(self, pose):
        #rospy.loginfo("compute is_done............")
        # Define workspace boundaries (example values)
        workspace_x_min, workspace_x_max = -10, 10
        workspace_y_min, workspace_y_max = -10, 10
        workspace_z_min, workspace_z_max = -30, -10

        # Check if AUV is out of bounds
        out_of_bounds = not (workspace_x_min <= self.auv_pose[0] <= workspace_x_max and
                             workspace_y_min <= self.auv_pose[1] <= workspace_y_max and
                             workspace_z_min <= self.auv_pose[2] <= workspace_z_max)

        # Check if AUV reached the goal (within a tolerance)
        rospy.loginfo("[CustomEnv] AUV POSE: {}".format(self.auv_pose))
        goal_tolerance = 1.0
        reached_goal = np.linalg.norm(self.auv_pose[:3] - self.goal) < goal_tolerance

        return out_of_bounds or reached_goal
    

    def compute_reward_and_done(self):
        if self.auv_pose is None:
            rospy.logwarn("AUV data not available for reward computation.")
            return -100, True, False, {'error': 'Data not available'}

        # Extract current position and orientation
        
        
        current_position = np.array([
            self.auv_pose[0],
            self.auv_pose[1],
            self.auv_pose[2]
        ], dtype=np.float32)

        current_orientation = np.array([
            self.auv_pose[3],
            self.auv_pose[4],
            self.auv_pose[5],
            self.auv_pose[6]
        ], dtype=np.float32)

        #print("----> ", current_position)
        # Compute distance to goal and orientation error
        distance_to_goal = np.linalg.norm(current_position - self.goal)
        desired_orientation = np.array([0, 0, 0, 1], dtype=np.float32)
        orientation_error = np.linalg.norm(current_orientation - desired_orientation)

        # Compute thrust penalty
        thrust_penalty = np.sum(np.abs(self.thrust_vector)) * 0.1

        # Base reward components
        reward = -distance_to_goal - orientation_error - thrust_penalty

        # Check conditions
        goal_tolerance = 1.0
        reached_goal = distance_to_goal < goal_tolerance
        out_of_bounds = self._is_done(current_position)

        # Check termination condition
        terminated = reached_goal or out_of_bounds

        # Log for debugging
        rospy.loginfo("Distance to Goal: {}".format(distance_to_goal))
        rospy.loginfo("Orientation Error: {}".format(orientation_error))
        rospy.loginfo("Thrust Penalty: {}".format(thrust_penalty))
        rospy.loginfo("Out of Bounds: {}, Reached Goal: {}".format(out_of_bounds, reached_goal))
        rospy.loginfo("Terminated: {}, Reached Goal: {}".format(terminated, reached_goal))

        truncated = False
        info = {'distance_to_goal': distance_to_goal, 'orientation_error': orientation_error}

        if reached_goal:
            reward += 1000

        return reward, terminated, truncated, info



