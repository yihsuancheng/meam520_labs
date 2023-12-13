import sys
import numpy as np
from copy import deepcopy
from math import pi
import math
import time
from core.utils import time_in_seconds
from lib.IK_velocity import IK_velocity
from scipy.spatial.transform import Rotation as R

import rospy
# Common interfaces for interacting with both the simulation and real environments!
from core.interfaces import ArmController
from core.interfaces import ObjectDetector
from lib.calculateFK import FK
from lib.calcJacobian import calcJacobian
from lib.IK_position_null import IK


# for timing that is consistent with simulation or real time as appropriate
from core.utils import time_in_seconds

lowerLim = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
upperLim = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])

grab_force = 50

fk = FK()
ik = IK()


def get_H_ee_camera():
    if team == "red":
        H_ee_camera = np.array([[0, -1, 0, 3.00e-2],
                                [1, 0, 0, 0.14e-2], 
                                [0, 0, 1, -5.224e-2], 
                                [0, 0, 0, 1]])
    elif team == "blue":
        H_ee_camera = np.array([[0, -1, 0, 6.008e-2],
                                [1, 0, 0, -1.5e-2], 
                                [0, 0, 1, -6.837e-2], 
                                [0, 0, 0, 1]])
    return H_ee_camera


def camera_to_robot_base(q):
    # Transform from camera to end-effector
    #H_ee_camera = np.array([[0, -1, 0, 5e-2],[1, 0, 0, 0], [0, 0, 1, -6e-2], [0, 0, 0, 1]])
    
    H_ee_camera = get_H_ee_camera()
    
    _, T0e = fk.forward(q)
    #print("T0e: ", T0e)
    H_camera_to_robot_base = np.dot(T0e, H_ee_camera)
    return H_camera_to_robot_base

def block_to_ee(block_to_camera):
    # Transform from block to camera
    block_to_ee = {}
    H_ee_camera = get_H_ee_camera()
    for key, value in block_to_camera.items():
        block_to_ee[key] = np.dot(value, H_ee_camera)
        #print("transformed block to end effector frame ", block_to_ee)
    return block_to_ee

def transform_block_from_cam_frame_to_robot_frame(block_to_camera, camera_in_robot_base):
    block_to_robot_base = {}
    for key, value in block_to_camera.items():
        block_to_robot_base[key] = np.dot(camera_in_robot_base, value)
        #block_to_robot_base[key][2,3] = 0.3 + 0.06 # manually set z value for blocks
        #print("transformed block to robot base ", block_to_robot_base)
    return block_to_robot_base

def get_rot_offset(H_block_to_ee):
    block_to_ee_rot_offset = []
    for key, value in H_block_to_ee.items():
        if abs(value[2,0]) > 0.97 :
            rot_offset = math.atan(value[1,1]/value[0,1])
        else:
            rot_offset = math.atan(value[1,0]/value[0,0])
        block_to_ee_rot_offset.append(np.array([0,0,0,0,0,0,rot_offset]))
    #print("lock_to_ee_rot_offset is ", block_to_ee_rot_offset)
    return block_to_ee_rot_offset

def get_rot_offset2(H_block_to_ee):
    block_to_ee_rot_offset = []
    for key, value in H_block_to_ee.items():
        
        qw = (math.sqrt(1 + value[0,0] + value[1,1] + value[2,2])) / 2
        qz = (value[1,0] - value[0,1])/(4*qw)
        rot_offset = math.acos(qw) * 2
        if qz > 0 :
            block_to_ee_rot_offset.append(np.array([0,0,0,0,0,0,rot_offset]))
        else:
            block_to_ee_rot_offset.append(np.array([0,0,0,0,0,0,-rot_offset]))
    print("lock_to_ee_rot_offset is ", block_to_ee_rot_offset)
    return block_to_ee_rot_offset

def get_blocks_config(blocks_to_robot_base):
    blocks_config = []
    blocks_position = []
    #rot_offset_list = []
    for key, value in blocks_to_robot_base.items():
        #print("block: ", key)
        print("key:", key, " value:", value)
        
        if team == "red":
            value[:3, 3] = value[:3, 3] + np.array([0.025,0.01,0.0125])

        if team == "blue":
            value[:3, 3] = value[:3, 3] + np.array([-0.025, -0.0125, 0.0])
        
        blocks_position.append(np.array(value[:3, 3]))

        value[:3, :3] = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        q, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(value, seed3, method='J_pseudo', alpha=1)

        blocks_config.append(np.array(q))
        print("blocks config", q)
    return blocks_config, blocks_position

def get_goal_pose(goal_area):
    goal_locations = []
    goal_configs = []
    for n in range(len(blocks_config)):
        goal = goal_area.copy()
        if n !=0 :
            goal[2,3] += (0.05 * n + 0.02)
        print(goal)
        goal_locations.append(goal)
        q, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(goal, seed3, method='J_pseudo', alpha=1)
        goal_configs.append(np.array(q))
    print("goals_config", q)
    print("goal_locations: ", goal_locations)
    return goal_locations, goal_configs

# For dynamic blocks
def compute_distance(T0e, target):
    distance = np.linalg.norm(target[:3,3] - T0e[:3,3])
    return distance

def nearest_neighbor(T0e, blocks_to_robot_base, rot_offset):
    #first_value = list(blocks_to_robot_base.values())[0]
    #min_dist = compute_distance(T0e, first_value)
    min_dist = float('inf')
    nearest_node = None
    nearest_rot_offset = None
    if len(blocks_to_robot_base) == 0:
        return None, None, None
    for i, (key, value) in enumerate(blocks_to_robot_base.items()):
        dist = compute_distance(T0e, value)
        if dist < min_dist:
            min_dist = dist
            nearest_node = value
            nearest_rot_offset = rot_offset[i]               # for list - initial rot offset
            #nearest_rot_offset = rot_offset[key]            # for dictionary - final rot offset
    print("distance: ", min_dist, "nearest_value: ", nearest_node, "nearest_rot_offset: ", nearest_rot_offset)
    return min_dist, nearest_node, nearest_rot_offset

def robot_base_to_turntable():
    if team == "red":
        H_base_to_turntable = np.array([[1, 0, 0, 0],
                                        [0, 1, 0, -0.99],
                                        [0, 0, 1, -0.2],
                                        [0, 0, 0, 1]])
    elif team == "blue":
        H_base_to_turntable = np.array([[1, 0, 0, 0],
                                        [0, 1, 0, 0.99],
                                        [0, 0, 1, -0.2],
                                        [0, 0, 0, 1]])
    return H_base_to_turntable

def block_to_turntable(H_block_to_robot_base, H_base_to_turntable):
    H_block_to_turntable = {}
    for key, value in H_block_to_robot_base.items():
        H_block_to_turntable[key] = np.dot(H_base_to_turntable, value)
    #print("H_block_to_turntable: ", H_block_to_turntable)
    return H_block_to_turntable

def transform_block_from_turntable_to_robot_base(H_base_to_turntable, H_block_to_turntable):
    H_block_from_turntable_to_robot_base = {}
    H_turntable_to_base = np.linalg.inv(H_base_to_turntable)
    for key, value in H_block_to_turntable.items():
        H_block_from_turntable_to_robot_base[key] = np.dot(H_turntable_to_base, value)
    print("transform_block_from_turntable_to_robot_base: ", H_block_from_turntable_to_robot_base)
    return H_block_from_turntable_to_robot_base

def predict_current_block_angle(H_block_to_turntable):
    current_angle = {}
    for key, value in H_block_to_turntable.items():
        x, y = value[0, 3], value[1, 3]
        theta = math.atan2(y,x)
        #print("Theta (radians):", theta)
        theta_degrees = math.degrees(theta)
        #print("Theta (degrees):", theta_degrees)
        current_angle[key] = theta
    #print("current_angles: ", current_angle)
    return current_angle

def predict_new_block_position(H_block_to_turntable, current_angle, r = 0.25, omega=0.052, t = 6):
    predicted_position = {}
    for key, value in H_block_to_turntable.items():
        theta = current_angle[key] + omega * t

        r = math.sqrt(value[0,3]**2 + value[1,3]**2)
        print("initial radius: ", r)
        # Calculate x and y coordinates
        x = r * math.cos(theta)
        y = r * math.sin(theta)

        value[:2, 3] = np.array([x,y])
        predicted_position[key] = value
    #print("predicted_postion: ", predicted_position)
    return predicted_position

def final_rotation_angle(predicted_position):
    final_rot_offset = {}
    for key, value in predicted_position.items():
        x, y = value[0, 3], value[1, 3]
        theta = math.atan(y/x)
        #print("Theta (radians):", theta)
        theta_degrees = math.degrees(theta)
        #print("Theta (degrees):", theta_degrees)
        final_rot_offset[key] = theta
    #print("final_rot_offset is: ", final_rot_offset)
    return final_rot_offset

def get_dynamic_goal_location(num_of_block_at_goal):
    goal = goal_area.copy()
    goal[2,3] += (0.05 * num_of_block_at_goal + 0.03)
    print(goal)
    q, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(goal, arm.neutral_position(), method='J_pseudo', alpha=1)
    #print("q to move on top of goal is: ", q)
    #print("dynamic goal location is: ", goal)
    return goal, q

def block_detection():
    detected_blocks = {}
    for (name, pose) in detector.get_detections():
        print(name,'\n',pose)
        detected_blocks[name] = pose
    #print("block_detection ", detected_blocks)
    return detected_blocks

def test_ik(target, q):
    q_new, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(target, q, method='J_pseudo', alpha=.5)
    print("q_new: ", q_new)
    return q_new


def vertical_line(t,f=0.1,L=.3, position=None):
        """
        Calculate the position and velocity of the line trajector

        Inputs:
        t - time in sec since start
        f - frequecny in Hz of the line trajectory
        L - length of the line in meters
        
        Outputs:
        xdes = 0x3 np array of target end effector position in the world frame
        vdes = 0x3 np array of target end effector linear velocity in the world frame
        """
        
        x0 = np.array(position)
        print("x0: ", x0)
        v_z = 0.1
        z_movement = min(v_z * t, L) 
        xdes = x0 + np.array([0, 0, z_movement])
        
        # Set the velocity
        if z_movement < L:
            vdes = np.array([0, 0, v_z])
        else:
            vdes = np.array([0,0,0])

        return xdes, vdes

start_time, last_iteration_time = 0, None
def follow_trajectory(trajectory):
    global last_iteration_time
    global start_time
    t = time_in_seconds() - start_time
    xdes, vdes = trajectory(t)
    # get current end effector position
    q = arm.get_positions()
    joints, T0e = fk.forward(q)
    x = (T0e[0:3,3])

    # First Order Integrator, Proportional Control with Feed Forward
    kp = 20
    v = vdes + kp * (xdes - x)
     # Velocity Inverse Kinematics
    dq = IK_velocity(q,v,np.array([np.nan,np.nan,np.nan]))
    if last_iteration_time == None:
        last_iteration_time = time_in_seconds()
    dt =  time_in_seconds() - last_iteration_time
    new_q = q + dt * dq
    arm.safe_set_joint_positions_velocities(new_q, dq)


if __name__ == "__main__":
    try:
        team = rospy.get_param("team") # 'red' or 'blue'
    except KeyError:
        print('Team must be red or blue - make sure you are running final.launch!')
        exit()

    rospy.init_node("team_script")
    arm = ArmController()
    gripper_state = arm.get_gripper_state()
    #new_speed = 0.3  # Example speed, you can adjust this value
    #arm.set_arm_speed(new_speed)
    arm.set_gripper_speed(0.2)
    detector = ObjectDetector()

    start_position = np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866])
    arm.safe_move_to_position(start_position) # on your mark!

    #arm.safe_move_to_position(arm.neutral_position())

    print("\n****************")
    if team == 'blue':
        print("** BLUE TEAM  **")
    else:
        print("**  RED TEAM  **")
    print("****************")
    input("\nWaiting for start... Press ENTER to begin!\n") # get set!
    print("Go!\n") # go!

    # STUDENT CODE HERE

    '''
    # get the transform from camera to panda_end_effector
    H_ee_camera = detector.get_H_ee_camera()

    # Detect some blocks...
    for (name, pose) in detector.get_detections():
         print(name,'\n',pose)

    
    # Move around...

    # END STUDENT CODE
    '''

    # Define goal area
    goal_area = np.zeros((4,4))
    goal_area[:3, :3] = np.array([[1, 0, 0],
                                 [0, -1, 0],
                                 [0, 0, -1]])
    goal_area[3,3] = 1
    
    if team == "blue":
        goal_area[:3, 3] = np.array([0.562, -0.169, 0.235])
        initial_pose = np.array([[1, 0, 0, 0.54], [0, -1, 0, 0.169], [0, 0, -1, 0.5], [0, 0, 0, 1]])
        seed3 = np.array([0.16359389,  0.02404338,  0.14198584, -1.59996016, -0.00340682,  1.62376137, 1.09111777])
        dynamic_initial_pose = np.array([-1.34320485,  0.52483039, -0.36709061, -0.99525212,  0.18138584,  1.49201766, 0.70618114])

    elif team == "red":
        goal_area[:3, 3] = np.array([0.562, 0.169, 0.235])
        #initial_pose = np.array([[1, 0, 0, 0.52], [0, -1, 0, -0.169], [0, 0, -1, 0.5], [0, 0, 0, 1]])
        initial_pose = np.array([[1, 0, 0, 0.52], [0, -1, 0, -0.169], [0, 0, -1, 0.4], [0, 0, 0, 1]])
        #seed3 = np.array([-0.16049601, -0.03926891, -0.14988066, -1.67550379, -0.00587491,  1.63667354, 0.47552206])
        seed3 = np.array([-0.16293594, -0.04484732, -0.14845203, -1.93735274, -0.00699081,  1.89299069, 0.47637094])
        dynamic_initial_pose = np.array([1.43071902,  0.51110667,  0.22365165, -0.99712137, -0.10899183,  1.49797222, 0.833205])
    

    # Move end effector to above goal platform so it can detect blocks
    
    arm.safe_move_to_position(seed3)
    
    # Static
    # Detect blocks in camera frame (H_block_to_camera)
    H_block_to_camera = block_detection()
    num_pose_adjust = 1
    while len(H_block_to_camera)!= 4:
        new_view_pose = initial_pose.copy()
        new_view_pose[2,3] += 0.05 * num_pose_adjust
        q, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(new_view_pose, seed3, method='J_pseudo', alpha=1)
        arm.safe_move_to_position(q)
        H_block_to_camera = block_detection()
        num_pose_adjust += 1
        if new_view_pose[2,3] > 0.6:
            break

    if num_pose_adjust > 1:
        seed3 = q
    
    H_block_to_ee = block_to_ee(H_block_to_camera)
    H_camera_to_robot_base = camera_to_robot_base(seed3)
    #H_camera_to_robot_base = camera_to_robot_base(np.array([0,0,0,-pi/2,0,pi/2,pi/4]))
    blocks_to_robot_base = transform_block_from_cam_frame_to_robot_frame(H_block_to_camera, H_camera_to_robot_base)
    block_to_ee_rot_offset = get_rot_offset(H_block_to_ee)
    blocks_config, blocks_position = get_blocks_config(blocks_to_robot_base)
    goal_locations, goal_configs = get_goal_pose(goal_area)
    
    #start = time.time()
    # Planning and Stacking for static blocks
    num_block_at_goal = 0
    goal_blocks = []
    success = False
    track_block = 1
    arm.exec_gripper_cmd(0.09, grab_force)
    start = time.time()
    for n in range(len(blocks_config)):
        if n == 0:
            goal_new = seed3
    
        blocks_matrix = np.zeros((4,4))
        blocks_matrix[:3, :3] = np.array([[1, 0, 0],[0, -1, 0],[0, 0, -1]])
        print("blocks_matrix: ", blocks_matrix[:3, :3])
        blocks_matrix[:3, 3] = blocks_position[n] + np.array([0, 0, 0.1])
        blocks_matrix[3,3] = 1

        # Move from previous goal to new block config on top of it
        q_on_top_block, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(blocks_matrix, goal_new, method='J_pseudo', alpha=1)
        print("q_on_top_block: ", q_on_top_block)
        q_with_rot = q_on_top_block + block_to_ee_rot_offset[n]
        print("q_with_rot: ", q_with_rot)
        #q_new = q_on_top_block
        arm.safe_move_to_position(q_with_rot)

        #Descend gripper
        blocks_matrix[:3, 3] = blocks_matrix[:3, 3] - np.array([0, 0, 0.1])
        print("blocks_matrix: ", blocks_matrix[:3, :3])
        _, block_transform = fk.forward(q_with_rot)
        block_transform[:3, 3] = block_transform[:3, 3] - np.array([0, 0, 0.1])
        q_grip_block, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(block_transform, q_with_rot, method='J_pseudo', alpha=1)
        print("q_grip_block: ", q_grip_block)
        arm.safe_move_to_position(q_grip_block)

        arm.exec_gripper_cmd(0.04, grab_force)

        # Lift the gripper
        current_position = blocks_position[n]
        #follow_trajectory(lambda t: vertical_line(t, position=current_position))
        desired_pose = np.zeros((4,4))
        desired_pose[:3,:3] = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        desired_pose[:3, 3] = blocks_position[n] + np.array([0, 0, 0.1])
        desired_pose[3, 3] = 1
        q_new, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(desired_pose, blocks_config[n], method='J_pseudo', alpha=1)
        arm.safe_move_to_position(q_new)
        
        #rospy.sleep(1)
        """gripper_position = gripper_state['position']
        print("gripper_position: ", gripper_position)
        grip_width = gripper_position[0] + gripper_position[1]
        print("grip_width is ", grip_width)
        if grip_width > 0.049:
            success = True
            print("successfully gripped the block", success)
        else: 
            pass #if have time implement this if gripper failed to catch block"""
        
        # Move to goal
        arm.safe_move_to_position(goal_configs[n])

        gripper_state = arm.get_gripper_state()
        gripper_position = gripper_state['position']
        print("gripper_position: ", gripper_position)
        grip_width = gripper_position[0] + gripper_position[1]
        if grip_width > 0.035:
            print("successfully gripped the block")
            num_block_at_goal += 1

        arm.exec_gripper_cmd(0.09, grab_force)

        # Lift the gripper
        goal_lift = np.zeros((4,4))
        goal_lift[:3,:3] = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        goal_lift[:3, 3] = goal_locations[n][:3, 3] + np.array([0, 0, 0.1])
        goal_lift[3, 3] = 1
        goal_new, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(goal_lift, goal_configs[n], method='J_pseudo', alpha=1)
        arm.safe_move_to_position(goal_new)

        print("num of goal blocks:", num_block_at_goal)

    end = time.time()
        #break
    #end = time.time()
    print("Total Time taken to place and stack 4 blocks: ", end - start)
    
    #num_block_at_goal = 0
    # Dynamic blocks
    success = False
    while not success or num_block_at_goal < 7:
        success = False
        if team == "red":
            dynamic_initial_pose = np.array([1.43071902,  0.51110667,  0.22365165, -0.99712137, -0.10899183,  1.49797222, 0.833205])
            r, omega, t = 0.25, 0.05, 7
        elif team == "blue":
            dynamic_initial_pose = np.array([-1.34320485,  0.52483039, -0.36709061, -0.99525212,  0.18138584,  1.49201766, 0.70618114])
            r, omega, t = 0.24, 0.05, 8
        
        arm.safe_move_to_position(dynamic_initial_pose)
        
        # Open Gripper and calculate transformations
        arm.exec_gripper_cmd(0.09, grab_force)
        H_block_to_camera = block_detection()
        #print("H_block_to_camera: ", H_block_to_camera)
        if len(H_block_to_camera) == 0: # no block detection
            continue

        H_block_to_ee = block_to_ee(H_block_to_camera)
        initial_rot_offset = get_rot_offset(H_block_to_ee)
        H_camera_to_robot_base = camera_to_robot_base(dynamic_initial_pose)
        blocks_to_robot_base = transform_block_from_cam_frame_to_robot_frame(H_block_to_camera, H_camera_to_robot_base)
        H_base_to_turntable = robot_base_to_turntable()
        H_block_to_turntable = block_to_turntable(blocks_to_robot_base, H_base_to_turntable)
        current_angle = predict_current_block_angle(H_block_to_turntable)
    
        predicted_position = predict_new_block_position(H_block_to_turntable, current_angle, r=r, omega=omega, t=t)
        final_rot_offset = final_rotation_angle(predicted_position) # this is dictionary
        H_block_from_turntable_to_robot_base = transform_block_from_turntable_to_robot_base(H_base_to_turntable, predicted_position)
        #blocks_to_robot_base = transform_block_from_cam_frame_to_robot_frame(H_block_to_camera, H_camera_to_robot_base)
        _, T0e = fk.forward(dynamic_initial_pose)

        min_dist, nearest_value, offset = nearest_neighbor(T0e, H_block_from_turntable_to_robot_base, initial_rot_offset) # could change to intial rot offset
        print("min distance from ee to block: ", min_dist)
        print("End effector position: ", T0e)

        if min_dist > 0.27:   # may need to change to 0.28 for team red
            continue
        
        #grip dynamic blocks
        nearest_pose = np.zeros((4,4))
        nearest_pose[:3,:3] = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        nearest_pose[:3, 3] = nearest_value[:3, 3] + np.array([0, 0, 0.01])
        nearest_pose[3, 3] = 1
        q_grip_block, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(nearest_pose, dynamic_initial_pose, method='J_pseudo', alpha=1)
        q_new = q_grip_block + offset
        constrained_q_new = np.clip(q_new, lowerLim, upperLim)
        arm.safe_move_to_position(constrained_q_new)
        arm.exec_gripper_cmd(0.04, grab_force)
        
        #lift gripper
        #_, T0e = fk.forward(q_new)
        """desired_pose = np.zeros((4,4))
        desired_pose[:3,:3] = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        desired_pose[:3, 3] = nearest_pose[:3, 3] + np.array([0, 0, 0.06])
        desired_pose[3, 3] = 1
        #T0e[:3, :3] = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        #T0e[:3, 3] = T0e[:3, 3] + np.array([0, 0, 0.2])
        #print("TOe ", T0e)
        q_new2, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(desired_pose, q_new , method='J_pseudo', alpha=1)
        if team == "red":
            final_q = q_new2 + np.array([0, 0, 0, 0, 0, 0, -pi/2])
        elif team == "blue":
            final_q = q_new2 + np.array([0, 0, 0, 0, 0, 0, pi/2])
        arm.safe_move_to_position(final_q)
        print("q_lift: ", final_q)"""
        if team == "red":
            dynamic_initial_pose = np.array([1.43071902,  0.51110667,  0.22365165, -0.99712137, -0.10899183,  1.49797222, 0.833205])
        elif team == "blue":
            dynamic_initial_pose = np.array([-1.34320485,  0.52483039, -0.36709061, -0.99525212,  0.18138584,  1.49201766, 0.70618114])
        arm.safe_move_to_position(dynamic_initial_pose)

        rospy.sleep(1)
        gripper_state = arm.get_gripper_state()
        gripper_position = gripper_state["position"]
        print("gripper_position", gripper_position)
        gripper_width = gripper_position[0] + gripper_position[1]
        print("gripper_width", gripper_width)
        if gripper_width > 0.034:
            success = True
            print("success or not", success)
            # Move to goal pose
            if team == "red":
                dynamic_initial_pose = np.array([1.43071902,  0.51110667,  0.22365165, -0.99712137, -0.10899183,  1.49797222, 0.833205])
            elif team == "blue":
                dynamic_initial_pose = np.array([-1.34320485,  0.52483039, -0.36709061, -0.99525212,  0.18138584,  1.49201766, 0.70618114])

            arm.safe_move_to_position(dynamic_initial_pose)
            dynamic_goal, q = get_dynamic_goal_location(num_block_at_goal)
            arm.safe_move_to_position(q)
            arm.exec_gripper_cmd(0.09, grab_force)

            # Lift the gripper
            goal_lift = np.zeros((4,4))
            goal_lift[:3,:3] = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
            goal_lift[:3, 3] = dynamic_goal[:3, 3] + np.array([0, 0, 0.1])
            goal_lift[3, 3] = 1
            goal_new, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(goal_lift, q, method='J_pseudo', alpha=1)
            arm.safe_move_to_position(goal_new)
            
            num_block_at_goal += 1
            print("total number of blocks on stack: ", num_block_at_goal)
        else:
            print("Failed to grip dynamic blocks: ", success, " retry")
    
    
        




    

