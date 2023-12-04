import sys
import numpy as np
from copy import deepcopy
from math import pi
import math
import time
from core.utils import time_in_seconds
from lib.IK_velocity import IK_velocity

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

def camera_to_robot_base(q):
    # Transform from camera to end-effector
    #H_ee_camera = np.array([[0, -1, 0, 5e-2],[1, 0, 0, 0], [0, 0, 1, -6e-2], [0, 0, 0, 1]])
    
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
    
    _, T0e = fk.forward(q)
    print("T0e: ", T0e)
    H_camera_to_robot_base = np.dot(T0e, H_ee_camera)
    return H_camera_to_robot_base

def block_to_ee(block_to_camera):
    # Transform from block to camera
    block_to_ee = {}
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
    for key, value in block_to_camera.items():
        block_to_ee[key] = np.dot(value, H_ee_camera)
        print("transformed block to end effector frame ", block_to_ee)
    return block_to_ee


def transform_block_from_cam_frame_to_robot_frame(block_to_camera, camera_in_robot_base):
    block_to_robot_base = {}
    for key, value in block_to_camera.items():
        block_to_robot_base[key] = np.dot(camera_in_robot_base, value)
        #block_to_robot_base[key][2,3] = 0.3 + 0.06 # manually set z value for blocks
        print("transformed block to robot base ", block_to_robot_base)
    return block_to_robot_base


def block_detection():
    detected_blocks = {}
    for (name, pose) in detector.get_detections():
        print(name,'\n',pose)
        detected_blocks[name] = pose
    return detected_blocks

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
    detector = ObjectDetector()

    #start_position = np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866])
    #arm.safe_move_to_position(start_position) # on your mark!

    arm.safe_move_to_position(arm.neutral_position())

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
        goal_area[:3, 3] = np.array([0.562, -0.169, 0.22])
    elif team == "red":
        goal_area[:3, 3] = np.array([0.562, 0.169, 0.22])


    # Move end effector to above goal platform so it can detect blocks

    # matches figure in the handout
    seed = np.array([0,0,0,-pi/2,0,pi/2,pi/4])
    seed2 = np.array([-0.14986305, 0.03810911, -0.15776718, -1.52707491, 0.00598615, 1.56471128, 0.47791702])
    arm.safe_move_to_position(seed2)
    
    
    # Detect blocks in camera frame (H_block_to_camera)
    H_block_to_camera = block_detection()
    H_block_to_ee = block_to_ee(H_block_to_camera)
    H_camera_to_robot_base = camera_to_robot_base(seed2)
    #H_camera_to_robot_base = camera_to_robot_base(np.array([0,0,0,-pi/2,0,pi/2,pi/4]))
    blocks_to_robot_base = transform_block_from_cam_frame_to_robot_frame(H_block_to_camera, H_camera_to_robot_base)

    '''
    cube1=np.array([[-1.84465972e-05,  6.73923272e-01, -7.38801342e-01, 4.85695892e-01],
       [ 2.13914367e-06, -7.38801342e-01, -6.73923272e-01, -1.15492499e-01],
       [-1.00000000e+00, -1.40119934e-05,  1.21867521e-05, 2.18623486e-01],
       [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 1.00000000e+00]])
    
    cube2 = np.array([[-5.14910918e-06,  7.19241514e-01, -6.94760134e-01, 4.68099156e-01],
       [-2.29246824e-07, -6.94760134e-01, -7.19241514e-01, -2.39771222e-01],
       [-1.00000000e+00, -3.54418153e-06,  3.74227962e-06, 2.18629970e-01],
       [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 1.00000000e+00]])
    
    cube1_to_ee =  np.array([[ 7.38801342e-01,  2.14108651e-06,  6.73923273e-01, -8.90789947e-02],
       [-6.73923272e-01, -1.84412576e-05,  7.38801342e-01, 4.47661861e-02],
       [ 1.40098303e-05, -1.00000000e+00, -1.21814978e-05, 3.85117166e-01],
       [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 1.00000000e+00]])
    
    cube2_to_ee = np.array([[ 6.94760134e-01, -2.27303970e-07,  7.19241515e-01, 3.27707159e-02],
       [-7.19241515e-01, -5.14376953e-06,  6.94760134e-01, 6.45997906e-02],
       [ 3.54169086e-06, -1.00000000e+00, -3.73717246e-06, 3.85110227e-01],
       [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 1.00000000e+00]])
    
    cube3= np.array([[-2.60282821e-01, -9.65532419e-01,  9.88594333e-06, 5.97095548e-01],
       [ 9.65532419e-01, -2.60282821e-01, -8.45796330e-07, -1.19049860e-01],
       [ 3.38978499e-06,  9.32505252e-06,  1.00000000e+00, 2.18626426e-01],
       [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 1.00000000e+00]])
    
    cube3_to_ee = np.array([[ 2.60282822e-01,  9.65532419e-01,  8.47739182e-07, -7.99517605e-02],
       [ 9.65532419e-01, -2.60282822e-01, -9.88060368e-06, -1.79348016e-02],
       [-9.31939123e-06,  3.39027107e-06, -1.00000000e+00, 4.07353455e-01],
       [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 1.00000000e+00]])
    
    cube4 =  np.array([[-1.07996738e-05,  9.87367538e-01, -1.58446660e-01, 5.98478407e-01],
       [ 1.15368220e-06, -1.58446660e-01, -9.87367538e-01, -2.25989562e-01],
       [-1.00000000e+00, -1.08460444e-05,  5.72063878e-07, 2.18625746e-01],
       [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 1.00000000e+00]])
    
    cube4_to_ee = np.array([[ 1.58446659e-01,  1.15562506e-06,  9.87367539e-01, 4.23127301e-03],
       [-9.87367539e-01, -1.07943341e-05,  1.58446659e-01, -3.81376519e-02],
       [ 1.08410800e-05, -1.00000000e+00, -5.69299514e-07, 3.85114295e-01],
       [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 1.00000000e+00]])
    

    qw = (math.sqrt(1 + cube3_to_ee [0,0] + cube3_to_ee [1,1] + cube3_to_ee[2,2])) / 2
    #qz = (cube3_to_ee [1,0] - cube3_to_ee [0,1])/(4*qw)
    #rot_offset = math.acos(qw) * 2
    rot_offset = math.atan2(cube4_to_ee[1,0], cube4_to_ee[0,0])
    #print("qz: ", qz)
    print("rot_offset is ", rot_offset)

    intermediate = np.zeros((4,4))
    intermediate[:3,:3] = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    intermediate[:3, 3] = np.array([cube4[0,3], cube4[1,3], cube4[2,3] + 0.2])
    intermediate[3,3] = 1


    q, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(intermediate, seed2, method='J_pseudo', alpha=.5)
    #arm.safe_move_to_position(q + np.array([0,0,0,0,0,0, rot_offset-45*pi/180]))
    print("q: ", q)
    arm.safe_move_to_position(q + np.array([0,0,0,0,0,0, rot_offset]))
    print("q - q[-1]: ", q - np.array([0,0,0,0,0,0, q[-1]]))
    '''

    block_to_ee_rot_offset = []
    for key, value in H_block_to_ee.items():
        rot_offset = math.atan2(value[1,0], value[0,0])
        block_to_ee_rot_offset.append(np.array([0,0,0,0,0,0,rot_offset]))
    print("Rot offset: ", block_to_ee_rot_offset)
    blocks_config = []
    blocks_position = []
    #rot_offset_list = []
    for key, value in blocks_to_robot_base.items():
        #print("block: ", key)
        print("key:", key, " value:", value)
        
        value[:3, 3] = value[:3, 3] + np.array([0.025,0,0])
        blocks_position.append(np.array(value[:3, 3]))
        #qw = (math.sqrt(1 + value[0,0] + value[1,1] + value[2,2])) / 2
        #qz = (value[1,0] - value[0,1])/(4*qw)
        #rot_offset = math.acos(qw) * 2
        #print('Rotational_offset', rot_offset)
        
        #rot_offset = arctan2(value)
        value[:3, :3] = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        q, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(value, seed2, method='J_pseudo', alpha=.5)
        #rot_offset_list.append(np.array([0,0,0,0,0,0,-rot_offset]))
        blocks_config.append(np.array(q))
        print("blocks config", q)

    goal_locations = []
    goal_configs = []
    for n in range(len(blocks_config)):
        goal = goal_area.copy()
        if n !=0 :
            goal[2,3] += 0.055 * n 
        print(goal)
        goal_locations.append(goal)
        q, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(goal, seed2, method='J_pseudo', alpha=.5)
        goal_configs.append(np.array(q))
        print("goals_config", q)
    #print("Goal locations: ", goal_locations)
    
     # Planning and Stacking
    for n in range(len(blocks_config)):

        if n == 0:

            arm.exec_gripper_cmd(0.09, grab_force)
            blocks_matrix = np.zeros((4,4))
            blocks_matrix[:3, :3] = np.array([[1, 0, 0],[0, -1, 0],[0, 0, -1]])
            print("blocks_matrix: ", blocks_matrix[:3, :3])
            blocks_matrix[:3, 3] = blocks_position[n] + np.array([0, 0, 0.1])
            blocks_matrix[3,3] = 1

            # Move from previous goal to new block config on top of it
            q_on_top_block, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(blocks_matrix, seed2, method='J_pseudo', alpha=.5)
            print("q_on_top_block: ", q_on_top_block)
            q_with_rot = q_on_top_block + block_to_ee_rot_offset[n]
            #q_new = q_on_top_block
            arm.safe_move_to_position(q_with_rot)

            #Descend gripper
            blocks_matrix[:3, 3] = blocks_matrix[:3, 3] - np.array([0, 0, 0.1])
            print("blocks_matrix: ", blocks_matrix[:3, :3])
            _, block_transform = fk.forward(q_with_rot)
            block_transform[:3, 3] = block_transform[:3, 3] - np.array([0, 0, 0.1])
            q_grip_block, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(block_transform, q_with_rot, method='J_pseudo', alpha=.5)
            print("q_grip_block: ", q_grip_block)
            arm.safe_move_to_position(q_grip_block)
        
        else:
            blocks_matrix = np.zeros((4,4))
            blocks_matrix[:3, :3] = np.array([[1, 0, 0],[0, -1, 0],[0, 0, -1]])
            print("blocks_matrix: ", blocks_matrix[:3, :3])
            blocks_matrix[:3, 3] = blocks_position[n] + np.array([0, 0, 0.1])
            blocks_matrix[3,3] = 1

            # Move from previous goal to new block config on top of it
            q_on_top_block, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(blocks_matrix, goal_new, method='J_pseudo', alpha=.5)
            q_with_rot = q_on_top_block + block_to_ee_rot_offset[n]
            #q_new = q_on_top_block
            arm.safe_move_to_position(q_with_rot)

            #Descend gripper
            blocks_matrix[:3, 3] = blocks_matrix[:3, 3] - np.array([0, 0, 0.1])
            print("blocks_matrix: ", blocks_matrix[:3, :3])
            _, block_transform = fk.forward(q_with_rot)
            block_transform[:3, 3] = block_transform[:3, 3] - np.array([0, 0, 0.1])
            q_grip_block, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(block_transform, q_with_rot, method='J_pseudo', alpha=.5)
            print("q_grip_block: ", q_grip_block)
            arm.safe_move_to_position(q_grip_block)

        arm.exec_gripper_cmd(0.049, grab_force)
        #print("Gripping_ time block{n} is {grip_time}".format(n=n, grip_time=grip_time))
        # Move to seed
        current_position = blocks_position[n]
        #follow_trajectory(lambda t: vertical_line(t, position=current_position))
        desired_pose = np.zeros((4,4))
        desired_pose[:3,:3] = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        desired_pose[:3, 3] = blocks_position[n] + np.array([0, 0, 0.2])
        desired_pose[3, 3] = 1
        q_new, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(desired_pose, blocks_config[n], method='J_pseudo', alpha=.5)
        arm.safe_move_to_position(q_new)
        
        # Move to goal
        arm.safe_move_to_position(goal_configs[n])
        arm.exec_gripper_cmd(0.09, grab_force)

        goal_lift = np.zeros((4,4))
        goal_lift[:3,:3] = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        goal_lift[:3, 3] = goal_locations[n][:3, 3] + np.array([0, 0, 0.2])
        goal_lift[3, 3] = 1
        goal_new, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(goal_lift, goal_configs[n], method='J_pseudo', alpha=.5)
        arm.safe_move_to_position(goal_new)
    

        
    
        





    

