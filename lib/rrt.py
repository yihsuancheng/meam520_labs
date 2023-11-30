import numpy as np
import random
from lib.detectCollision import detectCollision
from lib.loadmap import loadmap
from copy import deepcopy
from lib.calculateFK import FK
'''
Code for RRT-Connect
'''

fk = FK()

def check_entire_path_collision_free(fk, path, obstacles):
    collision_indices = []
    for i in range(len(path) - 1):
        if not isCollisionFree(fk, path[i], path[i+1], obstacles):
            collision_indices.append(i)
    return True if len(collision_indices) == 0 else False

def isCollisionFree(fk, nearestNode, newPoint, obstacles):
    joint_positions, _ = fk.forward(nearestNode)
    new_joint_positions, _ = fk.forward(newPoint)
    
    for obs in obstacles:
        if (True in detectCollision(joint_positions, joint_positions, obs)):
            return False
        
        elif (True in detectCollision(new_joint_positions, new_joint_positions, obs)):
            return False
    
    for obs in obstacles:
        if (True in detectCollision(joint_positions, new_joint_positions, obs)):
            return False
        
    return True


def extend(tree, point, fk, map, step_size):
    nearestNode = min(tree, key=lambda node: np.linalg.norm(np.array(node) - np.array(point)))
    direction = np.array(point) - np.array(nearestNode)
    length = np.linalg.norm(direction)
    norm_vec = direction / length
    distance = min(step_size, length)

    newPoint = np.array(nearestNode) + distance * norm_vec
    if isCollisionFree(fk, nearestNode, newPoint, map.obstacles):
        tree[tuple(newPoint)] = tuple(nearestNode)
        return tuple(newPoint)
    
    return None

def connect(tree, point, fk, map, step_size):
    extended = True
    while extended:
        newPoint = extend(tree, point, fk, map, step_size)
        if newPoint is None:
            return False, None
        
        # Check if the new point is close enough to the goal or can connect directly
        if np.linalg.norm(np.array(newPoint) - np.array(point)) <= 0.001:
            if isCollisionFree(fk, point, newPoint, map.obstacles):
                #tree[tuple(goal)] = tuple(newPoint)
                return True, tuple(newPoint)

        extended = np.linalg.norm(np.array(newPoint) - np.array(point)) > step_size
    # quits the loop when the newPoint and point is close
    return True, tuple(newPoint)

def build_path(tree_a, tree_b, connect_point_a, connect_point_b, is_tree_a_start):
    path = []

    # Backtrack from connect_point_a to start
    while connect_point_a is not None:
        path.append(connect_point_a)
        connect_point_a = tree_a[connect_point_a]
    path = path[::-1] # Reverse the path

    # Backtrack from connect_point_b to goal
    connect_point_b = tree_b[connect_point_b] # Skip the connecting point as it's already included
    while connect_point_b is not None:
        path.append(connect_point_b)
        connect_point_b = tree_b[connect_point_b]
    
    if not is_tree_a_start:
        return np.array(path[::-1])
    
    return np.array(path)

def rrt(map, start, goal):
    
    lowerLim = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
    upperLim = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])

    tree_a = {tuple(start): None} #Tree from start
    tree_b = {tuple(goal): None}  #Tree from goal

    max_iter = 5000
    step_size = 0.1
    is_tree_a_start = True # Flag to identify which tree is the start tree
    
    for i in range(max_iter):
        # Sample random point
        randPoint = np.random.uniform(low=lowerLim, high=upperLim)
        #randPoint = lowerLim + np.random.random(size=lowerLim.shape) * (upperLim - lowerLim)

        # Extend tree_a towards random point
        newPoint_a = extend(tree_a, randPoint, fk, map, step_size)
        if newPoint_a is not None:
            # Try to connect tree_b to newPoint_a
            success, final_point = connect(tree_b, newPoint_a, fk, map, step_size)
            if success:
                path =  build_path(tree_a, tree_b, newPoint_a, final_point, is_tree_a_start)
                collision_free = check_entire_path_collision_free(fk, path, map.obstacles)

                print(collision_free)
            
                if collision_free:
                    return path

        # Swap trees
        tree_a, tree_b = tree_b, tree_a
        is_tree_a_start = not is_tree_a_start 

    return np.array([]) # Return empty path if no connection made
    
if __name__ == '__main__':
    map_struct = loadmap("../maps/map3.txt")
    
    start = np.array([0,-1,0,-2,0,1.57,0])
    #start = np.array([0, 0.4, 0, -2.5, 0, 2.7, 0.707])
    goal =  np.array([-1.2, 1.57 , 1.57, -2.07, -1.57, 1.57, 0.7])
    #goal = np.array([1.9, 1.57, -1.57, -1.57, 1.57, 1.57, 0.707])
    #goal =  np.array([0,-1,0,-2,0,1.57,0])
    path = rrt(deepcopy(map_struct), deepcopy(start), deepcopy(goal))
    
    print("Path: ", path)
