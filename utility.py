import math
import numpy as np

#--------------------------------------------------------------------------------------------

def difference_btw_angles(theta_1, theta_2):
    theta_1_normalized = (theta_1 + math.pi) % (2 * math.pi) - math.pi
    theta_2_normalized = (theta_2 + math.pi) % (2 * math.pi) - math.pi
    
    difference = theta_1_normalized - theta_2_normalized
    difference_normalized = (difference + math.pi) % (2 * math.pi) - math.pi
    
    return difference_normalized

#--------------------------------------------------------------------------------------------

def t2v(transformation):
    vector = np.zeros((1,3))
    vector[0, 0:2] = transformation[0:2,2]
    vector[0, 2] = math.atan2(transformation[1,0], transformation[0,0])
    
    return np.array(vector[0, :])

#--------------------------------------------------------------------------------------------

def v2t(vector):
    cos = math.cos(vector[2])
    sin = math.sin(vector[2])
    transformation = np.array([[cos, -sin, vector[0]],[sin, cos, vector[1]],[0, 0, 1]])
    
    return np.array(transformation)

#--------------------------------------------------------------------------------------------
