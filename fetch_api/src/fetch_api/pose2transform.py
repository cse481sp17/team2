#!usr/bin/env python

import tf
import tf.transformations as tft
import numpy as np
from geometry_msgs.msg import Pose

def pose2transform(fromPose, toPose, inv=False):
    fromMatrix = pose2matrix(fromPose)
    toMatrix = pose2matrix(toPose)
    if inv:
        fromMatrix = tft.inverse_matrix(fromMatrix)
    return np.dot(fromMatrix, toMatrix)

def pose2matrix(fromPose):
    quaternion = [fromPose.orientation.x, fromPose.orientation.y,
                  fromPose.orientation.z, fromPose.orientation.w]
    fromMatrix = tf.transformations.quaternion_matrix(quaternion)
    fromMatrix[0][3] = fromPose.position.x
    fromMatrix[1][3] = fromPose.position.y
    fromMatrix[2][3] = fromPose.position.z
    fromMatrix[3][0] = 0
    fromMatrix[3][1] = 0
    fromMatrix[3][2] = 0
    fromMatrix[3][3] = 1
    return fromMatrix

def matrix2pose(matrix):
    quaternion = tf.transformations.quaternion_from_matrix(matrix)
    pose = Pose()
    pose.position.x = matrix[0][3]
    pose.position.y = matrix[1][3]
    pose.position.z = matrix[2][3]
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    return pose


