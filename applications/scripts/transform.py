import tf
import numpy as np
from geometry_msgs.msg import Pose

obj_pos = (0.6, -0.1, 0.7)
obj_ori = (0.0, 0.0, 0.38268343, 0.92387953)

pre_pos = (-0.1, 0.0, 0.0)
pre_ori = (0.0, 0.0, 0.0, 0.0)

rot_matrix = tf.transformations.quaternion_matrix(obj_ori)
rot_matrix[0][3] = obj_pos[0]
rot_matrix[1][3] = obj_pos[1]
rot_matrix[2][3] = obj_pos[2]

pregrasp = np.zeros(shape=(4,4), dtype=float)
"""
1 | 0 | 0 | -0.1
0 | 1 | 0 | 0
0 | 0 | 1 | 0
0 | 0 | 0 | 1
"""
pregrasp[0][3] = -0.1
pregrasp[3][3] = 1.0
pregrasp[0][0] = 1.0
pregrasp[1][1] = 1.0
pregrasp[2][2] = 1.0

ans = np.dot(rot_matrix, pregrasp)
quaternion = tf.transformations.quaternion_from_matrix(ans)

pose = Pose()
pose.orientation.x = quaternion[0]
pose.orientation.y = quaternion[1]
pose.orientation.z = quaternion[2]
pose.orientation.w = quaternion[3]
pose.position.x = ans[0][3]
pose.position.y = ans[1][3]
pose.position.z = ans[2][3]

print (pose)

