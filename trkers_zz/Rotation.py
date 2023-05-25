import math
import numpy as np
from XYZ_Lon_Lat import rotate_mat

rand_axis = [0, 0, 1]
# 旋转角度
# yaw = 200 * math.pi / 180
yaw = 0
# 返回旋转矩阵
rot_matrix = rotate_mat(rand_axis, yaw)

def Rotation(x, y, z, angle):
    point_x = [x, y, z]
    point_x1 = np.dot(rot_matrix, point_x)
    x = point_x1[0]
    y = point_x1[1]
    z = point_x1[2]
    ag = (angle - 200) % 360  # 角度转正北方向
    if ag < 0:
        ag = ag + 360
    return x, y, z, ag

def Rotation_(x, y, z):
    point_x = [x, y, z]
    point_x1 = np.dot(rot_matrix, point_x)
    x = point_x1[0]
    y = point_x1[1]
    z = point_x1[2]
    return x, y, z