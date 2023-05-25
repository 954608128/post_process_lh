from __future__ import print_function

import math
from shapely.geometry import Polygon

import numpy as np
import core.box_np_ops as box_np_ops

#/*
#计算两帧的全部iou值
# #
def rotate_nms_cc(dets, trackers):
    trackers_corners = box_np_ops.center_to_corner_box2d(trackers[:, :2], trackers[:, 2:4], trackers[:, 4])
    trackers_standup = box_np_ops.corner_to_standup_nd(trackers_corners)
    dets_corners = box_np_ops.center_to_corner_box2d(dets[:, :2], dets[:, 2:4],dets[:, 4])
    dets_standup = box_np_ops.corner_to_standup_nd(dets_corners)
    standup_iou = box_np_ops.iou_jit(dets_standup, trackers_standup, eps=0.0)
    return standup_iou


#/*
#计算两个目标的iou
#/*
def Iou_Two_Box1(dets, trackers):
    trackers_corners = box_np_ops.center_to_corner_box2d(trackers[:, :2], trackers[:, 2:4], trackers[:, 4])
    trackers_standup = box_np_ops.corner_to_standup_nd(trackers_corners)
    dets_corners = box_np_ops.center_to_corner_box2d(dets[:, :2], dets[:, 2:4], dets[:, 4])
    dets_standup = box_np_ops.corner_to_standup_nd(dets_corners)
    standup_iou = box_np_ops.iou_jit(dets_standup, trackers_standup, eps=0.0)
    # np.savetxt('./data_save/1.csv', trackers_corners[0], delimiter=',')
    # np.savetxt('./data_save/2.csv', dets_corners[0], delimiter=',')

    iou = standup_iou[0, 0]
    return iou

#/*
#计算两个目标的iou
#/*
def Iou_Two_Box(dets, trackers):
    dets = np.array([[dets[0], dets[1], dets[2], dets[3], dets[4] * math.pi / 180.0, dets[5], dets[6], dets[7]]])  # X,Y,L,W,角度,Z,H,class
    trackers = np.array([[trackers[0], trackers[1], trackers[2], trackers[3], trackers[4] * math.pi / 180.0, trackers[5], trackers[6], trackers[7]]])  # X,Y,W,L,角度,Z,H,class
    trackers_corners = box_np_ops.center_to_corner_box2d(trackers[:, :2], trackers[:, 2:4], trackers[:, 4])
    trackers_standup = box_np_ops.corner_to_standup_nd(trackers_corners)
    dets_corners = box_np_ops.center_to_corner_box2d(dets[:, :2], dets[:, 2:4], dets[:, 4])
    dets_standup = box_np_ops.corner_to_standup_nd(dets_corners)
    standup_iou = box_np_ops.iou_jit(dets_standup, trackers_standup, eps=0.0)

    iou = standup_iou[0, 0]
    return iou

def intersection(g, p):
    g = np.asarray(g)
    p = np.asarray(p)
    g = Polygon(g[:8].reshape((4, 2)))
    p = Polygon(p[:8].reshape((4, 2)))
    if not g.is_valid or not p.is_valid:
        return 0
    inter = Polygon(g).intersection(Polygon(p)).area
    union = g.area + p.area - inter
    if union == 0:
        return 0
    else:
        return inter / union

#/*
# 如果要检测一帧中与另一帧中的所有box的iou， dets，trackers分别为所有的box
# 如果要检测一个box为另一个box的iou， dets，trackers分别为单个box
#/*
# 314	1	2973.143387	-1667.336693	-374.227381	2.99902	180	452.093601	200.879049	161.117005	0	1	0	0	0
#
# 381	1	2956.69754	-1950.961933	-368.69216	4.122714	180	474.537802	200.994492	160.072803	0	1	0	0	0

# / *
# 计算iou的方法,3D IoU caculate code for 3D object detection
# / ********************************************************************************
from scipy.spatial import ConvexHull
from numpy import *


def polygon_clip(subjectPolygon, clipPolygon):
    """ Clip a polygon with another polygon.
    Ref: https://rosettacode.org/wiki/Sutherland-Hodgman_polygon_clipping#Python
    Args:
      subjectPolygon: a list of (x,y) 2d points, any polygon.
      clipPolygon: a list of (x,y) 2d points, has to be *convex*
    Note:
      **points have to be counter-clockwise ordered**
    Return:
      a list of (x,y) vertex point for the intersection polygon.
    """

    def inside(p):
        return (cp2[0] - cp1[0]) * (p[1] - cp1[1]) > (cp2[1] - cp1[1]) * (p[0] - cp1[0])

    def computeIntersection():
        dc = [cp1[0] - cp2[0], cp1[1] - cp2[1]]
        dp = [s[0] - e[0], s[1] - e[1]]
        n1 = cp1[0] * cp2[1] - cp1[1] * cp2[0]
        n2 = s[0] * e[1] - s[1] * e[0]
        n3 = 1.0 / (dc[0] * dp[1] - dc[1] * dp[0])
        return [(n1 * dp[0] - n2 * dc[0]) * n3, (n1 * dp[1] - n2 * dc[1]) * n3]

    outputList = subjectPolygon
    cp1 = clipPolygon[-1]

    for clipVertex in clipPolygon:
        cp2 = clipVertex
        inputList = outputList
        outputList = []
        s = inputList[-1]

        for subjectVertex in inputList:
            e = subjectVertex
            if inside(e):
                if not inside(s):
                    outputList.append(computeIntersection())
                outputList.append(e)
            elif inside(s):
                outputList.append(computeIntersection())
            s = e
        cp1 = cp2
        if len(outputList) == 0:
            return None
    return outputList


def poly_area(x, y):
    """ Ref: http://stackoverflow.com/questions/24467972/calculate-area-of-polygon-given-x-y-coordinates """
    return 0.5 * np.abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))


def convex_hull_intersection(p1, p2):
    """ Compute area of two convex hull's intersection area.
        p1,p2 are a list of (x,y) tuples of hull vertices.
        return a list of (x,y) for the intersection and its volume
    """
    inter_p = polygon_clip(p1, p2)
    if inter_p is not None:
        hull_inter = ConvexHull(inter_p)
        return inter_p, hull_inter.volume
    else:
        return None, 0.0


def box3d_vol(corners):
    """
    corners: (8,3) no assumption on axis direction
    """
    a = np.sqrt(np.sum((corners[0, :] - corners[1, :]) ** 2))
    b = np.sqrt(np.sum((corners[1, :] - corners[2, :]) ** 2))
    c = np.sqrt(np.sum((corners[0, :] - corners[4, :]) ** 2))
    return a * b * c


def is_clockwise(p):
    x = p[:, 0]
    y = p[:, 1]
    return np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)) > 0


def box3d_iou(corners1, corners2):
    """ Compute 3D bounding box IoU.
    Input:
        corners1: numpy array (8,3), assume up direction is negative Y
        corners2: numpy array (8,3), assume up direction is negative Y
    Output:
        iou: 3D bounding box IoU
        iou_2d: bird's eye view 2D bounding box IoU
    todo (kent): add more description on corner points' orders.
    """
    # corner points are in counter clockwise order
    rect1 = [(corners1[i, 0], corners1[i, 2]) for i in range(3, -1, -1)]
    rect2 = [(corners2[i, 0], corners2[i, 2]) for i in range(3, -1, -1)]

    area1 = poly_area(np.array(rect1)[:, 0], np.array(rect1)[:, 1])
    area2 = poly_area(np.array(rect2)[:, 0], np.array(rect2)[:, 1])

    inter, inter_area = convex_hull_intersection(rect1, rect2)
    iou_2d = inter_area / (area1 + area2 - inter_area)
    ymax = min(corners1[0, 1], corners2[0, 1])
    ymin = max(corners1[4, 1], corners2[4, 1])

    inter_vol = inter_area * max(0.0, ymax - ymin)

    vol1 = box3d_vol(corners1)
    vol2 = box3d_vol(corners2)
    iou = inter_vol / (vol1 + vol2 - inter_vol)
    return iou, iou_2d


# ----------------------------------
# Helper functions for evaluation
# ----------------------------------

def get_3d_box(box_size, heading_angle, center):
    """ Calculate 3D bounding box corners from its parameterization.
    Input:
        box_size: tuple of (length,wide,height)
        heading_angle: rad scalar, clockwise from pos x axis
        center: tuple of (x,y,z)
    Output:
        corners_3d: numpy array of shape (8,3) for 3D box cornders
    """

    def roty(t):
        c = np.cos(t)
        s = np.sin(t)
        return np.array([[c, 0, s],
                         [0, 1, 0],
                         [-s, 0, c]])

    R = roty(heading_angle)
    l, w, h = box_size
    x_corners = [l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2, -l / 2, -l / 2]
    y_corners = [h / 2, h / 2, h / 2, h / 2, -h / 2, -h / 2, -h / 2, -h / 2]
    z_corners = [w / 2, -w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2, w / 2]
    corners_3d = np.dot(R, np.vstack([x_corners, y_corners, z_corners]))
    corners_3d[0, :] = corners_3d[0, :] + center[0]
    corners_3d[1, :] = corners_3d[1, :] + center[1]
    corners_3d[2, :] = corners_3d[2, :] + center[2]
    corners_3d = np.transpose(corners_3d)
    return corners_3d
# / *
# 计算iou的方法,3D IoU caculate code for 3D object detection
# / ****************************************************************

# if __name__ == '__main__':
#     print('------------------')
#     # get_3d_box(box_size, heading_angle, center)
#     corners_3d_ground = get_3d_box((1.497255, 1.644981, 3.628938), -1.531692, (2.882992, 1.698800, 20.785644))
#     corners_3d_predict = get_3d_box((1.458242, 1.604773, 3.707947), -1.549553, (2.756923, 1.661275, 20.943280))
    # (IOU_3d, IOU_2d) = box3d_iou(corners_3d_predict, corners_3d_ground)
    # print(IOU_3d, IOU_2d)  # 3d IoU/ 2d IoU of BEV(bird eye's view)

if __name__=='__main__':
    # dets = np.array([[39.78967, 25.2866, 4.30937,  1.75163,  179*np.pi/180, -3.96833, 1.61235, 2]]) #X,Y,L,W,角度,Z,H,class
    # trackers = np.array([[43.22218, 25.21163, 4.41272, 1.78851, 179*np.pi/180, -3.98864, 1.63135, 2]]) #X,Y,L,W,角度,Z,H,class
    # dets = [39.78967, 25.2866, 4.30937, 1.75163, 179, -3.96833, 1.61235, 2]  # X,Y,L,W,角度,Z,H,class
    # trackers = [43.22218, 25.21163, 4.41272, 1.78851, 179, -3.98864, 1.63135, 2]  # X,Y,L,W,角度,Z,H,class
    # dets = [-25.52552, -17.89754, 4.47355, 1.86836, 85.647, -4.42247, 1.76923, 2]  # X,Y,L,W,角度,Z,H,class
    # trackers = [-22.51646, -18.18984, 4.64129, 1.80820, 88.055, -4.45095, 1.63135, 2]  # X,Y,L,W,角度,Z,H,class
    dets = [481.665, 2180.282, 435.585, 196.646, 179, -390.936, 165.677, 2]  # X,Y,L,W,角度,Z,H,class
    trackers = [468.854, 2248.347, 486.644, 210.970, 179, -405.371, 171.215, 2]  # X,Y,L,W,角度,Z,H,class

    # dets = [30.3166524, 7.80281822, 1.96646357, 4.35585070, 91, -3.90935636, 1.65875, 2]  # X,Y,L,W,角度,Z,H,class
    # trackers = [30.1885353, 8.48347468, 2.10970154, 4.86643988, 91, -4.05371189, 1.61644, 2]  # X,Y,L,W,角度,Z,H,class

    dets[0] = dets[0] / 100
    dets[1] = dets[1] / 100
    dets[2] = dets[2] / 100
    dets[3] = dets[3] / 100
    dets[5] = dets[5] / 100
    dets[6] = dets[6] / 100
    trackers[0] = trackers[0] / 100
    trackers[1] = trackers[1] / 100
    trackers[2] = trackers[2] / 100
    trackers[3] = trackers[3] / 100
    trackers[5] = trackers[5] / 100
    trackers[6] = trackers[6] / 100

    # trackers = np.array([[5.78482246, 7.60582113, 1.82195735, 4.64982462, 186.89111195, 1, 2]])
    IOU = Iou_Two_Box(dets, trackers)
    #
    print("IOU=", IOU)
    # list1 = {}
    # list1[1] = [1,2,3,4]
    # list1[2] = [2,5,6,2,35,6]
    # list1[3] = [2,6,4,2,6,2,6]
    # for j in list1.keys():
    #     for i in range(len(list1[j])):
    #         list1[j][i] = 0
    #     for n in list1.keys():
    #         list1[j].append(100)
    #         list1[j].append(100)
    # for j in list1.keys():
    #     print(list1[j])

    # dit = {}
    # dit[1] = [1, 2, 3, 4, 5, 6]
    # dit[2] = [1, 2, 3, 4, 5, 6]
    # dit[3] = [1, 2, 3, 4, 5, 6]
    # for i in dit.keys():
    #     for j in range(len(dit[i])):
    #         dit[i].append(dit[i][j])
    #         dit[i][j] = 0
    # print(dit)
    # pos = [1, 2, 3, 4, 5]
    # pos1 = pos
    # pos1 = pos[0]
    # print(pos1[0], pos1[1])