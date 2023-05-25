#
# """
#     SORT: A Simple, Online and Realtime Tracker
#     Copyright (C) 2016 Alex Bewley alex@dynamicdetection.com
#
#     This program is free software: you can redistribute it and/or modify
#     it under the terms of the GNU General Public License as published by
#     the Free Software Foundation, either version 3 of the License, or
#     (at your option) any later version.
#
#     This program is distributed in the hope that it will be useful,
#     but WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#     GNU General Public License for more details.
#
#     You should have received a copy of the GNU General Public License
#     along with this program.  If not, see <http://www.gnu.org/licenses/>.
# """
# from __future__ import print_function
# from numba import jit
# import numpy as np
# import time
# import math
# from sklearn.utils.linear_assignment_ import linear_assignment
# from filterpy.kalman import KalmanFilter
# import second.core.box_np_ops as box_np_ops
# Rads_cov = 180 / math.pi
# #############对lidar计算iou值##############
# def rotate_nms_cc(dets,trackers):
#     trackers_corners = box_np_ops.center_to_corner_box2d(trackers[:, :2], trackers[:, 2:4], trackers[:, 4])
#     trackers_standup = box_np_ops.corner_to_standup_nd(trackers_corners)
#     dets_corners = box_np_ops.center_to_corner_box2d(dets[:, :2], dets[:, 2:4],dets[:, 4])
#     dets_standup = box_np_ops.corner_to_standup_nd(dets_corners)
#     standup_iou = box_np_ops.iou_jit(dets_standup, trackers_standup, eps=0.0)
#     return standup_iou
#
#
# @jit(nopython=True)
# def iou_jit(boxes, query_boxes, eps=0.0):
#     """calculate box iou. note that jit version runs 2x faster than cython in
#     my machine!
#     Parameters
#     ----------
#     boxes: (N, 4) ndarray of float
#     query_boxes: (K, 4) ndarray of float
#     Returns
#     -------
#     overlaps: (N, K) ndarray of overlap between boxes and query_boxes
#     """
#     N = boxes.shape[0]
#     K = query_boxes.shape[0]
#     overlaps = np.zeros((N, K), dtype=boxes.dtype)
#     for k in range(K):
#         box_area = ((query_boxes[k, 2] - query_boxes[k, 0] + eps) *
#                     (query_boxes[k, 3] - query_boxes[k, 1] + eps))
#         for n in range(N):
#             iw = (min(boxes[n, 2], query_boxes[k, 2]) - max(
#                 boxes[n, 0], query_boxes[k, 0]) + eps)
#             if iw > 0:
#                 ih = (min(boxes[n, 3], query_boxes[k, 3]) - max(
#                     boxes[n, 1], query_boxes[k, 1]) + eps)
#                 if ih > 0:
#                     ua = (
#                         (boxes[n, 2] - boxes[n, 0] + eps) *
#                         (boxes[n, 3] - boxes[n, 1] + eps) + box_area - iw * ih)
#                     overlaps[n, k] = iw * ih / ua
#     return overlaps
#
#
# class KalmanBoxTracker(object):
#   """
#   This class represents the internel state of individual tracked objects observed as bbox.
#   """
#   count = 0
#   def __init__(self,bbox):
#     """
#     Initialises a tracker using initial bounding box.
#     """
#     #define constant velocity model
#     self.kf = KalmanFilter(dim_x=7, dim_z=4)
#     self.kf.F = np.array([[1,0,0,0,1,0,0],[0,1,0,0,0,1,0],[0,0,1,0,0,0,1],
#                           [0,0,0,1,0,0,0],[0,0,0,0,1,0,0],[0,0,0,0,0,1,0],
#                           [0,0,0,0,0,0,1]])
#     self.kf.H = np.array([[1,0,0,0,0,0,0],[0,1,0,0,0,0,0],[0,0,1,0,0,0,0],[0,0,0,1,0,0,0]])
#
#     self.kf.R[2:,2:] *= 10.
#     self.kf.P[4:,4:] *= 1000 #give high uncertainty to the unobservable initial velocities
#     self.kf.P *= 10.
#     self.kf.Q[-1,-1] *= 0.01
#     self.kf.Q[4:,4:] *= 0.01
#     ####使用box_lidar,里面包含中心点，面积和航向角度
#     self.kf.x[:4] = bbox[:4].reshape((-1,1))
#     self.bbox=bbox###对应存储的状态值
#     self.time_since_update = 0
#     self.id = KalmanBoxTracker.count
#     KalmanBoxTracker.count += 1
#     self.history = []
#     self.angle_box=[]#####对应history
#     self.hits = 0
#     self.hit_streak = 0
#     self.age = 0 #####表示跟踪目标可以存活的时间（帧数）
#     ################追踪目标的速度################
#     self.speed=0
#     self.center_angle=0
#   def update(self,bbox):
#     """
#     Updates the state vector with observed bbox.
#     """
#     self.time_since_update = 0 ####表示距离上一次更新后没有再匹配更新的次数
#     self.history = []
#     # self.angle_box=[]
#     self.hits += 1 ###表示在age时间内其能够匹配上的次数
#     self.hit_streak += 1##表示连续匹配上的次数，从第一次开始
#     ####使用box_lidar,里面包含中心点，面积和航向角度
#     self.kf.update(bbox[:4].reshape((-1,1)))
#     self.bbox=bbox
#     ####速度的计算###########
#     delta_x = bbox[0] - self.kf.x[0]
#     delta_y = bbox[1] - self.kf.x[1]
#     theta = self.kf.x[4]
#     if len(self.angle_box)>=4:
#         delta_x1 = self.angle_box[-1][0] - self.angle_box[-4][0]
#         delta_y1=self.angle_box[-1][1] - self.angle_box[-4][1]
#         length = (delta_x1 ** 2 + delta_y1 ** 2) ** 0.5
#         # speed = length / 0.4
#         speed = length / 0.3
#
#     else:
#         delta_x1 = delta_x[0]
#         delta_y1 =delta_y[0]
#         length = (delta_x1 ** 2 + delta_y1 ** 2) ** 0.5
#         speed = length / 0.1
#
#     if speed>0.5 and len(self.angle_box)>=4:
#         center_angle = math.acos(delta_x1 / length) * Rads_cov - 180
#         if delta_y1 > 0:
#             self.center_angle = 90 - center_angle
#         else:
#             self.center_angle = 90 - (360 - center_angle)
#         self.center_angle = (self.center_angle % 360)
#     else:
#         self.center_angle=1000
#
#
#     self.speed=speed
#
#   def predict(self):
#     """
#     Advances the state vector and returns the predicted bounding box estimate.
#     """
#     if((self.kf.x[6]+self.kf.x[2])<=0):
#       self.kf.x[6] *= 0.0
#     self.kf.predict()
#     self.age += 1
#     if(self.time_since_update>0):
#       self.hit_streak = 0
#     self.time_since_update += 1
#     ##########直接使用box_lidar#####不需要进行转换##########
#     output_history = self.kf.x[:4].reshape((1, 4))
#     self.angle_box.append(self.bbox)
#     self.history.append(output_history)
#
#     if len(self.angle_box)>10:
#         self.angle_box=self.angle_box[-10:]
#
#     return self.history[-1],self.angle_box[-1]
#
#   def get_state(self):
#     """
#     Returns the current bounding box estimate.
#     """
#     ##########直接使用box_lidar#####不需要进行转换##########
#     output_x=self.kf.x[:4].reshape((1,4))
#     # output_x = self.kf.x[:5].reshape((1, 5))
#     return output_x,self.bbox,self.speed,self.angle_box,self.center_angle
#
# def associate_detections_to_trackers(detections,trackers,iou_threshold = 0.1):
#   """
#   Assigns detections to tracked object (both represented as bounding boxes)
#
#   Returns 3 lists of matches, unmatched_detections and unmatched_trackers
#   """
#   if(len(trackers)==0):
#     return np.empty((0,2),dtype=int), np.arange(len(detections)), np.empty((0,5),dtype=int)
#
#     #######distance match ###############
#   det_xy = detections[:, :2]
#   trk_xy = trackers[:, :2]
#   distance_matrix = (np.reshape(np.sum(det_xy ** 2, axis=1), (det_xy.shape[0], 1)) + np.sum(trk_xy ** 2,axis=1) - 2 * det_xy.dot(trk_xy.T)) ** 0.5
#   distance_matrix = 1 - (distance_matrix / 160)
#
#   ######直接使用lidar部分的iou，这里直接使用矩阵就行#####
#   iou_matrix = rotate_nms_cc(detections,trackers)
#   final_matrix = 0.3 * distance_matrix + 0.7 *iou_matrix
#   # matched_indices = linear_assignment(-final_matrix)
#   matched_indices = linear_assignment(-iou_matrix)
#   unmatched_detections = []
#   for d,det in enumerate(detections):
#     if(d not in matched_indices[:,0]):
#       unmatched_detections.append(d)
#   unmatched_trackers = []
#   for t,trk in enumerate(trackers):
#     if(t not in matched_indices[:,1]):
#       unmatched_trackers.append(t)
#
#   #filter out matched with low IOU
#   matches = []
#   for m in matched_indices:
#     if(iou_matrix[m[0],m[1]]<iou_threshold):
#       unmatched_detections.append(m[0])
#       unmatched_trackers.append(m[1])
#     else:
#       matches.append(m.reshape(1,2))
#   if(len(matches)==0):
#     matches = np.empty((0,2),dtype=int)
#   else:
#     matches = np.concatenate(matches,axis=0)
#   return matches, np.array(unmatched_detections), np.array(unmatched_trackers)
#
# class Sort(object):
#   def __init__(self,max_age=4,min_hits=2):
#     """
#     Sets key parameters for SORT
#     """
#     self.max_age = max_age
#     self.min_hits = min_hits
#     self.trackers = []
#     self.frame_count = 0
#   def update(self,dets):
#     """
#     Params:
#       dets - a numpy array of detections in the format [[x1,y1,x2,y2,score],[x1,y1,x2,y2,score],...]
#     Requires: this method must be called once for each frame even with empty detections.参数输入为
#     x,y,l,角度，z坐标，高度
#     Returns the a similar array, where the last column is the object ID.
#
#     NOTE: The number of objects returned may differ from the number of detections provided.
#     """
#
#     self.frame_count += 1
#     heading_mode=1
#     #get predicted locations from existing trackers.
#     trks = np.zeros((len(self.trackers),5))
#     to_del = []
#     ret = []
#     for t,trk in enumerate(trks):
#       pos,bbox = self.trackers[t].predict()
#       pos=pos[0]
#       trk[:] = [pos[0], pos[1], pos[2], pos[3], bbox[4]]
#       if(np.any(np.isnan(pos))):
#         to_del.append(t)
#     trks = np.ma.compress_rows(np.ma.masked_invalid(trks))
#     for t in reversed(to_del):
#       self.trackers.pop(t)
#     matched, unmatched_dets, unmatched_trks = associate_detections_to_trackers(dets,trks)
#     #update matched trackers with assigned detections
#     for t,trk in enumerate(self.trackers):
#       if(t not in unmatched_trks):
#         d = matched[np.where(matched[:,1]==t)[0],0]
#         trk.update(dets[d,:][0])
#     #create and initialise new trackers for unmatched detections
#     for i in unmatched_dets:
#         trk = KalmanBoxTracker(dets[i,:])
#         self.trackers.append(trk)
#     i = len(self.trackers)
#     for trk in reversed(self.trackers):
#         d,x_temp,trk_speed,trk_angle_box,center_angle = trk.get_state()
#         # print(f'len of cache is {len(trk_angle_box)}')
#         d=d[0]
#         if center_angle<0:
#             center_angle+=360
#         if((trk.time_since_update < self.max_age) and (trk.hits >= self.min_hits or self.frame_count <= self.min_hits)):
#         # if ((trk.time_since_update < self.max_age) and (trk.hits >= self.min_hits) and trk.speed > 1):
#           #########使用5帧中最常出现的角度值#############
#           if heading_mode==1:
#               if len(trk_angle_box)!=0:
#                   head_angle=trk_angle_box[-1][4]
#                   count_head=0
#                   more_25=[]
#                   less_25=[]
#                   for k in trk_angle_box:
#                       if abs(k[4]-head_angle)>90:
#                           count_head+=1
#                           temp_head=k[4]
#                           more_25.append(k[4])
#                       else:
#                           less_25.append(k[4])
#
#                   if count_head>=0.5*len(trk_angle_box):
#                       head_final = sum(more_25)/len(more_25)
#                       # head_final=temp_head
#                   else:
#                       head_final=sum(less_25)/len(less_25)
#                       # head_final=head_angle
#
#                   # if delta_x>0 and abs(head_final-90)<30:
#                   #     head_final=head_final+180
#                   # if delta_x>0 and abs(head_final-90)<15:
#                   #     head_final=head_final+180
#               else:
#                   head_final=x_temp[4]
#           else:
#               if len(trk_angle_box) != 0:
#                   head_final=trk_angle_box[-1][4]
#               else:
#                   head_final = x_temp[4]
#
#           #########insert angle for pedes and bike##3#
#           if trk_speed > 0.5 and (x_temp[7] in [1,3,4]) and center_angle!=1000:
#               head_final = center_angle
#
#           ########使用5帧中最常出现的角度值#############
#           d_conv = [d[0], d[1], d[2], d[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
#           # d_conv = [d[0], d[1], d[2], d[3], x_temp[4],x_temp[5],x_temp[6],x_temp[7]]
#           ret.append(np.concatenate((d_conv,[trk_speed],[trk.id+1], [d[0]])).reshape(1,-1)) # +1 as MOT benchmark requires positive
#         i -= 1
#
#         #remove dead tracklet
#         if(trk.time_since_update > self.max_age):
#           self.trackers.pop(i)
#     if(len(ret)>0):
#       return np.concatenate(ret)
#     return np.empty((0,5))







#
#
#
#
#
#
#
#
#
#
#

"""
    SORT: A Simple, Online and Realtime Tracker
    Copyright (C) 2016 Alex Bewley alex@dynamicdetection.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""
from __future__ import print_function
from numba import jit
import numpy as np
import time
import math
from sklearn.utils.linear_assignment_ import linear_assignment
from filterpy.kalman import KalmanFilter
import second.core.box_np_ops as box_np_ops
Rads_cov = 180 / math.pi
#############对lidar计算iou值##############
def rotate_nms_cc(dets,trackers):
    trackers_corners = box_np_ops.center_to_corner_box2d(trackers[:, :2], trackers[:, 2:4], trackers[:, 4])
    trackers_standup = box_np_ops.corner_to_standup_nd(trackers_corners)
    dets_corners = box_np_ops.center_to_corner_box2d(dets[:, :2], dets[:, 2:4],dets[:, 4])
    dets_standup = box_np_ops.corner_to_standup_nd(dets_corners)
    standup_iou = box_np_ops.iou_jit(dets_standup, trackers_standup, eps=0.0)
    return standup_iou


@jit(nopython=True)
def iou_jit(boxes, query_boxes, eps=0.0):
    """calculate box iou. note that jit version runs 2x faster than cython in
    my machine!
    Parameters
    ----------
    boxes: (N, 4) ndarray of float
    query_boxes: (K, 4) ndarray of float
    Returns
    -------
    overlaps: (N, K) ndarray of overlap between boxes and query_boxes
    """
    N = boxes.shape[0]
    K = query_boxes.shape[0]
    overlaps = np.zeros((N, K), dtype=boxes.dtype)
    for k in range(K):
        box_area = ((query_boxes[k, 2] - query_boxes[k, 0] + eps) *
                    (query_boxes[k, 3] - query_boxes[k, 1] + eps))
        for n in range(N):
            iw = (min(boxes[n, 2], query_boxes[k, 2]) - max(
                boxes[n, 0], query_boxes[k, 0]) + eps)
            if iw > 0:
                ih = (min(boxes[n, 3], query_boxes[k, 3]) - max(
                    boxes[n, 1], query_boxes[k, 1]) + eps)
                if ih > 0:
                    ua = (
                        (boxes[n, 2] - boxes[n, 0] + eps) *
                        (boxes[n, 3] - boxes[n, 1] + eps) + box_area - iw * ih)
                    overlaps[n, k] = iw * ih / ua
    return overlaps


class KalmanBoxTracker(object):
  """
  This class represents the internel state of individual tracked objects observed as bbox.
  """
  count = 0
  def __init__(self,bbox):
    """
    Initialises a tracker using initial bounding box.
    """
    #define constant velocity model
    self.kf = KalmanFilter(dim_x=7, dim_z=4)
    self.kf.F = np.array([[1,0,0,0,1,0,0],[0,1,0,0,0,1,0],[0,0,1,0,0,0,1],
                          [0,0,0,1,0,0,0],[0,0,0,0,1,0,0],[0,0,0,0,0,1,0],
                          [0,0,0,0,0,0,1]])
    self.kf.H = np.array([[1,0,0,0,0,0,0],[0,1,0,0,0,0,0],[0,0,1,0,0,0,0],[0,0,0,1,0,0,0]])

    self.kf.R[2:,2:] *= 10.
    self.kf.P[4:,4:] *= 1000 #give high uncertainty to the unobservable initial velocities
    self.kf.P *= 10.
    self.kf.Q[-1,-1] *= 0.01
    self.kf.Q[4:,4:] *= 0.01
    ####使用box_lidar,里面包含中心点，面积和航向角度
    self.kf.x[:4] = bbox[:4].reshape((-1,1))
    self.bbox=bbox###对应存储的状态值
    self.time_since_update = 0
    self.id = KalmanBoxTracker.count
    KalmanBoxTracker.count += 1
    self.history = []
    self.label_box = []
    self.head_angle = []
    self.l_rate = []
    self.kf_speed = []
    self.kf_speed_x = []
    self.kf_speed_y = []
    self.angle_box=[]#####对应history
    self.hits = 0
    self.hit_streak = 0
    self.age = 0 #####表示跟踪目标可以存活的时间（帧数）
    ################追踪目标的速度################
    self.speed = 0
    self.speed_x = 0
    self.speed_y = 0
    self.center_angle = 0
    #####*****是否采用曹工的方法*****#####
    self.use_c_way = 0
    #####*****曹工方法的航向角存储*****#####
    self.c_angle = []
    #####*****是否对卡尔曼预测的速度进行限制，防止检测框飞出的现象*****#####
    self.kf_vl_limit_mode = 0
    #####*****是否采用相邻框尺寸的均值作为当前检测框的大小*****#####
    self.correct_box_size_ave = 0
    #####*****是否采用相邻框尺寸作为当前检测框的大小*****#####
    self.correct_box_size_NN = 0
    #####**********航向角是否平滑**********#####
    self.head_angle_smooth_mode = 1
    #####*****存储更改的label信息*****#####
    self.label = 100
  def update(self,bbox):
    """
    Updates the state vector with observed bbox.
    """
    self.time_since_update = 0 ####表示距离上一次更新后没有再匹配更新的次数
    self.history = []
    # self.angle_box=[]
    self.hits += 1 ###表示在age时间内其能够匹配上的次数
    self.hit_streak += 1##表示连续匹配上的次数，从第一次开始
    ####使用box_lidar,里面包含中心点，面积和航向角度
    self.kf.update(bbox[:4].reshape((-1,1)))
    self.bbox=bbox

    if self.kf_vl_limit_mode == 1:
        kf_speed_x = self.kf.x[4]
        kf_speed_y = self.kf.x[5]
        kf_speed = (self.kf.x[4] * self.kf.x[4] + self.kf.x[5] * self.kf.x[5]) ** 0.5

        ########kalman速度的计算########
        self.kf_speed.append(kf_speed[0])
        self.kf_speed_x.append(kf_speed_x[0])
        self.kf_speed_y.append(kf_speed_y[0])
        if len(self.kf_speed) > 10:
            self.kf_speed = self.kf_speed[-10:]
            self.kf_speed_x = self.kf_speed_x[-10:]
            self.kf_speed_y = self.kf_speed_y[-10:]
    ##########对卡尔曼的估计速度进行限制，加速度不得超过5m/s2    状态量包括加速度时候可以对加速度分析
        if len(self.kf_speed_x) > 1:
            diff_x = self.kf_speed_x[-1] - self.kf_speed_x[-2]
            if abs(diff_x) > 0.35:
                self.kf_speed_x[-1] = self.kf_speed_x[-2] + 0.1 * 3.5 * diff_x / abs(diff_x)
                self.kf.x[4] = self.kf_speed_x[-2] + 0.1 * 3.5 * diff_x / abs(diff_x)
            if abs(self.kf.x[4]) < 0.5:
                self.kf.x[4] = 0
            diff_y = self.kf_speed_y[-1] - self.kf_speed_y[-2]
            if abs(diff_y) > 0.35:
                self.kf_speed_y[-1] = self.kf_speed_y[-2] + 0.1 * 3.5 * diff_y / abs(diff_y)
                self.kf.x[5] = self.kf_speed_y[-2] + 0.1 * 3.5 * diff_y / abs(diff_y)
            if abs(self.kf.x[5]) < 0.5:
                self.kf.x[5] = 0


    #######计算的平滑速度
    delta_x = bbox[0] - self.kf.x[0]
    delta_y = bbox[1] - self.kf.x[1]
    if len(self.angle_box)>=4:
        delta_x1 = self.angle_box[-1][0] - self.angle_box[-4][0]
        delta_y1=self.angle_box[-1][1] - self.angle_box[-4][1]
        length = (delta_x1 ** 2 + delta_y1 ** 2) ** 0.5
        # speed = length / 0.4
        speed = length / 0.3
        speed_x = delta_x1 / 0.3
        speed_y = delta_y1 / 0.3

    else:
        delta_x1 = delta_x[0]
        delta_y1 =delta_y[0]
        length = (delta_x1 ** 2 + delta_y1 ** 2) ** 0.5
        speed = length / 0.1
        speed_x = delta_x1 / 0.1
        speed_y = delta_y1 / 0.1


#####*****利用相邻帧位置计算航向角并存储*****#####
    if len(self.angle_box) > 1:
        c_dx = self.angle_box[-1][0] - self.angle_box[-2][0]
        c_dy = self.angle_box[-1][1] - self.angle_box[-2][1]
        c_len = (c_dx ** 2 + c_dy ** 2) ** 0.5
        c_angle = math.acos(c_dx / c_len) * Rads_cov - 180
        if c_dy > 0:
            c_angle = 90 - c_angle
        else:
            c_angle = 90 - (360 - c_angle)
        c_angle = c_angle % 360
    else:
        c_angle = self.bbox[4]
    self.c_angle.append(c_angle)
    if len(self.c_angle) > 10:
        self.c_angle = self.c_angle[-10:]

######计算的平滑航向角   相邻4帧计算平滑的速度和航向角
    if self.head_angle_smooth_mode == 1:
        if speed>0.5 and len(self.angle_box)>=4:
        # if len(self.angle_box) >= 4:
            center_angle = math.acos(delta_x1 / length) * Rads_cov - 180
            if delta_y1 > 0:
                self.center_angle = 90 - center_angle
            else:
                self.center_angle = 90 - (360 - center_angle)
        else:
            self.center_angle=1000
    #######不使用平滑的航向角
    else:
        self.center_angle = self.bbox[4]
    self.center_angle = (self.center_angle % 360)
    self.speed=speed
    self.speed_x = speed_x
    self.speed_y = speed_y

  def predict(self):
    """
    Advances the state vector and returns the predicted bounding box estimate.
    """
    if((self.kf.x[6]+self.kf.x[2])<=0):
      self.kf.x[6] *= 0.0
    self.kf.predict()
    self.age += 1
    if(self.time_since_update>0):
      self.hit_streak = 0
    self.time_since_update += 1
    ########30##直接使用box_lidar#####不需要进行转换##########
    output_history = self.kf.x[:4].reshape((1, 4))
    self.angle_box.append(self.bbox)
    self.label_box.append(self.bbox)
    self.history.append(output_history)
    # self.l_rate.append(self.kf.x)

    if len(self.angle_box)>10:
        self.angle_box=self.angle_box[-10:]
        self.label_box = self.label_box[-10:]

    ######分析历史帧的标签，取次数最多的label，并赋对应的box尺寸
    label_num = np.array([0, 0, 0, 0, 0, 0, 0])
    size_x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    size_y = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    size_z = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    angle_temp = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    if len(self.label_box) > 0:
        for his in self.label_box:
            if his[7] == 0:
                if self.correct_box_size_ave == 1:
                    label_num[0] += 1
                    #####*****记录目标尺寸和*****#####
                    size_x[0] += his[2]
                    size_y[0] += his[3]
                    size_z[0] += his[6]
                    angle_temp[0] += his[4]
                elif self.correct_box_size_NN == 1:
                    label_num[0] += 1
                    #####*****记录最临近同label目标的尺寸*****#####
                    size_x[0] = his[2]
                    size_y[0] = his[3]
                    size_z[0] = his[6]
                    angle_temp[0] = his[4]
                else:
                    label_num[0] += 1
            elif his[7] == 1 or his[7] == 3:
                # elif his[7] == 1:
                if self.correct_box_size_ave == 1:
                    label_num[1] += 1
                    #####*****记录目标尺寸和*****#####
                    size_x[1] += his[2]
                    size_y[1] += his[3]
                    size_z[1] += his[6]
                    angle_temp[1] += his[4]
                elif self.correct_box_size_NN == 1:
                    #####*****记录最临近同label目标的尺寸*****#####
                    label_num[1] += 1
                    size_x[1] = his[2]
                    size_y[1] = his[3]
                    size_z[1] = his[6]
                    angle_temp[1] = his[4]
                else:
                    label_num[1] += 1
            elif his[7] == 2:
                if self.correct_box_size_ave == 1:
                    label_num[2] += 1
                    #####*****记录目标尺寸和*****#####
                    size_x[2] += his[2]
                    size_y[2] += his[3]
                    size_z[2] += his[6]
                    angle_temp[2] += his[4]
                elif self.correct_box_size_NN == 1:
                    label_num[2] += 1
                    #####*****记录最临近同label目标的尺寸*****#####
                    size_x[2] = his[2]
                    size_y[2] = his[3]
                    size_z[2] = his[6]
                    angle_temp[2] = his[4]
                else:
                    label_num[2] += 1
            elif his[7] == 4:
                if self.correct_box_size_ave == 1:
                    label_num[4] += 1
                    #####*****记录目标尺寸和*****#####
                    size_x[4] += his[2]
                    size_y[4] += his[3]
                    size_z[4] += his[6]
                    angle_temp[4] += his[4]
                elif self.correct_box_size_NN == 1:
                    #####*****记录最临近同label目标的尺寸*****#####
                    label_num[4] += 1
                    size_x[4] = his[2]
                    size_y[4] = his[3]
                    size_z[4] = his[6]
                    angle_temp[4] = his[4]
                else:
                    label_num[4] += 1
            elif his[7] == 5:
                if self.correct_box_size_ave == 1:
                    label_num[5] += 1
                    #####*****记录目标尺寸和*****#####
                    size_x[5] += his[2]
                    size_y[5] += his[3]
                    size_z[5] += his[6]
                    angle_temp[5] += his[4]
                elif self.correct_box_size_NN == 1:
                    #####*****记录最临近同label目标的尺寸*****#####
                    label_num[5] += 1
                    size_x[5] = his[2]
                    size_y[5] = his[3]
                    size_z[5] = his[6]
                    angle_temp[5] = his[4]
                else:
                    label_num[5] += 1
            elif his[7] == 6:
                if self.correct_box_size_ave == 1:
                    label_num[6] += 1
                    #####*****记录目标尺寸和*****#####
                    size_x[6] += his[2]
                    size_y[6] += his[3]
                    size_z[6] += his[6]
                    angle_temp[6] += his[4]
                elif self.correct_box_size_NN == 1:
                    #####*****记录最临近同label目标的尺寸*****#####
                    label_num[6] += 1
                    size_x[6] = his[2]
                    size_y[6] = his[3]
                    size_z[6] = his[6]
                    angle_temp[6] = his[4]
                else:
                    label_num[6] += 1
        more_label = np.argmax(label_num)

        # self.bbox[7] = more_label
        self.label = more_label
        if self.correct_box_size_ave == 1:
            #####******采用大小均值赋给当前目标*****#####
            self.bbox[2] = size_x[more_label] / label_num[more_label]
            self.bbox[3] = size_y[more_label] / label_num[more_label]
            self.bbox[6] = size_z[more_label] / label_num[more_label]
            # self.bbox[4] = angle_temp[more_label] / label_num[more_label]
            #####*****采用最临近目标的尺寸作为当前目标的尺寸*****#####
        if self.correct_box_size_NN == 1:
            self.bbox[2] = size_x[more_label]
            self.bbox[3] = size_y[more_label]
            self.bbox[6] = size_z[more_label]
            # self.bbox[4] = angle_temp[more_label]
    else:
        self.label = self.bbox[7]

    return self.history[-1],self.angle_box[-1]

  def get_state(self):
    """
    Returns the current bounding box estimate.
    """
    ##########直接使用box_lidar#####不需要进行转换##########
    output_x=self.kf.x[:4].reshape((1,4))
    return output_x,self.bbox,self.speed,self.angle_box,self.center_angle,self.speed_x,self.speed_y

def associate_detections_to_trackers(detections,trackers,iou_threshold = 0.1):
  """
  Assigns detections to tracked o192.168.3.181bject (both represented as bounding boxes)

  Returns 3 lists of matches, unmatched_detections and unmatched_trackers
  """
  if(len(trackers)==0):
    return np.empty((0,2),dtype=int), np.arange(len(detections)), np.empty((0,5),dtype=int)

    #######distance match ###############
  det_xy = detections[:, :2]
  trk_xy = trackers[:, :2]
  distance_matrix = (np.reshape(np.sum(det_xy ** 2, axis=1), (det_xy.shape[0], 1)) + np.sum(trk_xy ** 2,axis=1) - 2 * det_xy.dot(trk_xy.T)) ** 0.5
  distance_matrix = 1 - (distance_matrix / 160)

  ######直接使用lidar部分的iou，这里直接使用矩阵就行#####
  iou_matrix = rotate_nms_cc(detections,trackers)
  final_matrix = 0.3 * distance_matrix + 0.7 *iou_matrix
  # matched_indices = linear_assignment(-final_matrix)
  matched_indices = linear_assignment(-iou_matrix)
  unmatched_detections = []
  for d,det in enumerate(detections):
    if(d not in matched_indices[:,0]):
      unmatched_detections.append(d)
  unmatched_trackers = []
  for t,trk in enumerate(trackers):
    if(t not in matched_indices[:,1]):
      unmatched_trackers.append(t)

  #filter out matched with low IOU
  matches = []
  for m in matched_indices:
    if(iou_matrix[m[0],m[1]]<iou_threshold):
      unmatched_detections.append(m[0])
      unmatched_trackers.append(m[1])
    else:
      matches.append(m.reshape(1,2))
  if(len(matches)==0):
    matches = np.empty((0,2),dtype=int)
  else:
    matches = np.concatenate(matches,axis=0)
  return matches, np.array(unmatched_detections), np.array(unmatched_trackers)

class Sort(object):
  def __init__(self,max_age=4,min_hits=2):
    """
    Sets key parameters for SORT
    """
    self.max_age = max_age
    self.min_hits = min_hits
    self.trackers = []
    self.frame_count = 0
    #####*****判断动态目标和静态目标的阈值(暂时觉得速度作为判断条件不太靠谱，采用位移作为判断条件)*****#####
    self.dis_thresh = 1.0
  def update(self,dets):
    """
    Params:
      dets - a numpy array of detections in the format [[x1,y1,x2,y2,score],[x1,y1,x2,y2,score],...]
    Requires: this method must be called once for each frame even with empty detections.参数输入为
    x,y,l,角度，z坐标，高度
    Returns the a similar array, where the last column is the object ID.

    NOTE: The number of objects returned may diffkf_speed_xer from the number of detections provided.
    """

    self.frame_count += 1

    #get predicted locations from existing trackers.
    trks = np.zeros((len(self.trackers),5))
    to_del = []
    ret = []
    speed = []
    for t,trk in enumerate(trks):
      pos,bbox = self.trackers[t].predict()#pos:x,y,w,l       bbox:x,y,w,l,angle,z,h,label,score
      pos=pos[0]
      trk[:] = [pos[0], pos[1], pos[2], pos[3], bbox[4]]
      if(np.any(np.isnan(pos))):
        to_del.append(t)
    trks = np.ma.compress_rows(np.ma.masked_invalid(trks))
    for t in reversed(to_del):
      self.trackers.pop(t)
    matched, unmatched_dets, unmatched_trks = associate_detections_to_trackers(dets, trks)#matched, unmatched_dets, unmatched_trks是保存dets, trks的索引值
    # for i in range(len(unmatched_dets)):
    #     print("unmatched_dets",dets[unmatched_dets[i]])

    #update matched trackers with assigned detections
    for t,trk in enumerate(self.trackers):        # if ((trk.time_since_update < self.max_age) and (trk.hits >= self.min_hits or self.frame_count <= self.min_hits)):
      if(t not in unmatched_trks):
        d = matched[np.where(matched[:,1]==t)[0],0]
        trk.update(dets[d,:][0])
      elif trk.kf_vl_limit_mode == 1 and len(trk.kf_speed_x) > 1:
      # elif len(trk.kf_speed_x) > 1:
        trk.kf.x[4] = trk.kf_speed_x[-2]
        trk.kf.x[5] = trk.kf_speed_y[-2]
    #create and initialise new trackers for unmatched detections
    for i in unmatched_dets:
        trk = KalmanBoxTracker(dets[i,:])
        self.trackers.append(trk)
    i = len(self.trackers)
    ilen = len(self.trackers)
    j = len(matched)
    th = ilen - j
    num = 0
    if ilen > len(dets):
        print('more trackers')
    for trk in reversed(self.trackers):
        d, x_temp, trk_speed, trk_angle_box, center_angle, trk_speedx, trk_speedy = trk.get_state()
        d = d[0]
        if center_angle < 0:
            center_angle += 360

        num += 1
        if num > th:
            speed.append(np.concatenate(([trk_speedx], [trk_speedy])).reshape(1, -1))

        # print('trk_speed', speed)
        if ((trk.time_since_update < self.max_age) and (trk.hits >= self.min_hits or self.frame_count <= self.min_hits)):
            if trk.use_c_way == 1:
                if len(trk.c_angle) > 10:
                    c_angle = np.array(trk.c_angle)
                    ave_angle = (np.sum(c_angle)-np.max(c_angle)-np.min(c_angle)) / 8
                else:
                    ave_angle = trk.center_angle
                ave_angle = ave_angle % 360
                if trk.time_since_update > 0:
                    if trk.label != 100:
                        d_conv = [d[0], d[1], x_temp[2], x_temp[3], ave_angle, x_temp[5], x_temp[6], trk.label]
                    else:
                        d_conv = [d[0], d[1], x_temp[2], x_temp[3], ave_angle, x_temp[5], x_temp[6], x_temp[7]]
                    ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1], [trk_speedx], [trk_speedy])).reshape(1, -1))
                else:
                    if trk.label != 100:
                        d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], ave_angle, x_temp[5], x_temp[6], trk.label]
                    else:
                        d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], ave_angle, x_temp[5], x_temp[6], x_temp[7]]
                    ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1], [trk_speedx], [trk_speedy])).reshape(1, -1))

            else:
                trk.l_rate.append(d)
                if len(trk.l_rate) > 10:
                    trk.l_rate = trk.l_rate[-10:]

                if trk_speed<0.5:
                    x_temp[0] = np.mean(np.array(trk.l_rate)[:, 0])
                    x_temp[1] = np.mean(np.array(trk.l_rate)[:, 1])
                    d[0] = np.mean(np.array(trk.l_rate)[:, 0])
                    d[1] = np.mean(np.array(trk.l_rate)[:, 1])


                ###line param
                region_1_y1 = d[1] - (-0.0532 * d[0] - 4.6637)
                region_1_y2 = d[1] - (-0.05147 * d[0] - 23.5824)
                region_2_y1 = d[1] - (-0.086558 * d[0] - 37.9651)
                region_3_y1 = d[1] - (-12.310246 * d[0] - 85.820473)
                region_3_y2 = d[1] - (21.37346 * d[0] - 40.0895032)
                region_4_y1 = d[1] - (16.289419 * d[0] - 13.162859)
                region_5_y1 = d[1] - (-0.08429 * d[0] - 40.767545)
                region_5_y2 = d[1] - (-0.0745971 * d[0] - 21.671528)
                region_6_y1 = d[1] - (-0.0881895 * d[0] - 8.541781)
                region_7_y1 = d[1] - (7.03442783 * d[0] - 3.9048982)
                region_7_y2 = d[1] - (6.831139 * d[0] - 43.3341)
                region_8_y1 = d[1] - (12.7822052 * d[0] - 120.40918)
                region_8_y2 = d[1] - (7.6707842 * d[0] - 112.956155)
                if region_1_y1 * region_1_y2 < 0 and d[0] > 30.2981:
                    head_final = 90
                    trk.head_angle.append(head_final)
                    # if x_temp[7] != 4 and trk.label != 4:
                    if x_temp[7] not in [1,3,4] and trk.label not in [1,3,4]:
                        if trk.time_since_update > 0:
                            # d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
                            if trk.label != 100:
                                d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
                            else:
                                d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
                            ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1], [trk_speedx], [trk_speedy])).reshape(1, -1))
                        else:
                            # d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
                            if trk.label != 100:
                                d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
                            else:
                                d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
                            ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1], [trk_speedx], [trk_speedy])).reshape(1, -1))
                elif region_2_y1 * region_1_y2 < 0 and d[0] > 30.2981:
                    head_final = 270
                    trk.head_angle.append(head_final)
                    # if x_temp[7] != 4 and trk.label != 4:
                    if x_temp[7] not in [1, 3, 4] and trk.label not in [1, 3, 4]:
                        if trk.time_since_update > 0:
                            # d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
                            if trk.label != 100:
                                d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
                            else:
                                d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
                            ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1], [trk_speedx], [trk_speedy])).reshape(1, -1))
                        else:
                            # d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
                            if trk.label != 100:
                                d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
                            else:
                                d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
                            ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1], [trk_speedx], [trk_speedy])).reshape(1, -1))
            # elif region_3_y1 * region_3_y2 < 0 and d[1] < -16.4494:
            #     elif region_3_y1 * region_3_y2 < 0 and d[1] < -44.1581:
            #         head_final = 180
            #         trk.head_angle.append(head_final)
            #         if x_temp[7] != 4 and trk.label != 4:
            #             if trk.time_since_update > 0:
            #                 # d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
            #                 if trk.label != 100:
            #                     d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
            #                 else:
            #                     d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
            #                 ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1])).reshape(1, -1))
            #             else:
            #                 # d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
            #                 if trk.label != 100:
            #                     d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
            #                 else:
            #                     d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
            #                 ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1])).reshape(1, -1))
            #     elif region_3_y2 * region_4_y1 < 0 and d[1] < -44.1581:
            #         head_final = 0
            #         trk.head_angle.append(head_final)
            #         if x_temp[7] != 4 and trk.label != 4:
            #             if trk.time_since_update > 0:
            #                 # d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
            #                 if trk.label != 100:
            #                     d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
            #                 else:
            #                     d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
            #                 ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1])).reshape(1, -1))
            #             else:
            #                 # d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
            #                 if trk.label != 100:
            #                     d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
            #                 else:
            #                     d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
            #                 ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1])).reshape(1, -1))
            #     elif region_5_y1 * region_5_y2 < 0 and d[0] < -22.3123:
                elif region_5_y1 * region_5_y2 < 0 and d[0] < -10:
                    head_final = 270
                    trk.head_angle.append(head_final)
                    # if x_temp[7] != 4 and trk.label != 4:
                    if x_temp[7] not in [1, 3, 4] and trk.label not in [1, 3, 4]:
                        if trk.time_since_update > 0:
                            # d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
                            if trk.label != 100:
                                d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
                            else:
                                d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
                            ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1], [trk_speedx], [trk_speedy])).reshape(1, -1))
                        else:
                            # d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
                            if trk.label != 100:
                                d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
                            else:
                                d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
                            ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1], [trk_speedx], [trk_speedy])).reshape(1, -1))
                elif region_5_y2 * region_6_y1 < 0 and d[0] < -22.3123:
                    head_final = 90
                    trk.head_angle.append(head_final)
                    # if x_temp[7] != 4 and trk.label != 4:
                    if x_temp[7] not in [1, 3, 4] and trk.label not in [1, 3, 4]:
                        if trk.time_since_update > 0:
                            # d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
                            if trk.label != 100:
                                d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
                            else:
                                d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
                            ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1], [trk_speedx], [trk_speedy])).reshape(1, -1))
                        else:
                            # d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
                            if trk.label != 100:
                                d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
                            else:
                                d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
                            ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1], [trk_speedx], [trk_speedy])).reshape(1, -1))
                elif region_7_y1 * region_7_y2 < 0 and d[1] > 8.79752:
                    head_final = 0
                    trk.head_angle.append(head_final)
                    # if x_temp[7] != 4 and trk.label != 4:
                    if x_temp[7] not in [1, 3, 4] and trk.label not in [1, 3, 4]:
                        if trk.time_since_update > 0:
                            # d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
                            if trk.label != 100:
                                d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
                            else:
                                d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
                            ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1], [trk_speedx], [trk_speedy])).reshape(1, -1))
                        else:
                            # d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
                            if trk.label != 100:
                                d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
                            else:
                                d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
                            ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1], [trk_speedx], [trk_speedy])).reshape(1, -1))
                elif region_8_y1 * region_8_y2 < 0 and d[1] > 8.79752:
                    head_final = 180
                    trk.head_angle.append(head_final)
                    # if x_temp[7] != 4 and trk.label != 4:
                    if x_temp[7] not in [1, 3, 4] and trk.label not in [1, 3, 4]:
                        if trk.time_since_update > 0:
                            # d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
                            if trk.label != 100:
                                d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
                            else:
                                d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
                            ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1], [trk_speedx], [trk_speedy])).reshape(1, -1))
                        else:
                            # d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
                            if trk.label != 100:
                                d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
                            else:
                                d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
                            ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1], [trk_speedx], [trk_speedy])).reshape(1, -1))
                else:
                    if len(trk_angle_box) != 0:
                        head_angle = trk_angle_box[-1][4]
                        count_head = 0
                        more_25 = []
                        less_25 = []
                        for k in trk_angle_box:
                            if abs(k[4] - head_angle) > 90:
                                count_head += 1
                                temp_head = k[4]
                                more_25.append(k[4])
                            else:
                                less_25.append(k[4])

                        if count_head >= 0.5 * len(trk_angle_box):
                            head_final = sum(more_25) / len(more_25)
                        else:
                            head_final = sum(less_25) / len(less_25)
                    else:
                        head_final = x_temp[4]

                # ########insert angle for pedes and bike##3#
                    if trk_speed > 0.5 and (x_temp[7] in [1, 3, 4]) and center_angle != 1000:
                        head_final = center_angle

                    ######solve the problem of instability of static target direction
                    head_final = head_final % 360
                    lane_std = 0

                    # # #####*****待转区域中车辆航向角的限定*****#####
                    # if region_1_y1 * region_1_y2 < 0 and d[1] > 15:
                    #     lane_std = 1
                    #     std_head_angle = 0
                    # elif region_3_y1 * region_3_y2 < 0 and d[1] < -5:
                    #     lane_std = 1
                    #     std_head_angle = 180

                    trk.head_angle.append(head_final)
                    if len(trk.head_angle) > 10:
                        trk.head_angle = trk.head_angle[-10:]

                    ######*******条件合并的判断语句  (new)*********############
                    if len(trk.l_rate) > 9:
                    # if len(trk.l_rate) > 4:
                        dis_x = trk.l_rate[-1][0] - trk.l_rate[-10][0]
                        dis_y = trk.l_rate[-1][1] - trk.l_rate[-10][1]
                        # dis_x = trk.l_rate[-1][0] - trk.l_rate[-5][0]
                        # dis_y = trk.l_rate[-1][1] - trk.l_rate[-5][1]
                        dis_len = (dis_x * dis_x + dis_y * dis_y) ** 0.5
                        dis_angle = math.acos(dis_x / dis_len) * Rads_cov - 180
                        if dis_y > 0:
                            dis_angle = 90 - dis_angle
                        else:
                            dis_angle = 90 - (360 - dis_angle)
                        dis_angle = (dis_angle % 360)
                        # if lane_std == 1 and abs(std_head_angle - dis_angle) > 100 and abs(std_head_angle - dis_angle) < 260:
                        #     dis_angle = std_head_angle
                        if dis_len > self.dis_thresh:
                            if lane_std == 1 and abs(std_head_angle - dis_angle) > 100 and abs(std_head_angle - dis_angle) < 260:
                                dis_angle = std_head_angle
                        # if dis_len > 0.5:
                            elif len(trk.head_angle) > 0 and abs(trk.head_angle[-1] - dis_angle) > 120 and abs(trk.head_angle[-1] - dis_angle) < 240:
                            # if len(trk.head_angle) > 0 and abs(trk.head_angle[-1] - dis_angle) > 120 and abs(trk.head_angle[-1] - dis_angle) < 240:
                        #     if len(trk.head_angle) > 1 and abs(trk.head_angle[-1] - dis_angle) > 90 and abs(trk.head_angle[-1] - dis_angle) < 270:
                                trk.head_angle[-1] += 180
                                trk.head_angle[-1] = trk.head_angle[-1] % 360
                            #####*****解决航向角横摆90度的问题*****#####
                            elif len(trk.head_angle) > 1 and abs(trk.head_angle[-1] - dis_angle) > 60 and abs(trk.head_angle[-1] - dis_angle) < 120:
                                trk.head_angle[-1] = dis_angle
                            elif len(trk.head_angle) > 1 and abs(trk.head_angle[-1] - dis_angle) > 240 and abs(trk.head_angle[-1] - dis_angle) < 300:
                                trk.head_angle[-1] = dis_angle
                        else:
                            if lane_std == 1 and abs(trk.head_angle[-1] - std_head_angle) > 100 and abs(trk.head_angle[-1] - std_head_angle) < 260:
                                # trk.head_angle[-1] += 180
                                trk.head_angle[-1] = std_head_angle
                                trk.head_angle[-1] = trk.head_angle[-1] % 360
                            # if abs(trk.head_angle[-1] - trk.head_angle[-2]) > 130 and abs(trk.head_angle[-1] - trk.head_angle[-2]) < 230:
                            elif len(trk.head_angle) > 0 and abs(trk.head_angle[-1] - trk.head_angle[-2]) > 100 and abs(trk.head_angle[-1] - trk.head_angle[-2]) < 260:
                            # if len(trk.head_angle) > 1 and abs(trk.head_angle[-1] - trk.head_angle[-2]) > 120 and abs(trk.head_angle[-1] - trk.head_angle[-2]) < 240:
                                trk.head_angle[-1] += 180
                                trk.head_angle[-1] = trk.head_angle[-1] % 360
                            elif len(trk.head_angle) > 1 and abs(trk.head_angle[-1] - trk.head_angle[-2]) > 60 and abs(trk.head_angle[-1] - trk.head_angle[-2]) < 120:
                                trk.head_angle[-1] = trk.head_angle[-2]
                            elif len(trk.head_angle) > 1 and abs(trk.head_angle[-1] - trk.head_angle[-2]) > 240 and abs(trk.head_angle[-1] - trk.head_angle[-2]) < 300:
                                trk.head_angle[-1] = trk.head_angle[-2]
                        head_final = trk.head_angle[-1]

                        # x_temp[0] = np.mean(trk.l_rate[:,0])
                        # x_temp[1] = np.mean(trk.l_rate[:,1])
                        # x_temp[0] = np.mean(np.array(trk.l_rate)[:, 0])
                        # x_temp[1] = np.mean(np.array(trk.l_rate)[:, 1])
                        # #####*****只显示已经调整过航向角的目标*****#####
                        # #####*****有的目标速度很快但是识别为person，解决此类问题  不显示和更改标签两种方式*****#####
                        # #####*****不显示*****#####
                        # if trk.speed > 3 and trk.label == 4:
                        #     continue
                        # #####*****更改标签*****#####
                        # if trk.speed > 3 and trk.label == 4:
                        #     trk.label = 3

                        if trk.time_since_update > 0:
                            # d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
                            if trk.label != 100:
                                d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
                            else:
                                d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
                            ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1], [trk_speedx], [trk_speedy])).reshape(1, -1))
                        else:
                            # d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
                            if trk.label != 100:
                                d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
                            else:
                                d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
                            ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1], [trk_speedx], [trk_speedy])).reshape(1, -1))
                    else:
                        #####*****显示所有检测到的和预测的目标*****#####
                        if trk.time_since_update > 0:
                            # d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
                            if trk.label != 100:
                                d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
                            else:
                                d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
                            ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1], [trk_speedx], [trk_speedy])).reshape(1, -1))
                        else:
                            # d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
                            if trk.label != 100:
                                d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
                            else:
                                d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
                            ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1], [trk_speedx], [trk_speedy])).reshape(1, -1))
        i -= 1
                            #d_conv:x,y,w,l,angle,z,h,label
        #remove dead tracklet
        if(trk.time_since_update > self.max_age):
            self.trackers.pop(i)

    for ret_i in range(len(ret)):
        ret[ret_i][0][10] = 0
        ret[ret_i][0][11] = 0

    if(len(ret)>0):
        if(len(speed)>0):
            return np.concatenate(ret) #, np.concatenate(speed), matched, unmatched_dets, unmatched_trks,dets, trks
        else:
            return np.concatenate(ret)
    else:
        return None, None, None, None, None, None, None
    return np.empty((0,5))










#
# """
#     SORT: A Simple, Online and Realtime Tracker
#     Copyright (C) 2016 Alex Bewley alex@dynamicdetection.com
#
#     This program is free software: you can redistribute it and/or modify
#     it under the terms of the GNU General Public License as published by
#     the Free Software Foundation, either version 3 of the License, or
#     (at your option) any later version.
#
#     This program is distributed in the hope that it will be useful,
#     but WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#     GNU General Public License for more details.
#
#     You should have received a copy of the GNU General Public License
#     along with this program.  If not, see <http://www.gnu.org/licenses/>.
# """
# from __future__ import print_function
# from numba import jit
# import numpy as np
# import time
# import math
# from sklearn.utils.linear_assignment_ import linear_assignment
# from filterpy.kalman import KalmanFilter
# import second.core.box_np_ops as box_np_ops
# Rads_cov = 180 / math.pi
# #############对lidar计算iou值##############
# def rotate_nms_cc(dets,trackers):
#     trackers_corners = box_np_ops.center_to_corner_box2d(trackers[:, :2], trackers[:, 2:4], trackers[:, 4])
#     trackers_standup = box_np_ops.corner_to_standup_nd(trackers_corners)
#     dets_corners = box_np_ops.center_to_corner_box2d(dets[:, :2], dets[:, 2:4],dets[:, 4])
#     dets_standup = box_np_ops.corner_to_standup_nd(dets_corners)
#     standup_iou = box_np_ops.iou_jit(dets_standup, trackers_standup, eps=0.0)
#     return standup_iou
#
#
# @jit(nopython=True)
# def iou_jit(boxes, query_boxes, eps=0.0):
#     """calculate box iou. note that jit version runs 2x faster than cython in
#     my machine!
#     Parameters
#     ----------
#     boxes: (N, 4) ndarray of float
#     query_boxes: (K, 4) ndarray of float
#     Returns
#     -------
#     overlaps: (N, K) ndarray of overlap between boxes and query_boxes
#     """
#     N = boxes.shape[0]
#     K = query_boxes.shape[0]
#     overlaps = np.zeros((N, K), dtype=boxes.dtype)
#     for k in range(K):
#         box_area = ((query_boxes[k, 2] - query_boxes[k, 0] + eps) *
#                     (query_boxes[k, 3] - query_boxes[k, 1] + eps))
#         for n in range(N):
#             iw = (min(boxes[n, 2], query_boxes[k, 2]) - max(
#                 boxes[n, 0], query_boxes[k, 0]) + eps)
#             if iw > 0:
#                 ih = (min(boxes[n, 3], query_boxes[k, 3]) - max(
#                     boxes[n, 1], query_boxes[k, 1]) + eps)
#                 if ih > 0:
#                     ua = (
#                         (boxes[n, 2] - boxes[n, 0] + eps) *
#                         (boxes[n, 3] - boxes[n, 1] + eps) + box_area - iw * ih)
#                     overlaps[n, k] = iw * ih / ua
#     return overlaps
#
#
# class KalmanBoxTracker(object):
#   """
#   This class represents the internel state of individual tracked objects observed as bbox.
#   """
#   count = 0
#   def __init__(self,bbox):
#     """
#     Initialises a tracker using initial bounding box.
#     """
#     #define constant velocity model
#     self.kf = KalmanFilter(dim_x=7, dim_z=4)
#     self.kf.F = np.array([[1,0,0,0,1,0,0],[0,1,0,0,0,1,0],[0,0,1,0,0,0,1],
#                           [0,0,0,1,0,0,0],[0,0,0,0,1,0,0],[0,0,0,0,0,1,0],
#                           [0,0,0,0,0,0,1]])
#     self.kf.H = np.array([[1,0,0,0,0,0,0],[0,1,0,0,0,0,0],[0,0,1,0,0,0,0],[0,0,0,1,0,0,0]])
#
#     self.kf.R[2:,2:] *= 10.
#     self.kf.P[4:,4:] *= 1000 #give high uncertainty to the unobservable initial velocities
#     self.kf.P *= 10.
#     self.kf.Q[-1,-1] *= 0.01
#     self.kf.Q[4:,4:] *= 0.01
#     ####使用box_lidar,里面包含中心点，面积和航向角度
#     self.kf.x[:4] = bbox[:4].reshape((-1,1))
#     self.bbox=bbox###对应存储的状态值
#     self.bbox_temp = bbox
#     self.time_since_update = 0
#     self.id = KalmanBoxTracker.count
#     KalmanBoxTracker.count += 1
#     self.history = []
#     self.label_box = []
#     self.head_angle = []
#     self.l_rate = []
#     self.box_angle = []
#     self.cloud_angle = []
#     self.kf_angle = []
#     self.corner_speed = []
#     self.corner_speed_x = []
#     self.corner_speed_y = []
#     self.box_speed = []
#     self.box_speed_x = []
#     self.box_speed_y = []
#     self.kf_speed = []
#     self.kf_speed_x = []
#     self.kf_speed_y = []
#     self.angle_box=[]#####对应history
#     self.hits = 0
#     self.hit_streak = 0
#     self.age = 0 #####表示跟踪目标可以存活的时间（帧数）
#     ################追踪目标的速度################
#     self.speed=0
#     self.center_angle=0
#   def update(self, bbox):
#     """
#     Updates the state vector with observed bbox.
#     """
#     self.time_since_update = 0 ####表示距离上一次更新后没有再匹配更新的次数
#     self.history = []
#     # self.angle_box=[]
#     self.hits += 1 ###表示在age时间内其能够匹配上的次数
#     self.hit_streak += 1##表示连续匹配上的次数，从第一次开始
#     ####使用box_lidar,里面包含中心点，面积和航向角度
#     self.kf.update(bbox[:4].reshape((-1,1)))
#     self.bbox=bbox
#     self.bbox_temp=bbox
#
#
# # ##########kalman  sudu  pinghua
# #     kf_speed_x = self.kf.x[4]
# #     kf_speed_y = self.kf.x[5]
# #     kf_speed = (self.kf.x[4] * self.kf.x[4] + self.kf.x[5] * self.kf.x[5]) ** 0.5
#
# #     ########kalman速度的计算########
# #     self.kf_speed.append(kf_speed[0])
# #     self.kf_speed_x.append(kf_speed_x[0])
# #     self.kf_speed_y.append(kf_speed_y[0])
# #     if len(self.kf_speed) > 10:
# #         self.kf_speed = self.kf_speed[-10:]
# #         self.kf_speed_x = self.kf_speed_x[-10:]
# #         self.kf_speed_y = self.kf_speed_y[-10:]
# # ##########对卡尔曼的估计速度进行限制，加速度不得超过5m/s2    状态量包括加速度时候可以对加速度分析
# #     if len(self.kf_speed_x) > 1:
# #         diff_x = self.kf_speed_x[-1] - self.kf_speed_x[-2]
# #         if abs(diff_x) > 0.35:
# #             self.kf_speed_x[-1] = self.kf_speed_x[-2] + 0.1 * 3.5 * diff_x / abs(diff_x)
# #             self.kf.x[4] = self.kf_speed_x[-2] + 0.1 * 3.5 * diff_x / abs(diff_x)
# #         if abs(self.kf.x[4]) < 0.5:
# #             self.kf.x[4] = 0
# #         diff_y = self.kf_speed_y[-1] - self.kf_speed_y[-2]
# #         if abs(diff_y) > 0.35:
# #             self.kf_speed_y[-1] = self.kf_speed_y[-2] + 0.1 * 3.5 * diff_y / abs(diff_y)
# #             self.kf.x[5] = self.kf_speed_y[-2] + 0.1 * 3.5 * diff_y / abs(diff_y)
# #         if abs(self.kf.x[5]) < 0.5:
# #             self.kf.x[5] = 0
#
#
#     #######计算的平滑速度
#     delta_x = bbox[0] - self.kf.x[0]
#     delta_y = bbox[1] - self.kf.x[1]
#     if len(self.angle_box)>=4:
#         delta_x1 = self.angle_box[-1][0] - self.angle_box[-4][0]
#         delta_y1=self.angle_box[-1][1] - self.angle_box[-4][1]
#         length = (delta_x1 ** 2 + delta_y1 ** 2) ** 0.5
#         # speed = length / 0.4
#         speed = length / 0.3
#
#     else:
#         delta_x1 = delta_x[0]
#         delta_y1 =delta_y[0]
#         length = (delta_x1 ** 2 + delta_y1 ** 2) ** 0.5
#         speed = length / 0.1
# ######计算的平滑航向角   相邻4帧计算平滑的速度和航向角
#     if speed>0.5 and len(self.angle_box)>=4:
#         center_angle = math.acos(delta_x1 / length) * Rads_cov - 180
#         if delta_y1 > 0:
#             self.center_angle = 90 - center_angle
#         else:
#             self.center_angle = 90 - (360 - center_angle)
#         self.center_angle = (self.center_angle % 360)
#     else:
#         self.center_angle=1000
#
#
#     self.speed=speed
#
#     ######分析历史帧的标签，取次数最多的label，并赋对应的box尺寸
#     label_num = np.array([0, 0, 0, 0, 0])
#     if len(self.label_box) > 10:
#         for his in self.label_box:
#             if self.id == 104:
#                 print('****id 105 label***')
#                 print(self.label_box[-1][7])
#             if his[7] == 0:
#                 label_num[0] += 1
#             elif his[7] == 1 or his[7] == 3:
#             # if his[7] == 1 or his[7] == 3:
#             # elif his[7] == 1:
#                 label_num[1] += 1
#             elif his[7] == 2:
#             # if his[7] == 2:
#                 label_num[2] += 1
#             # elif his[7] == 3:
#                 # label_num[3] += 1
#             elif his[7] == 4:
#             # if his[7] == 4:
#                 label_num[4] += 1
#         more_label = np.argmax(label_num)
#
#         self.bbox[7] = more_label
#
#
#   def predict(self):
#     """
#     Advances the state vector and returns the predicted bounding box estimate.
#     """
#     if((self.kf.x[6]+self.kf.x[2])<=0):
#       self.kf.x[6] *= 0.0
#     self.kf.predict()
#     self.age += 1
#     if(self.time_since_update>0):
#       self.hit_streak = 0
#     self.time_since_update += 1
#     ##########直接使用box_lidar#####不需要进行转换##########
#     output_history = self.kf.x[:4].reshape((1, 4))
#     self.angle_box.append(self.bbox)
#     self.label_box.append(self.bbox)
#     self.history.append(output_history)
#     self.l_rate.append(self.kf.x)
#
#     if len(self.angle_box)>10:
#         self.angle_box=self.angle_box[-10:]
#         self.l_rate = self.l_rate[-10:]
#         self.label_box = self.label_box[-10:]
#
#     return self.history[-1],self.angle_box[-1]
#
#   def get_state(self):
#     """
#     Returns the current bounding box estimate.
#     """
#     ##########直接使用box_lidar#####不需要进行转换##########
#     output_x=self.kf.x[:4].reshape((1,4))
#     return output_x,self.bbox,self.speed,self.angle_box,self.center_angle
#     # return output_x, self.bbox, self.bbox_temp, self.speed, self.angle_box, self.center_angle
#
# def associate_detections_to_trackers(detections,trackers,iou_threshold = 0.1):
#   """
#   Assigns detections to tracked object (both represented as bounding boxes)
#
#   Returns 3 lists of matches, unmatched_detections and unmatched_trackers
#   """
#   if(len(trackers)==0):
#     return np.empty((0,2),dtype=int), np.arange(len(detections)), np.empty((0,5),dtype=int)
#
#     #######distance match ###############
#   det_xy = detections[:, :2]
#   trk_xy = trackers[:, :2]
#   distance_matrix = (np.reshape(np.sum(det_xy ** 2, axis=1), (det_xy.shape[0], 1)) + np.sum(trk_xy ** 2,axis=1) - 2 * det_xy.dot(trk_xy.T)) ** 0.5
#   distance_matrix = 1 - (distance_matrix / 160)
#
#   ######直接使用lidar部分的iou，这里直接使用矩阵就行#####
#   iou_matrix = rotate_nms_cc(detections,trackers)
#   final_matrix = 0.3 * distance_matrix + 0.7 *iou_matrix
#   # matched_indices = linear_assignment(-final_matrix)
#   matched_indices = linear_assignment(-iou_matrix)
#   unmatched_detections = []
#   for d,det in enumerate(detections):
#     if(d not in matched_indices[:,0]):
#       unmatched_detections.append(d)
#   unmatched_trackers = []
#   for t,trk in enumerate(trackers):
#     if(t not in matched_indices[:,1]):
#       unmatched_trackers.append(t)
#
#   #filter out matched with low IOU
#   matches = []
#   for m in matched_indices:
#     if(iou_matrix[m[0],m[1]]<iou_threshold):
#       unmatched_detections.append(m[0])
#       unmatched_trackers.append(m[1])
#     else:
#       matches.append(m.reshape(1,2))
#   if(len(matches)==0):
#     matches = np.empty((0,2),dtype=int)
#   else:
#     matches = np.concatenate(matches,axis=0)
#   return matches, np.array(unmatched_detections), np.array(unmatched_trackers)
#
# class Sort(object):
#   def __init__(self,max_age=4,min_hits=2):
#     """
#     Sets key parameters for SORT
#     """
#     self.max_age = max_age
#     self.min_hits = min_hits
#     self.trackers = []
#     self.frame_count = 0
#   def update(self,dets):
#     """
#     Params:
#       dets - a numpy array of detections in the format [[x1,y1,x2,y2,score],[x1,y1,x2,y2,score],...]
#     Requires: this method must be called once for each frame even with empty detections.参数输入为
#     x,y,l,角度，z坐标，高度
#     Returns the a similar array, where the last column is the object ID.
#
#     NOTE: The number of objects returned may differ from the number of detections provided.
#     """
#
#     self.frame_count += 1
#     # heading_mode=1
#     heading_mode = 0
#     #get predicted locations from existing trackers.
#     trks = np.zeros((len(self.trackers),5))
#     to_del = []
#     ret = []
#     for t,trk in enumerate(trks):
#       pos,bbox = self.trackers[t].predict()
#       pos=pos[0]
#       trk[:] = [pos[0], pos[1], pos[2], pos[3], bbox[4]]
#       if(np.any(np.isnan(pos))):
#         to_del.append(t)
#     trks = np.ma.compress_rows(np.ma.masked_invalid(trks))
#     for t in reversed(to_del):
#       self.trackers.pop(t)
#     matched, unmatched_dets, unmatched_trks = associate_detections_to_trackers(dets,trks)
#     #update matched trackers with assigned detections
#     for t,trk in enumerate(self.trackers):
#       if(t not in unmatched_trks):
#         d = matched[np.where(matched[:,1]==t)[0],0]
#         trk.update(dets[d,:][0])
#       # elif len(trk.kf_speed_x) > 1:
#       #   trk.kf.x[4] = trk.kf_speed_x[-2]
#       #   trk.kf.x[5] = trk.kf_speed_y[-2]
#     #create and initialise new trackers for unmatched detections
#     for i in unmatched_dets:
#         trk = KalmanBoxTracker(dets[i,:])
#         self.trackers.append(trk)
#     i = len(self.trackers)
#     for trk in reversed(self.trackers):
#         d, x_temp, trk_speed, trk_angle_box, center_angle = trk.get_state()
#         d = d[0]
#         if center_angle < 0:
#             center_angle += 360
#         # if ((trk.time_since_update < self.max_age) and (trk.hits >= self.min_hits or self.frame_count <= self.min_hits)):
#         #     if d[0] > 4 and d[0] < 21.9094 and d[1] > 29.6619:
#         #         head_final = 0
#         #         if x_temp[7] != 4:
#         #             d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
#         #             ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1], [d[0]])).reshape(1, -1))
#         #             # ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1])).reshape(1, -1))
#         #     elif d[0] < 36 and d[0] > 21.9094 and d[1] > 29.6619:
#         #         head_final = 180
#         #         if x_temp[7] != 4:
#         #             d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
#         #             ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1], [d[0]])).reshape(1, -1))
#         #             # ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1])).reshape(1, -1))
#         #     elif d[0] < 39 and d[0] > 21.9468 and d[1] < -16.4494:
#         #         head_final = 180
#         #         if x_temp[7] != 4:
#         #             d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
#         #             ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1], [d[0]])).reshape(1, -1))
#         #             # ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1])).reshape(1, -1))
#         #     elif d[0] > 9.5 and d[0] < 21.9468 and d[1] < -16.4494:
#         #         head_final = 0
#         #         if x_temp[7] != 4:
#         #             d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
#         #             ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1], [d[0]])).reshape(1, -1))
#         #             # ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1])).reshape(1, -1))
#         #     elif d[1] > 3.5 and d[1] < 6.90137 and d[0] < -9:
#         #         head_final = 270
#         #         if x_temp[7] != 4:
#         #             d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
#         #             ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1], [d[0]])).reshape(1, -1))
#         #             # ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1])).reshape(1, -1))
#         #     elif d[1] > 6.90137 and d[1] < 11.5 and d[0] < -9:
#         #         head_final = 90
#         #         if x_temp[7] != 4:
#         #             d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
#         #             ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1], [d[0]])).reshape(1, -1))
#         #             # ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1])).reshape(1, -1))
#         #     else:
#         #         heading_mode = 1
#         #         if heading_mode == 1:
#         #             if len(trk_angle_box) != 0:
#         #                 head_angle = trk_angle_box[-1][4]
#         #                 count_head = 0
#         #                 more_25 = []
#         #                 less_25 = []
#         #                 for k in trk_angle_box:
#         #                     if abs(k[4] - head_angle) > 90:
#         #                         count_head += 1
#         #                         temp_head = k[4]
#         #                         more_25.append(k[4])
#         #                     else:
#         #                         less_25.append(k[4])
#         #
#         #                 if count_head >= 0.5 * len(trk_angle_box):
#         #                     head_final = sum(more_25) / len(more_25)
#         #                 else:
#         #                     head_final = sum(less_25) / len(less_25)
#         #             else:
#         #                 head_final = x_temp[4]
#         #         else:
#         #             if len(trk_angle_box) != 0:
#         #                 head_final = trk_angle_box[-1][4]
#         #             else:
#         #                 head_final = x_temp[4]
#         #
#         #         ########insert angle for pedes and bike##3#
#         #         if trk_speed > 0.5 and (x_temp[7] in [1, 3, 4]) and center_angle != 1000:
#         #             head_final = center_angle
#         #
#         #         ######solve the problem of instability of static target direction
#         #         head_final = head_final % 360
#         #         trk.head_angle.append(head_final)
#         #         trk.l_rate.append(d)
#         #         if len(trk.head_angle) > 10:
#         #             trk.head_angle = trk.head_angle[-10:]
#         #             trk.l_rate = trk.l_rate[-10:]
#         #
#         #         ######*******条件合并的判断语句  (new)*********############
#         #         if len(trk.l_rate) > 9:
#         #         # if len(trk.l_rate) > 4:
#         #             dis_x = trk.l_rate[-1][0] - trk.l_rate[-10][0]
#         #             dis_y = trk.l_rate[-1][1] - trk.l_rate[-10][1]
#         #             # dis_x = trk.l_rate[-1][0] - trk.l_rate[-5][0]
#         #             # dis_y = trk.l_rate[-1][1] - trk.l_rate[-5][1]
#         #             dis_len = (dis_x * dis_x + dis_y * dis_y) ** 0.5
#         #             dis_angle = math.acos(dis_x / dis_len) * Rads_cov - 180
#         #             if dis_y > 0:
#         #                 dis_angle = 90 - dis_angle
#         #             else:
#         #                 dis_angle = 90 - (360 - dis_angle)
#         #             dis_angle = (dis_angle % 360)
#         #             # if dis_len > 1.5 and abs(trk.head_angle[-1] - dis_angle) > 130 and abs(trk.head_angle[-1] - dis_angle) < 230:
#         #             if dis_len > 1.5:
#         #                 if len(trk.head_angle) > 1 and abs(trk.head_angle[-1] - dis_angle) > 130 and abs(trk.head_angle[-1] - dis_angle) < 230:
#         #                     trk.head_angle[-1] += 180
#         #                     trk.head_angle[-1] = trk.head_angle[-1] % 360
#         #             else:
#         #                 # if abs(trk.head_angle[-1] - trk.head_angle[-2]) > 130 and abs(trk.head_angle[-1] - trk.head_angle[-2]) < 230:
#         #                 if len(trk.head_angle) > 1 and abs(trk.head_angle[-1] - trk.head_angle[-2]) > 130 and abs(trk.head_angle[-1] - trk.head_angle[-2]) < 230:
#         #                     trk.head_angle[-1] += 180
#         #                     trk.head_angle[-1] = trk.head_angle[-1] % 360
#         #             head_final = trk.head_angle[-1]
#         #             head_final = head_final % 360
#         #
#         #         d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
#         #         ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1], [d[0]])).reshape(1, -1))
#         #         # ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1])).reshape(1, -1))
#
#
#
#         if ((trk.time_since_update < self.max_age) and (trk.hits >= self.min_hits or self.frame_count <= self.min_hits)):
#             region_1_y1 = d[1] - (-17.5266 * d[0] + 89.7105)
#             region_1_y2 = d[1] - (-21.1314 * d[0] + 474.0634547)
#             region_2_y1 = d[1] - (-14.7332 * d[0] + 574.9071)
#             region_3_y1 = d[1] - (-39.8858 * d[0] + 1608.3578)
#             region_3_y2 = d[1] - (-37.2907 * d[0] + 793.5939)
#             region_4_y1 = d[1] - (-44.6893 * d[0] + 392.7884)
#             region_5_y1 = d[1] - (-0.0358 * d[0] + 2.7263)
#             region_5_y2 = d[1] - (-0.00882 * d[0] + 6.6286)
#             region_6_y1 = d[1] - (0.0121 * d[0] + 13.3117)
#             region_7_y1 = d[1] - (-0.0205 * d[0] + 9.357)
#             region_7_y2 = d[1] - (-0.0074 * d[0] + 5.3151456)
#             region_8_y1 = d[1] - (-0.0068 * d[0] + 2.330487)
#             if region_1_y1 * region_1_y2 < 0 and d[1] > 31:
#                 head_final = 0
#                 if x_temp[7] != 4:
#                     # d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
#                     d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
#                     ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1], [d[0]])).reshape(1, -1))
#                     # ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1])).reshape(1, -1))
#             elif region_2_y1 * region_1_y2 < 0 and d[1] > 31:
#                 head_final = 180
#                 if x_temp[7] != 4:
#                     # d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
#                     d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
#                     ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1], [d[0]])).reshape(1, -1))
#                     # ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1])).reshape(1, -1))
#             elif region_3_y1 * region_3_y2 < 0 and d[1] < -21:
#                 head_final = 180
#                 if x_temp[7] != 4:
#                     # d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
#                     d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
#                     ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1], [d[0]])).reshape(1, -1))
#                     # ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1])).reshape(1, -1))
#             elif region_3_y2 * region_4_y1 < 0 and d[1] < -21:
#                 head_final = 0
#                 if x_temp[7] != 4:
#                     # d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
#                     d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
#                     ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1], [d[0]])).reshape(1, -1))
#                     # ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1])).reshape(1, -1))
#             elif region_5_y1 * region_5_y2 < 0 and d[0] < -2:
#                 head_final = 270
#                 if x_temp[7] != 4:
#                     # d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
#                     d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
#                     ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1], [d[0]])).reshape(1, -1))
#                     # ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1])).reshape(1, -1))
#             elif region_5_y2 * region_6_y1 < 0 and d[0] < -2:
#                 head_final = 90
#                 if x_temp[7] != 4:
#                     # d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
#                     d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
#                     ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1], [d[0]])).reshape(1, -1))
#                     # ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1])).reshape(1, -1))
#             elif region_7_y1 * region_7_y2 < 0 and d[0] > 47:
#                 head_final = 90
#                 if x_temp[7] != 4:
#                     d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
#                     ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1], [d[0]])).reshape(1, -1))
#             elif region_7_y2 * region_8_y1 < 0 and d[0] > 47:
#                 head_final = 270
#                 if x_temp[7] != 4:
#                     d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
#                     ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1], [d[0]])).reshape(1, -1))
#             else:
#                 heading_mode = 1
#                 if heading_mode == 1:
#                     if len(trk_angle_box) != 0:
#                         head_angle = trk_angle_box[-1][4]
#                         count_head = 0
#                         more_25 = []
#                         less_25 = []
#                         for k in trk_angle_box:
#                             if abs(k[4] - head_angle) > 90:
#                                 count_head += 1
#                                 temp_head = k[4]
#                                 more_25.append(k[4])
#                             else:
#                                 less_25.append(k[4])
#
#                         if count_head >= 0.5 * len(trk_angle_box):
#                             head_final = sum(more_25) / len(more_25)
#                         else:
#                             head_final = sum(less_25) / len(less_25)
#                     else:
#                         head_final = x_temp[4]
#                 else:
#                     if len(trk_angle_box) != 0:
#                         head_final = trk_angle_box[-1][4]
#                     else:
#                         head_final = x_temp[4]
#
#                 if len(trk_angle_box) != 0:
#                     head_final = trk_angle_box[-1][4]
#                 else:
#                     head_final = x_temp[4]
#                 ########insert angle for pedes and bike##3#
#                 if trk_speed > 0.5 and (x_temp[7] in [1, 3, 4]) and center_angle != 1000:
#                     head_final = center_angle
#                 ######solve the problem of instability of static target direction
#                 head_final = head_final % 360
#                 trk.head_angle.append(head_final)
#                 trk.l_rate.append(d)
#                 if len(trk.head_angle) > 10:
#                     trk.head_angle = trk.head_angle[-10:]
#                     trk.l_rate = trk.l_rate[-10:]
#
#                 ######*******条件合并的判断语句  (new)*********############
#                 if len(trk.head_angle) > 1 and abs(trk.head_angle[-1] - trk.head_angle[-2]) > 130 and abs(trk.head_angle[-1] - trk.head_angle[-2]) < 230:
#                     trk.head_angle[-1] += 180
#                     # trk.head_angle[-1] = trk.head_angle[-2]
#                     trk.head_angle[-1] = trk.head_angle[-1] % 360
#                 head_final = trk.head_angle[-1]
#                 head_final = head_final % 360
#                 if len(trk.l_rate) > 9:
#                 # if len(trk.l_rate) > 4:
#                     dis_x = trk.l_rate[-1][0] - trk.l_rate[-10][0]
#                     dis_y = trk.l_rate[-1][1] - trk.l_rate[-10][1]
#                     # dis_x = trk.l_rate[-1][0] - trk.l_rate[-5][0]
#                     # dis_y = trk.l_rate[-1][1] - trk.l_rate[-5][1]
#                     dis_len = (dis_x * dis_x + dis_y * dis_y) ** 0.5
#                     dis_angle = math.acos(dis_x / dis_len) * Rads_cov - 180
#                     if dis_y > 0:
#                         dis_angle = 90 - dis_angle
#                     else:
#                         dis_angle = 90 - (360 - dis_angle)
#                     dis_angle = (dis_angle % 360)
#                     # if dis_len > 1.5 and abs(trk.head_angle[-1] - dis_angle) > 130 and abs(trk.head_angle[-1] - dis_angle) < 230:
#                     if dis_len > 0.8:
#                         if len(trk.head_angle) > 1 and abs(trk.head_angle[-1] - dis_angle) > 130 and abs(trk.head_angle[-1] - dis_angle) < 230:
#                             trk.head_angle[-1] += 180
#                             trk.head_angle[-1] = trk.head_angle[-1] % 360
#                     else:
#                         # if abs(trk.head_angle[-1] - trk.head_angle[-2]) > 130 and abs(trk.head_angle[-1] - trk.head_angle[-2]) < 230:
#                         if len(trk.head_angle) > 1 and abs(trk.head_angle[-1] - trk.head_angle[-2]) > 130 and abs(trk.head_angle[-1] - trk.head_angle[-2]) < 230:
#                             trk.head_angle[-1] += 180
#                             # trk.head_angle[-1] = trk.head_angle[-2]
#                             trk.head_angle[-1] = trk.head_angle[-1] % 360
#                 # else:
#                 #     if len(trk.head_angle) > 1 and abs(trk.head_angle[-1] - trk.head_angle[-2]) > 130 and abs(
#                 #             trk.head_angle[-1] - trk.head_angle[-2]) < 230:
#                 #         trk.head_angle[-1] += 180
#                 #         trk.head_angle[-1] = trk.head_angle[-1] % 360
#                 head_final = trk.head_angle[-1]
#                 head_final = head_final % 360
#
#                 # d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
#                 d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], x_temp[7]]
#                 ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1], [d[0]])).reshape(1, -1))
#                 # ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1])).reshape(1, -1))
#         i -= 1
#
#         #remove dead tracklet
#         if(trk.time_since_update > self.max_age):
#           self.trackers.pop(i)
#     if(len(ret)>0):
#       return np.concatenate(ret)
#     return np.empty((0,5))


