# #####*****  策略1:加入了框内框外点的判断，以及新的高速低速判断，而不是采用动静的绝对判断，
# #####*****  策略2:对机动车采用加速模型，对于非机动车采用匀速模型，
# #####*****  策略3:将两个跟踪列表进行合并,统一采用IOU进行数据关联,但是在级联阶段可选IOU和距离两种规则，需要修改传入的参数
# #####*****  策略4:关于历史状态，保证列表里存储的首尾两个状态量的距离在3m左右
# #####*****  策略5:静止目标框容易出现飘框，考虑到低速目标其运动状态基本不变，尝试其不进行卡尔曼的预测步骤，当有更新的时候执行更新
# #####*****  策略6:对于静态目标，通过增大其卡尔曼中的Q矩阵，来使其状态平滑，减少飘框等现象，功能待测试  当前更新了两个参数self.min_q, self.max_q，
# #####*****  保证参数缩小和增大都只有一次，不会累乘
# #####*****  改进1:将车道信息做成可选择的  加载车道信息文件，读取配置,车道信息的规定是从雷达y轴正轴方向开始，顺时针开始统计

# #####*****  尝试1:尝试除去高速和低速目标，再加上动静目标属性,依靠旧的速度和位移共同判断动静的方法
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

import copy

from numba import jit
import numpy as np
import time
import math
# from sklearn.utils.linear_assignment_ import linear_assignment
from scipy.optimize import linear_sum_assignment as linear_assignment
from filterpy.kalman import KalmanFilter
import core.box_np_ops as box_np_ops
# from CommonDefine import *


Rads_cov = 180 / math.pi

not_match_trks = []

label_dict={0:[1.95017717, 4.60718145, 1.72270761],1:[0.60058911, 1.68452161, 1.27192197],2:[2.94046906, 11.1885991, 3.47030982],
            3:[0.76279481, 2.09973778, 1.44403034],4:[0.66344886, 0.7256437, 1.75748069],5:[0.39694519, 0.40359262, 1.06232151],
            6:[2.4560939, 6.73778078, 2.73004906]}

trk_i = []
trk_iadd1 = []
# strExePath = getExcutePath()
# k_file = strExePath + '/Configs/PointCloud/chedao_k.npy'
# b_file = strExePath + '/Configs/PointCloud/chedao_b.npy'
# limit_file = strExePath + '/Configs/PointCloud/chedao_limit.npy'
# angle_file = strExePath + '/Configs/PointCloud/chedao_angle.npy'
# line_k = np.load(k_file).reshape(1, -1)
# line_b = np.load(b_file).reshape(1, -1)
# line_limit = np.load(limit_file).reshape(1, -1)
# line_angle = np.load(angle_file).reshape(1, -1)

#############对lidar计算iou值##############
def rotate_nms_cc(dets, trackers):
    trackers_corners = box_np_ops.center_to_corner_box2d(trackers[:, :2], trackers[:, 2:4], trackers[:, 4])
    trackers_standup = box_np_ops.corner_to_standup_nd(trackers_corners)
    dets_corners = box_np_ops.center_to_corner_box2d(dets[:, :2], dets[:, 2:4], dets[:, 4])
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

def cal_angle(state_list, thresh, ang):
    dis_x = state_list[-1][0][0] - state_list[0][0][0]
    dis_y = state_list[-1][0][1] - state_list[0][0][1]
    dis_len = (dis_x * dis_x + dis_y * dis_y) ** 0.5
    if dis_len > thresh:
        #转到y负轴
        dis_angle = math.acos(dis_x / dis_len) * Rads_cov - 180
        if dis_y > 0:
            dis_angle = 90 - dis_angle
        if dis_y <= 0:
            dis_angle = 90 - (360 - dis_angle)
        dis_angle = (dis_angle % 360)
        dis_angle_ = copy.deepcopy(dis_angle)
        if abs(ang - dis_angle_) > 120 and abs(ang - dis_angle_) < 190:
            dis_angle = (ang + 180) % 360
        if abs(ang - dis_angle_) > 0 and abs(ang - dis_angle_) < 20:
            dis_angle = ang % 360
        # if dis_angle < 180:
        #     dis_angle = dis_angle + 180
        # else:
        #     dis_angle = dis_angle - 180
        # dis_angle = math.asin(dis_y / dis_len) * Rads_cov
        # if dis_x > 0 and dis_y > 0:
        #     dis_angle = 90 - dis_angle
        # if dis_x < 0 and dis_y > 0:
        #     dis_angle = 270 + dis_angle
        # if dis_x < 0 and dis_y < 0:
        #     dis_angle = 270 - dis_angle
        # if dis_x > 0 and dis_y < 0:
        #     dis_angle = 90 + dis_angle
        # dis_angle = (dis_angle % 360)
    else:
        dis_angle = None
    return dis_angle


class KalmanBoxTracker(object):
  """
  This class represents the internel state of individual tracked objects observed as bbox.
  """
  count = 0
  def __init__(self, bbox):
    """
    Initialises a tracker using initial bounding box.
    """
    self.use_acc_model = 0
    # if bbox[7] in list([0, 2, 6]):
    if bbox[7] in list([0, 2, 5, 6, 1, 3]):
   # if bbox[7] in list([0, 1, 2, 3, 5, 6]):
        self.use_acc_model = 1
        self.kf = KalmanFilter(dim_x=8, dim_z=4)
        #**********x,y,w,l,vx,vy,ax,ay*********************
        self.kf.F = np.array([[1, 0, 0, 0, 0.1, 0, 0.005, 0],
                              [0, 1, 0, 0, 0, 0.1, 0, 0.005],
                              [0, 0, 1, 0, 0, 0, 0, 0],
                              [0, 0, 0, 1, 0, 0, 0, 0],
                              [0, 0, 0, 0, 1, 0, 0.01, 0],
                              [0, 0, 0,  0, 0, 1, 0, 0.01],
                              [0, 0,  0, 0, 0, 0, 1, 0],
                              [0, 0, 0, 0, 0, 0, 0, 1]])
        self.kf.H = np.array(
            [[1, 0, 0, 0, 0, 0, 0, 0],
             [0, 1, 0, 0, 0, 0, 0, 0],
             [0, 0, 1, 0, 0, 0, 0, 0],
             [0, 0, 0, 1, 0, 0, 0, 0]
             ])
        self.kf.Q *= 0.1
        self.kf.R *= 1
        self.kf.P *= 10
        self.kf.P[4:6, 4:6] *= 1000
    else:
        self.use_acc_model = 0
        self.kf = KalmanFilter(dim_x=7, dim_z=4)
        self.kf.F = np.array([[1, 0, 0, 0, 1, 0, 0],
                              [0, 1, 0, 0, 0, 1, 0],
                              [0, 0, 1, 0, 0, 0, 1],
                              [0, 0, 0, 1, 0, 0, 0],
                              [0, 0, 0, 0, 1, 0, 0],
                              [0, 0, 0, 0, 0, 1, 0],
                              [0, 0, 0, 0, 0, 0, 1]])
        self.kf.H = np.array([[1, 0, 0, 0, 0, 0, 0],
                              [0, 1, 0, 0, 0, 0, 0],
                              [0, 0, 1, 0, 0, 0, 0],
                              [0, 0, 0, 1, 0, 0, 0],
                              ])
        self.kf.Q[-1, -1] *= 0.01
        self.kf.Q[4:, 4:] *= 0.01
        self.kf.R *= 10
        self.kf.P *= 10
        self.kf.P[4:6, 4:6] *= 1000

    ####使用box_lidar,里面包含中心点，面积和航向角度
    self.kf.x[:4] = bbox[:4].reshape((-1, 1))
    self.bbox = bbox###对应存储的状态值
    self.time_since_update = 0

    # #####*****  减小和增大卡尔曼Q矩阵的标志位  *****#####
    self.min_q = True
    self.max_q = False

    # #####*****  存储检测目标的稳定航向角，以车道角和轨迹角为标准
    self.final_angle = None

    # self.id = KalmanBoxTracker.count
    # KalmanBoxTracker.count += 1
    self.history = []
    self.head_angle = []
    ######self.state存储卡尔曼滤波的状态量，目前来看存储的是预测的状态量，有更新时存储的是更新后的状态量
    self.state = []

    # self.output = 0

    # #####*****  存储检测出来的航向角，保证检测角不跳变
    self.angle_list = []

    # #####*****高速目标  低速目标
    self.high_speed = False
    self.low_speed = False

    # #####*****动态目标  静态目标
    self.dynamic = False
    self.static = False

    # #####*****存储状态量判断目标的动静属性
    self.state_judge = []

    # #####*****  车道航向角  轨迹航向角  检测航向角
    self.lane_angle = None
    self.track_angle = None
    self.detec_angle = None

    self.angle_box=[]#####对应history
    self.label_box=[]#####存储历史10帧的类别消息，用途是，寻找每个目标10帧内出现次数最多的标签作为新一帧目标
    self.hits = 0
    self.hit_streak = 0
    self.age = 0 #####表示跟踪目标可以存活的时间（帧数）
    ################追踪目标的速度################
    self.speed = 0
    #####*****存储更改的label信息*****#####
    self.label = 0
    #####*****高速和低速的阈值*****#####
    self.speed_thresh = 3

  def update_id(self):
      self.id = KalmanBoxTracker.count
      KalmanBoxTracker.count += 1

  def update(self, bbox):
    """
    Updates the state vector with observed bbox.
    """
    self.time_since_update = 0 ####表示距离上一次更新后没有再匹配更新的次数
    self.history = []
    self.hits += 1 ###表示在age时间内其能够匹配上的次数
    self.hit_streak += 1##表示连续匹配上的次数，从第一次开始
    ####使用匹配到的box信息对当前的卡尔曼状态量进行更新
    self.kf.update(bbox[:4].reshape((-1, 1)))
    self.bbox = bbox
    # if self.bbox[7] == 5:
    #     self.bbox[7] = 4
    self.label_box.append(self.bbox[7])
    ######保证列表的长度，防止长度过长
    if len(self.label_box) > 50:
        self.label_box = self.label_box[-50:]
    ######分析历史帧的标签，取次数最多的label，并赋对应的box尺寸
    if len(self.label_box) > 0:
        more_label = max(self.label_box, key=self.label_box.count)
        self.label = more_label
        # self.bbox[2] = label_dict[self.label][0]
        # self.bbox[3] = label_dict[self.label][1]
        # self.bbox[6] = label_dict[self.label][2]
        ###discuss
        if self.label_box.count(self.label) / len(self.label_box) > 0.7:
            self.bbox[7] = self.label
            if self.label_box.count(self.label) > 35:
                self.label_box[-1] = more_label
    else:
        self.label = self.bbox[7]


    if self.use_acc_model:
        self.speed = math.sqrt(self.kf.x[4] ** 2 + self.kf.x[5] ** 2)
    else:
        self.speed = 10 * math.sqrt(self.kf.x[4] ** 2 + self.kf.x[5] ** 2)

    if self.speed > self.speed_thresh:
        self.high_speed = True
        self.low_speed = False
    else:
        self.high_speed = False
        self.low_speed = True

    if len(self.state_judge) > 1:
        diff_x = self.state_judge[-1][0] - self.state_judge[0][0]
        diff_y = self.state_judge[-1][1] - self.state_judge[0][1]
        diif_dis = (diff_x**2+diff_y**2)**0.5
        if self.speed < 2 and diif_dis < 1.5:
            self.static = True
            self.dynamic = False
        else:
            self.static = False
            self.dynamic = True

    # # #####*****  低速的时候，将卡尔曼的Q矩阵变小，让轨迹更平滑，即受离谱检测目标的影响会小些，待测试的功能  *****#####

    # # #####*****  考虑对机动车采用Q矩阵策略  *****#####
    if self.label in [0, 2, 5, 6, 1, 3]:
        # if (not self.high_speed) and self.min_q:
        if self.static and self.min_q:
            self.kf.Q *= 0.1
            self.min_q = False
            self.max_q = True
        # if self.high_speed and self.max_q:
        if self.dynamic and self.max_q:
            self.kf.Q *= 10
            self.min_q = True
            self.max_q = False

    # # #####*****  原始的保证状态列表的长度，根据列表头尾的距离计算轨迹航向角  *****#####
    # self.state.append(self.kf.x[:2, :].reshape(1, -1))
    # if len(self.state) > 10:
    #     self.state = self.state[-10:]

    # #####*****  根据状态列表计算距离，找到距离最接近3的索引，然后保留该索引之后的所有状态量   *****#####
    # #####*****  在更新函数中添加，存在部分目标间隔好多帧才更新，这就造成两个状态之间的距离较远，超过3m  *****#####
    # #####*****  在预测函数中添加，依然会存在部分这样的问题，只是出现的次数会减少
    self.state.append(self.kf.x[:2, :].reshape(1, -1))
    if len(self.state) > 1:
        a = np.asarray(self.state).reshape(-1, 2)
        diff = a - self.state[-1].reshape(-1, 2)
        eve_dis = abs((diff[:, 0] ** 2 + diff[:, 1] ** 2) ** 0.5).reshape(-1, 1)
        eve_dis -= 3
        po_indice = np.where(eve_dis > 0)
        if po_indice[0].shape[0] > 0:
            new_eve_dis = eve_dis[po_indice[0], 0]
            min_dis_indice = np.argmin(new_eve_dis)
            if eve_dis[po_indice[0][min_dis_indice]] > 0:
                self.state = self.state[po_indice[0][min_dis_indice]:]

    ######self.angle_box存储的是检测状态量
    self.angle_box.append(self.bbox)
    ######保证列表的长度，防止长度过长
    if len(self.angle_box) > 10:
        self.angle_box = self.angle_box[-10:]


  def predict(self, trk_i, id):
    """
    Advances the state vector and returns the predicted bounding box estimate.
    """
    # 另外加入部分，用于判断id目标的移动速度
    if len(trk_i) > 2:
        dxi = 0
        dyi = 0
        dxj = 0
        dyj = 0
        for i in trk_i[len(trk_i) - 1]:
            if i.id == id:
                dxi = i.bbox[0]
                dyi = i.bbox[1]
        for j in trk_i[len(trk_i) - 2]:
            if j.id == id:
                dxj = i.bbox[0]
                dyj = i.bbox[1]
        d = math.sqrt((dxi - dxj) * (dxi - dxj) + (dyi - dyj) * (dyi - dyj))
        if d > 0.15 or self.time_since_update < 3: #**** 对于目标速度很慢不进行预测，阈值调整为0.8试试
            self.kf.predict()

    # # #####*****  对于速度较小，且连续3帧没有更新的轨迹不进行预测  *****#####
    if len(trk_i) <= 2:
        if self.speed > 1 or self.time_since_update < 3: #**** 对于目标速度很慢不进行预测，阈值调整为0.8试试
            self.kf.predict()
    # self.kf.predic()
    # self.output = 0
    self.age += 1
    if(self.time_since_update>0):
      self.hit_streak = 0
    self.time_since_update += 1
    ########30##直接使用box_lidar#####不需要进行转换##########
    output_history = self.kf.x[:4].reshape((1, 4))
    self.history.append(output_history)

    self.state_judge.append(self.kf.x)
    if len(self.state_judge) > 10:
        self.state_judge = self.state_judge[-10:]

    return self.history[-1], self.bbox

  def get_state(self):
    """
    Returns the current bounding box estimate.
    """
    ##########直接使用box_lidar#####不需要进行转换##########
    output_x = self.kf.x[:4].reshape((1,4))

    if self.speed < 0.5 and len(self.angle_box) > 1:
        x_mean = np.asarray(self.angle_box)
        output_x = np.mean(x_mean[:, :4], axis=0).reshape((1, 4))
    return output_x, self.bbox, self.speed, self.angle_box

def associate_detections_to_trackers(detections, trackers, flag, speed):
  """
  Assigns detections to tracked object (both represented as bounding boxes)

  Returns 3 lists of matches, unmatched_detections and unmatched_trackers
  """
  if(len(trackers)==0):
    return np.empty((0,2),dtype=int), np.arange(len(detections)), np.empty((0,5),dtype=int), 0

  # #####*****   计算距离矩阵，同时保证距离不大于1，也可以不压缩，修改后面的阈值即可  *****#####
  det_xy = detections[:, :2]
  trk_xy = trackers[:, :2]
  distance_matrix = (np.reshape(np.sum(det_xy ** 2, axis=1), (det_xy.shape[0], 1)) + np.sum(trk_xy ** 2,axis=1) - 2 * det_xy.dot(trk_xy.T)) ** 0.5
  distance_matrix = (1 - (distance_matrix / 160))


  #####直接使用lidar部分的iou，这里直接使用矩阵就行#####
  detections[:, 4] *= (np.pi / 180)
  trackers[:, 4] *= (np.pi / 180)
  iou_matrix = rotate_nms_cc(detections, trackers)

  big_trackers = trackers
  # #####*****  1.5是将目标框扩大，考虑稀疏点云检测目标的不稳定性，扩大框的尺寸，增加匹配上的概率，当然，你也可以选择不扩大  *****#####
  big_thresh = 1
  big_trackers[:, 2] *= big_thresh
  big_trackers[:, 3] *= big_thresh

  # #####*****  给出跟踪轨迹的目标的四个角点的信息  *****#####
  tra_corners = box_np_ops.center_to_corner_box2d(big_trackers[:, :2], big_trackers[:, 2:4], big_trackers[:, 4])
  detections[:, 4] *= (180 / np.pi)
  trackers[:, 4] *= (180 / np.pi)

  if flag == 0:
  # #####*****  IOU匹配  *****#####
      cost_matrix = iou_matrix
      iou_threshold = 0.000001
  else:
  # #####*****  距离匹配  *****#####
      cost_matrix = distance_matrix
      iou_threshold = 0.96875
  matched_indices = linear_assignment(-cost_matrix)
  # 没有用sklearn的匈牙利匹配，需要转置
  matched_indices = np.array(matched_indices).T
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
    # #####*****  以下一部分是为了计算关联点是在框内还是框外,根据向量叉乘计算面积，对比面积即可,类似多边形计算面积  *****#####
    # #####*****  扩大框尺寸之后的长和宽，计算面积  ******#####
    l = trackers[m[1], 2]*big_thresh
    w = trackers[m[1], 3]*big_thresh
    # #####*****  tra_corners是计算出来的扩大框之后的角点信息  *****#####
    asso_tra_corners = tra_corners[m[1]]
    stand_point = np.array([[detections[m[0]][0], detections[m[0]][1]]])
    new_diff = asso_tra_corners - stand_point
    diff_1 = new_diff[0]
    diff_2 = new_diff[1]
    diff_3 = new_diff[2]
    diff_4 = new_diff[3]
    area_1 = 0.5 * abs(diff_1[0] * diff_2[1] - diff_1[1] * diff_2[0])
    area_2 = 0.5 * abs(diff_2[0] * diff_3[1] - diff_2[1] * diff_3[0])
    area_3 = 0.5 * abs(diff_3[0] * diff_4[1] - diff_3[1] * diff_4[0])
    area_4 = 0.5 * abs(diff_4[0] * diff_1[1] - diff_4[1] * diff_1[0])
    total_area = area_1 + area_2 + area_3 + area_4
    area_tra = l * w
    # #####*****  trackers的最后一位存储的是轨迹的time_since_update  *****#####
    # #####*****  当time_since_update大于4时，可以最近距离的数据关联方式  *****#####
    # #####*****  距离关联与IOU关联的方式选择判断(采用高速低速的目标判断)  *****#####
    # if trackers[m[1]][-1] > 8 and speed[0][m[1]] > 3:
    if flag == 1:
        cost_matrix = distance_matrix
        threshold = 0.9875
        in_out_judege = True
        if cost_matrix[m[0], m[1]] < threshold and trackers[m[1]][-1] < 15:
           unmatched_detections.append(m[0])
           unmatched_trackers.append(m[1])
           continue
        else:
           matches.append(m.reshape(1, 2))
           continue

    if flag == 0:
        cost_matrix = iou_matrix
        threshold = 0.0001
        in_out_judege = True
        if cost_matrix[m[0], m[1]] < threshold or abs(total_area - area_tra) > 0.1:
          unmatched_detections.append(m[0])
          unmatched_trackers.append(m[1])
        else:
          matches.append(m.reshape(1, 2))
    # if in_out_judege:
    #     # if cost_matrix[m[0], m[1]] < threshold or (total_area > (area_tra + 0.01) and speed[0][m[1]] < 3):
    #     if cost_matrix[m[0], m[1]] < threshold or abs(total_area - area_tra) > 0.1:
    #       unmatched_detections.append(m[0])
    #       unmatched_trackers.append(m[1])
    #     else:
    #       matches.append(m.reshape(1, 2))
    # else:
    #     # if cost_matrix[m[0], m[1]] < threshold or (total_area > (area_tra + 0.01) and speed[0][m[1]] < 3):
    #     if cost_matrix[m[0], m[1]] < threshold or abs(total_area - area_tra) > 0.1:
    #        unmatched_detections.append(m[0])
    #        unmatched_trackers.append(m[1])
    #     else:
    #        matches.append(m.reshape(1, 2))
  if(len(matches)==0):
    matches = np.empty((0, 2), dtype=int)
  else:
    matches = np.concatenate(matches, axis=0)

  return matches, np.array(unmatched_detections), np.array(unmatched_trackers), cost_matrix


def associate_detections_to_trackers1(detections, trackers, flag, speed):
  """
  Assigns detections to tracked o192.168.3.181bject (both represented as bounding boxes)

  Returns 3 lists of matches, unmatched_detections and unmatched_trackers
  """
  if(len(trackers)==0):
    return np.empty((0,2),dtype=int), np.arange(len(detections)), np.empty((0,5),dtype=int), 0

  # #####*****   计算距离矩阵，同时保证距离不大于1，也可以不压缩，修改后面的阈值即可  *****#####
  det_xy = detections[:, :2]
  trk_xy = trackers[:, :2]
  distance_matrix = (np.reshape(np.sum(det_xy ** 2, axis=1), (det_xy.shape[0], 1)) + np.sum(trk_xy ** 2,axis=1) - 2 * det_xy.dot(trk_xy.T)) ** 0.5
  distance_matrix = (1 - (distance_matrix / 160))


  #####直接使用lidar部分的iou，这里直接使用矩阵就行#####
  detections[:, 4] *= (np.pi / 180)
  trackers[:, 4] *= (np.pi/180)
  iou_matrix = rotate_nms_cc(detections, trackers)

  big_trackers = trackers
  # #####*****  1.5是将目标框扩大，考虑稀疏点云检测目标的不稳定性，扩大框的尺寸，增加匹配上的概率，当然，你也可以选择不扩大  *****#####
  big_thresh = 1.3
  big_trackers[:, 2] *= big_thresh
  big_trackers[:, 3] *= big_thresh

  # #####*****  给出跟踪轨迹的目标的四个角点的信息  *****#####
  tra_corners = box_np_ops.center_to_corner_box2d(big_trackers[:, :2], big_trackers[:, 2:4], big_trackers[:, 4])
  detections[:, 4] *= (180 / np.pi)
  trackers[:, 4] *= (180 / np.pi)


  if flag == 0:
  # #####*****  IOU匹配  *****#####
      cost_matrix = iou_matrix
      iou_threshold = 0.000001
  else:
  # #####*****  距离匹配  *****#####
      cost_matrix = distance_matrix
      iou_threshold = 0.96875
  matched_indices = linear_assignment(-cost_matrix)
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
    if flag == 1:
        threshold = iou_threshold
        if cost_matrix[m[0], m[1]] < threshold:
          unmatched_detections.append(m[0])
          unmatched_trackers.append(m[1])
        else:
          matches.append(m.reshape(1,2))
    else:
        threshold = iou_threshold
        # #####*****  以下一部分是为了计算关联点是在框内还是框外,根据向量叉乘计算面积，对比面积即可,类似多边形计算面积  *****#####
        # #####*****  扩大框尺寸之后的长和宽，计算面积  ******#####
        l = trackers[m[1], 2]*big_thresh
        w = trackers[m[1], 3]*big_thresh
        # #####*****  tra_corners是计算出来的扩大框之后的角点信息  *****#####
        asso_tra_corners = tra_corners[m[1]]
        stand_point = np.array([[detections[m[0]][0], detections[m[0]][1]]])
        new_diff = asso_tra_corners - stand_point
        diff_1 = new_diff[0]
        diff_2 = new_diff[1]
        diff_3 = new_diff[2]
        diff_4 = new_diff[3]
        area_1 = 0.5 * abs(diff_1[0] * diff_2[1] - diff_1[1] * diff_2[0])
        area_2 = 0.5 * abs(diff_2[0] * diff_3[1] - diff_2[1] * diff_3[0])
        area_3 = 0.5 * abs(diff_3[0] * diff_4[1] - diff_3[1] * diff_4[0])
        area_4 = 0.5 * abs(diff_4[0] * diff_1[1] - diff_4[1] * diff_1[0])
        total_area = area_1 + area_2 + area_3 + area_4
        area_tra = l * w

        if cost_matrix[m[0], m[1]] < threshold or (total_area > (area_tra + 0.01) and speed[0][m[1]] < 3):
          unmatched_detections.append(m[0])
          unmatched_trackers.append(m[1])
        else:
          matches.append(m.reshape(1,2))
  if(len(matches)==0):
    matches = np.empty((0,2),dtype=int)
  else:
    matches = np.concatenate(matches,axis=0)

  return matches, np.array(unmatched_detections), np.array(unmatched_trackers), cost_matrix

class Sort1(object):
  def __init__(self, max_age=6, min_hits=4):
    """
    Sets key parameters for SORT
    """
    self.max_age = max_age
    self.max_age_new = 15
    self.min_hits = min_hits
    self.trackers = []
    self.trackers_bak = []
    self.frame_count = 0
    #####*****判断是否利用位移计算航向角的阈值(暂时觉得速度作为判断条件不太靠谱，采用位移作为判断条件)自己看情况修改吧，可以适当大*****#####
    self.dis_thresh = 1.5
    # #####*****  角度差，将相邻帧的航向角的变化限制在正负self.angle_judge内  *****#####
    self.angle_judge = 8

    self.angle_numbers = 0  #角度修正计数

  def update(self, dets, frame_number):
      """
      Params:
        dets - a numpy array of detections in the format [[x1,y1,x2,y2,score],[x1,y1,x2,y2,score],...]
      Requires: this method must be called once for each frame even with empty detections.参数输入为
      x,y,l,角度，z坐标，高度
      line_k: 车道的斜率
      line_b: 车道的截距
      line_limit: 车道限制的截止坐标值
      line_angle: 车道对应的航向角
      Returns the a similar array, where the last column is the object ID.

      NOTE: The number of objects returned may diffkf_speed_xer from the number of detections provided.
      """
      self.angle_numbers = 0
      self.frame_count += 1
      # get predicted locations from existing trackers.
      trks = np.zeros((len(self.trackers), 6))
      object_speed = np.zeros((1, len(self.trackers)))
      to_del = []
      ret = []
      # list_bbox = []#保存预测值，后面删除
      for t, trk in enumerate(trks):
          pos, bbox = self.trackers[t].predict(trk_i, self.trackers[t].id)
          pos = pos[0]
          # trk[:] = [pos[0], pos[1], bbox[2], bbox[3], bbox[4] * 180 / np.pi, self.trackers[t].time_since_update]
          trk[:] = [pos[0], pos[1], bbox[2], bbox[3], bbox[4], self.trackers[t].time_since_update]
          object_speed[0][t] = self.trackers[t].speed
          # list_bbox.append(bbox)#保存预测值，后面删除
          if (np.any(np.isnan(pos))):
              to_del.append(t)
      trks = np.ma.compress_rows(np.ma.masked_invalid(trks))#判断是否有无效值
      object_speed = np.ma.compress_cols(np.ma.masked_invalid(object_speed))
      for t in reversed(to_del):
          self.trackers.pop(t)
      trks_S = [np.matmul(np.matmul(tracker.kf.H, tracker.kf.P), tracker.kf.H.T) + tracker.kf.R for tracker in
                self.trackers]

      # #####*****  执行数据关联函数  *****#####
      matched, unmatched_dets, unmatched_trks, cost_matrix = associate_detections_to_trackers(dets, trks, 0, object_speed)
      not_match_dets = []
    
      for t, trk in enumerate(self.trackers):
          if (t not in unmatched_trks):
              d = matched[np.where(matched[:, 1] == t)[0], 0]
              trk.update(dets[d, :][0])
      for i in unmatched_dets:
          not_match_dets.append(dets[i, :])
      not_match_dets = np.asarray(not_match_dets).reshape((-1, dets.shape[1]))

      unmatched_trks.tolist()
      new_un_match_trks = []
      for my_num in range(len(unmatched_trks.tolist())):
          new_un_match_trks.append(self.trackers[unmatched_trks.tolist()[my_num]])

      # #####*****  没有匹配上的直接进入到距离匹配  *****#####
      un_match_object = dets[unmatched_dets.tolist(), :]
      un_match_track = trks[unmatched_trks.tolist(), :]

      object_speed_bak = np.zeros((1, len(unmatched_trks.tolist())))
      number_unmatched = 0
      for number_unmatched_ in unmatched_trks.tolist():
          object_speed_bak[0][number_unmatched] = object_speed[0][number_unmatched_]
          number_unmatched += 1

      # object_speed_bak = np.zeros((1, len(self.trackers_bak)))
      # for t_my, trk_my in enumerate(trks_bak):
      #     pos_bak, bbox_bak = self.trackers_bak[t_my].predict()
      #     pos_bak = pos_bak[0]
      #     trk_my[:] = [pos_bak[0], pos_bak[1], bbox_bak[2], bbox_bak[3], bbox_bak[4] * 180 / np.pi, self.trackers_bak[t_my].time_since_update]
      #     object_speed_bak[0][t_my] = self.trackers_bak[t_my].speed

      matched_bak, unmatched_dets_bak, unmatched_trks_bak, cost_matrix = associate_detections_to_trackers(un_match_object, un_match_track, 1, object_speed_bak)

      # #####*****  新的匹配策略对应的目标提取方式  *****#####
      for t, trk in enumerate(
              new_un_match_trks):  # if ((trk.time_since_update < self.max_age) and (trk.hits >= self.min_hits or self.frame_count <= self.min_hits)):
          if (t not in unmatched_trks_bak):
              d = matched_bak[np.where(matched_bak[:, 1] == t)[0], 0]
              trk.update(un_match_object[d, :][0])

      for i in unmatched_dets_bak:
          trk = KalmanBoxTracker(un_match_object[i, :])
          trk.update_id()
          self.trackers.append(trk)

      num_tra = len(self.trackers)
      #以上是预测和检测匹配关联

      #以下都是修正策略和轨迹寿命的判定
      for trk in reversed(self.trackers):
          d, x_temp, trk_speed, trk_angle_box= trk.get_state()
          d = d[0]
          # 存储检测航向角,并保证列表的长度
          trk.angle_list.append((x_temp[4]) % 360)#保存历史轨迹的跟踪角度
          if len(trk.angle_list) > 10:
              trk.angle_list = trk.angle_list[-10:]
          if len(trk.angle_list) > 1:#保证检测的航向角不跳变
              angle_diff = abs(trk.angle_list[-1] - trk.angle_list[-2])#计算角度偏移量
              if angle_diff < self.angle_judge or angle_diff > (360 - self.angle_judge):#判断的阈值为8度
                  pass
              elif angle_diff > (180 - self.angle_judge) and angle_diff < (180 + self.angle_judge):
                  trk.angle_list[-1] = (trk.angle_list[-1] + 180) % 360
                  self.angle_numbers += 1
              else:
                  trk.angle_list[-1] = trk.angle_list[-2]
                  self.angle_numbers += 1#航向角修正计数
          trk.detec_angle = trk.angle_list[-1]#修正后的角度给检测

          if len(trk.state) > 1:
              thresh = self.dis_thresh
              trk.track_angle = cal_angle(trk.state, thresh, trk.angle_list[-1])#利用位移计算航向角的阈值(暂时觉得速度作为判断条件不太靠谱，采用位移作为判断条件)自己看情况修改吧，可以适当大
          if trk.track_angle is not None:
              trk.final_angle = trk.track_angle
          if trk.track_angle is None:
              trk.final_angle = trk.detec_angle
          #高速目标的航向角优先顺序是轨迹航向角，车道航向角，检测航向角
          #低速目标的航向角优先顺序是车道航向角，轨迹航向角，检测航向角
          # if trk.high_speed:#高速目标的航向角修正部分，根据车道
          #     if trk.track_angle is not None:
          #         head_angle = trk.track_angle
          #         trk.final_angle = head_angle
          #     elif trk.lane_angle is not None:
          #         head_angle = trk.lane_angle
          #         trk.final_angle = head_angle
          #     else:
          #         head_angle = trk.detec_angle
          # else:#低速目标航向角修正部分
          #     if trk.lane_angle is not None:
          #         head_angle = trk.lane_angle
          #         trk.final_angle = head_angle
          #     elif trk.track_angle is not None:
          #         head_angle = trk.track_angle
          #         trk.final_angle = head_angle
          #     else:
          #         head_angle = trk.detec_angle
          # if trk.final_angle is not None:
          #     angle_diff = abs(trk.final_angle - head_angle) % 360
          #     if angle_diff < self.angle_judge or angle_diff > (360 - self.angle_judge):
          #         pass
          #     elif angle_diff > (180 - self.angle_judge) and angle_diff < (180 + self.angle_judge):
          #         head_angle = (head_angle + 180) % 360
          #     else:
          #         head_angle = trk.final_angle
          # trk.head_angle.append(head_angle)

          if ((trk.time_since_update < self.max_age) and (trk.hits >= self.min_hits or self.frame_count <= self.min_hits)):
              # head_final = trk.head_angle[-1]
              head_final = trk.final_angle
              d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
              ret.append(np.concatenate((d_conv, [trk.speed], [trk.id + 1], [0], [0], [0])).reshape(1, -1))
          num_tra -= 1
          if (trk.time_since_update >self.max_age_new):#如果跟踪跟丢超过30帧，弹出目标轨迹，没有进入update的次数
              self.trackers.pop(num_tra)
              # if trk.hits > 10:#一条轨迹进入update的次数，跟踪上的次数，大于10次，存入trackers_bak
              #     self.trackers_bak.append(trk)
      trk_i_frame = copy.deepcopy(self.trackers)
      trk_i.append(trk_i_frame)  # 保存trk值，用于计算速度，判断是否需要进行预测
      if (len(ret) > 0):
          # print("航向角修正计数帧号=", trk.id + 1)#, "    修正次数=", self.angle_numbers)
          return np.concatenate(ret)
      return np.empty((0, 14))
