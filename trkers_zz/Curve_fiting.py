import copy
import math

import numpy as np
from scipy.signal import savgol_filter

from Point_In_Kuang import Point_In_kuang
from compute_iou import Iou_Two_Box

PI_rads = math.pi / 180
Rads_cov = 180.0 / math.pi

def Curve_fiting(fram_list, x_list, y_list, angle_list):
    if len(fram_list) < 20:
        return x_list, y_list, angle_list
    else:
        z1 = np.polyfit(fram_list, x_list, 20)  # 用4次多项式拟合
        p1 = np.poly1d(z1)
        # frame_x = p1(fram_list)

        z2 = np.polyfit(fram_list, y_list, 20)  # 用4次多项式拟合
        p2 = np.poly1d(z2)
        # frame_y = p2(fram_list)

        for i in range(6):
            angle_dev = abs(angle_list[6 - i] - angle_list[5 - i])
            if angle_dev > 3:
                angle_list[5 - i] = angle_list[6 - i]

        z3 = np.polyfit(fram_list, angle_list, 20)  # 用4次多项式拟合
        p3 = np.poly1d(z3)
        # frame_angle = p3(fram_list)

        # 根据曲线拟合进行修正效果并不好，注释掉
        for i in range(len(fram_list)):
            if abs(x_list[i] - p1(fram_list[i])) > 1 and abs(x_list[i] - p1(fram_list[i])) < 2:
                x_list[i] = p1(fram_list[i])
            if abs(y_list[i] - p2(fram_list[i])) > 1 and abs(x_list[i] - p1(fram_list[i])) < 2:
                y_list[i] = p2(fram_list[i])
            # if abs(angle_list[i] - p3(fram_list[i])) > 0.2:
            #     if angle_list[i] < 8 or angle_list[i] > 352:
            #         continue
            #     angle_list[i] = p3(fram_list[i])
        return x_list, y_list, angle_list


def Cur_fit_10(x_list, y_list):
    z1 = np.polyfit(x_list, x_list, 20)  # 用4次多项式拟合
    p1 = np.poly1d(z1)

#*/
# 框，位置，角度的滑动平滑滤波处理
#输入轨迹，输出平滑后的轨迹，根据实际需求，可调整各项参数
#*/
def Huadong_Pinjunfa(det):
    det_ = copy.deepcopy(det)
    if len(det) > 10:
        for i in range(len(det)):
            if i < 2:
                continue
            elif i > len(det) - 3:
                continue
            else:
                det_[i][0] = (det[i - 2][0] * 1.0 + det[i - 1][0] * 2.0 + det[i][0] * 4.0 + det[i + 1][0] * 2.0 + det[i + 2][0] * 1.0) / 10.0
                det_[i][1] = (det[i - 2][1] * 1.0 + det[i - 1][1] * 2.0 + det[i][1] * 4.0 + det[i + 1][1] * 2.0 + det[i + 2][1] * 1.0) / 10.0
                det_[i][2] = (det[i - 2][2] * 1.0 + det[i - 1][2] * 2.0 + det[i][2] * 4.0 + det[i + 1][2] * 2.0 + det[i + 2][2] * 1.0) / 10.0
                det_[i][3] = (det[i - 2][3] * 1.0 + det[i - 1][3] * 2.0 + det[i][3] * 4.0 + det[i + 1][3] * 2.0 + det[i + 2][3] * 1.0) / 10.0
                if abs(det[i][4] - det[i - 2][4]) < 8 and abs(det[i][4] - det[i - 1][4]) < 8 and abs(det[i][4] - det[i + 2][4]) < 8 and abs(det[i][4] - det[i + 1][4]) < 8:
                    det_[i][4] = (det[i - 2][4] * 1.0 + det[i - 1][4] * 2.0 + det[i][4] * 4.0 + det[i + 1][4] * 2.0 + det[i + 2][4] * 1.0) / 10.0
                # else:
                #     if abs(det[i][4] - det[i - 2][4]) > 180:
                #         if det[i][4] > 180:
                #             det[i - 2][4] = det[i - 2][4] + 360
                #         if det[i][4] <= 180:
                #             det[i - 2][4] = det[i - 2][4] - 360
                #
                #     if abs(det[i][4] - det[i - 1][4]) > 180:
                #         if det[i][4] > 180:
                #             det[i - 1][4] = det[i - 1][4] + 360
                #         if det[i][4] <= 180:
                #             det[i - 1][4] = det[i - 1][4] - 360
                #
                #     if abs(det[i][4] - det[i + 1][4]) > 180:
                #         if det[i][4] > 180:
                #             det[i + 1][4] = det[i + 1][4] + 360
                #         if det[i][4] <= 180:
                #             det[i + 1][4] = det[i + 1][4] - 360
                #
                #     if abs(det[i][4] - det[i + 2][4]) > 180:
                #         if det[i][4] > 180:
                #             det[i + 2][4] = det[i + 2][4] + 360
                #         if det[i][4] <= 180:
                #             det[i + 2][4] = det[i + 2][4] - 360
                #     det_[i][4] = (det[i - 2][4] * 1.0 + det[i - 1][4] * 2.0 + det[i][4] * 4.0 + det[i + 1][4] * 2.0 + det[i + 2][4] * 1.0) / 10.0
                # ang_yuzhi = 20
                # if (360 - det[i][4]) < ang_yuzhi or (det[i][4] - 0) < ang_yuzhi:
                #     if (360 - det[i][4]) < ang_yuzhi:
                #         if (det[i - 2][4] - 0) < ang_yuzhi:
                #             det[i - 2][4] = abs(det[i - 2][4] + 360)
                #         if (det[i - 1][4] - 0) < ang_yuzhi:
                #             det[i - 1][4] = abs(det[i - 1][4] + 360)
                #         if (det[i + 1][4] - 0) < ang_yuzhi:
                #             det[i + 1][4] = abs(det[i + 1][4] + 360)
                #         if (det[i + 2][4] - 0) < ang_yuzhi:
                #             det[i + 2][4] = abs(det[i + 2][4] + 360)
                #     if (det[i][4] - 0) < ang_yuzhi:
                #         if (360 - det[i - 2][4]) < ang_yuzhi:
                #             det[i - 2][4] = (det[i - 2][4] - 360)
                #         if (360 - det[i - 1][4]) < ang_yuzhi:
                #             det[i - 1][4] = (det[i - 1][4] - 360)
                #         if (360 - det[i + 1][4]) < ang_yuzhi:
                #             det[i + 1][4] = (det[i + 1][4] - 360)
                #         if (360 - det[i + 2][4]) < ang_yuzhi:
                #             det[i + 2][4] = (det[i + 2][4] - 360)
                #     det_[i][4] = (det[i - 2][4] * 1.0 + det[i - 1][4] * 2.0 + det[i][4] * 4.0 + det[i + 1][4] * 2.0 +
                #                   det[i + 2][4] * 1.0) / 10.0
                # else:
                #     det_[i][4] = (det[i - 2][4] * 1.0 + det[i - 1][4] * 2.0 + det[i][4] * 4.0 + det[i + 1][4] * 2.0 +
                #                   det[i + 2][4] * 1.0) / 10.0
    return det_

# # 计算车辆帧间的距离值
# def Dis_doudong(trk):
#     for dj in range(len(trk)):
#         if dj == len(trk) - 1:
#             break
#         elif dj == 0:
#             continue
#         else:
#             dj_d1_x = trk[dj][0] - trk[dj - 1][0]
#             dj_d1_y = trk[dj][1] - trk[dj - 1][1]
#             dj_dis = math.sqrt(dj_d1_x * dj_d1_x + dj_d1_y * dj_d1_y)
#             if dj_dis > 0.2:  # 两个位置间距离过大，可能存在抖动或者行驶，进行抖动判断
#                m = 0
#                while True:
#                    m += 1
#                    dj_d1_x1 = trk[dj - 1][0] - trk[dj + m][0]
#                    dj_d1_y1 = trk[dj - 1][1] - trk[dj + m][1]
#                    dj_dis10 = math.sqrt(dj_d1_x1 * dj_d1_x1 + dj_d1_y1 * dj_d1_y1)
#
#     return trk

# 计算车辆帧间的距离值
def Dis_det(trk):
    trk_r = []
    for dj in range(len(trk)):
        if dj == len(trk) - 1:
            break
        else:
            dj_d1_x = trk[dj][0] - trk[dj + 1][0]
            dj_d1_y = trk[dj][1] - trk[dj + 1][1]
            dj_dis = math.sqrt(dj_d1_x * dj_d1_x + dj_d1_y * dj_d1_y)
            trk[dj][8] = dj_dis
            if dj_dis > 2.5:
                if len(trk) - 1 - dj < 10:
                    break
            else:
                trk_r.append(trk[dj])
    return trk_r

# 开始静止时，将启动时的航向角给与静止时
def Dis_det_qidong(trk):
    key_bool = True
    for dj in range(len(trk) - 1):
        dj_d1_x = trk[dj][0] - trk[dj + 1][0]
        dj_d1_y = trk[dj][1] - trk[dj + 1][1]
        dj_dis = math.sqrt(dj_d1_x * dj_d1_x + dj_d1_y * dj_d1_y)
        if dj_dis < 0.25:
            continue
        if dj_dis >= 0.25:
            for i in range(dj + 1):
                trk[i][4] = trk[dj + 1][4]
            key_bool = False
            break
        if not key_bool:
            break
    return trk

#计算一条轨迹中，停车的id和启动的id
def DJ_sure(all_trk_ID_det):
    list_dj = {}
    list_tc_id = []
    for dj in range(len(all_trk_ID_det)):
        if dj + 5 > len(all_trk_ID_det):
            break
        if dj - 1 < 0:
            continue
        # 滑动窗口的距离均值
        dj_d_sum = 0
        for dj_d in range(5):
            dj_d1_x = all_trk_ID_det[dj_d + dj][0] - all_trk_ID_det[dj_d + dj - 1][0]
            dj_d1_y = all_trk_ID_det[dj_d + dj][1] - all_trk_ID_det[dj_d + dj - 1][1]
            dj_dis = math.sqrt(dj_d1_x * dj_d1_x + dj_d1_y * dj_d1_y)
            dj_d_sum += dj_dis
        dj_d_sum = dj_d_sum / (all_trk_ID_det[dj + 4][13] - all_trk_ID_det[dj][13])
        d_j = []
        d_j.append(dj_d_sum)                 # 第dj个位置的平均距离值
        d_j.append(1)                        # 默认的停车标志位
        d_j.append(all_trk_ID_det[dj][0])    # 存入该位置的类别
        list_dj[dj] = dj_d_sum               # 存入字典
    for i in list_dj.keys():
        if list_dj[i][0] < 0.1:
            list_c = []
            list_c.append(i)
            list_tc_id.append(i)

    return list_tc_id
    print("done!")
#/*
# 角度修正，转到雷达负y轴
#/*
#通过位置修正航向角
def DJ_sure2(trk, angle_y):
    for ID_ang in range(len(trk)):
        if ID_ang > len(trk) - 1:
            break
        count_add = 1
        while True:
            if ID_ang + count_add > len(trk) - 1:
                break
            dis_x = (trk[ID_ang + count_add][0] - trk[ID_ang][0])
            dis_y = (trk[ID_ang + count_add][1] - trk[ID_ang][1])
            dis_len = math.sqrt(dis_x * dis_x + dis_y * dis_y)
            if trk[0][7] in [0, 2, 5, 6, 1, 3]:
                if dis_len > 1.5:
                    dis_angle = math.acos(dis_x / dis_len) * Rads_cov - 180
                    if dis_y > 0:
                        dis_angle = 90 - dis_angle
                    if dis_y <= 0:
                        dis_angle = 90 - (360 - dis_angle)
                    dis_angle = (dis_angle % 360)
                    for cad in range(count_add):
                        if abs(dis_angle - angle_y[ID_ang + cad]) > 6 or abs(dis_angle - angle_y[ID_ang + cad]) < (
                                360 - 6):
                            angle_y[ID_ang + cad] = dis_angle
                            trk[ID_ang + cad][4] = dis_angle
                            # trk[ID_ang + cad][12] += 1  # 是否存在修正标志位
                    break
                else:
                    count_add += 1
                    continue
            if trk[0][7] not in [0, 2, 5, 6, 1, 3]:
                if dis_len > 1.5:
                    dis_angle = math.acos(dis_x / dis_len) * Rads_cov - 180
                    if dis_y > 0:
                        dis_angle = 90 - dis_angle
                    if dis_y <= 0:
                        dis_angle = 90 - (360 - dis_angle)
                    dis_angle = (dis_angle % 360)
                    for cad in range(count_add):
                        if abs(dis_angle - angle_y[ID_ang + cad]) > 6 or abs(dis_angle - angle_y[ID_ang + cad]) < (360 - 6):
                            angle_y[ID_ang + cad] = dis_angle
                            trk[ID_ang + cad][4] = dis_angle
                            # trk[ID_ang + cad][12] += 1  # 是否存在修正标志位
                    break
                else:
                    count_add += 1
                    continue
    # *********************去掉每个轨迹静止时的航向角，将开始移动的位置的航向角赋值给开头
    if len(trk) > 10:
        for ID_trk_ in range(len(trk) - 1):
            _dev_dis = math.sqrt((trk[ID_trk_ + 1][0] - trk[0][0]) * (
                    trk[ID_trk_ + 1][0] - trk[0][0]) + \
                                 (trk[ID_trk_ + 1][1] - trk[0][1]) * (
                                         trk[ID_trk_ + 1][1] - trk[0][1]))
            if trk[ID_trk_ + 1][4] in [0, 2, 5, 6, 1, 3]:
                if _dev_dis > 1.5:
                    for ID_trk_i in range(ID_trk_ + 1):
                        trk[ID_trk_i][4] = trk[ID_trk_ + 1][4]
                    break
            if trk[ID_trk_ + 1][4] not in [0, 2, 5, 6, 1, 3]:
                if _dev_dis > 0.5:
                    for ID_trk_i in range(ID_trk_ + 1):
                        trk[ID_trk_i][4] = trk[ID_trk_ + 1][4]
                    break
    # *********************去掉每个轨迹静止时的航向角，将开始移动的位置的航向角赋值给开头
    return trk, angle_y

#计算一条轨迹中，停车的id和启动的id
def DJ_sure1(all_trk_ID_det):
    list_dj = {}
    for dj in range(len(all_trk_ID_det)):
        if dj + 3 > len(all_trk_ID_det):
            break
        # 滑动窗口的距离均值
        dj_d_sum = 0
        for dj_d in range(3):
            dj_d1_x = all_trk_ID_det[dj_d + dj][0] - all_trk_ID_det[dj_d + dj][0]
            dj_d1_y = all_trk_ID_det[dj_d + dj][1] - all_trk_ID_det[dj_d + dj][1]
            dj_dis = math.sqrt(dj_d1_x * dj_d1_x + dj_d1_y * dj_d1_y)
            dj_d_sum += dj_dis
        dj_d_sum = dj_d_sum / 3.0
        d_j = []
        d_j.append(dj_d_sum)                 # 0第dj个位置的平均距离值
        d_j.append(1)                        # 1默认的停车标志位
        d_j.append(all_trk_ID_det[dj][0])    # 2存入该位置的类别
        list_dj[dj] = d_j                    # 存入字典
    n = 0
    for i in range(len(list_dj)):
       if list_dj[i][0] < 0.1:
           if i < n:
               continue
           n = i
           if n >= len(list_dj) - 1:
               break
           while True:
               n += 1
               if list_dj[n][0] > 0.2:
                   dis_x = (all_trk_ID_det[n][0] - all_trk_ID_det[i][0])
                   dis_y = (all_trk_ID_det[n][1] - all_trk_ID_det[i][1])
                   dis_len = math.sqrt(dis_x * dis_x + dis_y * dis_y)
                   dis_angle = math.acos(dis_x / dis_len) * Rads_cov - 180
                   if dis_y > 0:
                       dis_angle = 90 - dis_angle
                   if dis_y <= 0:
                       dis_angle = 90 - (360 - dis_angle)
                   dis_angle = (dis_angle % 360)

                   m = n - i
                   for q in range(m):
                       if abs(dis_angle - all_trk_ID_det[i + q][4]) > 6 and abs(dis_angle - all_trk_ID_det[i + q][4]) < (360 - 6):
                           all_trk_ID_det[i + q][4] = dis_angle
                   break
               if n >= len(list_dj) - 1:
                   break
    return all_trk_ID_det

# /*
# 修正目标框
# /*
# *****************************************************************************对框求一个均值
def kuang_fiting(Kuang_size_w, Kuang_size_l, Kuang_size_h, X, Y, trk, kuang_numbers):
    kuang_size_x = 0
    kuang_size_y = 0
    kuang_size_z = 0
    kuang_lenght = 0
    kuang_size_x1 = 0
    kuang_size_y1 = 0
    kuang_size_z1 = 0
    if trk[0][9] == 62:
        print(1)
    for kuang_i in range(len(Kuang_size_w)):
        kuang_size_x1 += Kuang_size_w[kuang_i]
        kuang_size_y1 += Kuang_size_l[kuang_i]
        kuang_size_z1 += Kuang_size_h[kuang_i]
        d_angel_distance = math.sqrt(X[kuang_i] * X[kuang_i] + Y[kuang_i] * Y[kuang_i])
        if d_angel_distance < 52:
            kuang_size_x += Kuang_size_w[kuang_i]
            kuang_size_y += Kuang_size_l[kuang_i]
            kuang_size_z += Kuang_size_h[kuang_i]
            kuang_lenght += 1
    if kuang_lenght >= 5:
        kuang_size_x = kuang_size_x / float(kuang_lenght)
        kuang_size_y = kuang_size_y / float(kuang_lenght)
        kuang_size_z = kuang_size_z / float(kuang_lenght)
    if kuang_lenght < 5:
        kuang_size_x = kuang_size_x1 / len(Kuang_size_w)
        kuang_size_y = kuang_size_y1 / len(Kuang_size_w)
        kuang_size_z = kuang_size_z1 / len(Kuang_size_w)
    # ***********************************************************************修正框的的长宽高

    for ID_trk in range(len(trk)):
        kuang_yuzhi_dev = 0
        if trk[ID_trk][7] in [0, 2, 5, 6]:
            kuang_yuzhi_dev = 0.5
        if trk[ID_trk][7] not in [0, 2, 5, 6]:
            kuang_yuzhi_dev = 0.2
        if abs(trk[ID_trk][2] - kuang_size_x) > kuang_yuzhi_dev or abs(
                trk[ID_trk][3] - kuang_size_y) > kuang_yuzhi_dev:
            kuang_numbers += 1
            # trk[ID_trk][12] += 1  # 是否存在修正标志位
            # 由于框的大小偏小，
            if kuang_lenght >= 5 and trk[ID_trk][7] in [0, 2, 5, 6]:
                trk[ID_trk][2] = kuang_size_x * 1.1
                trk[ID_trk][3] = kuang_size_y * 1.08
                trk[ID_trk][6] = kuang_size_z

            if kuang_lenght < 5 and trk[ID_trk][7] in [0, 2, 5, 6]:
                trk[ID_trk][2] = kuang_size_x * 1.16
                trk[ID_trk][3] = kuang_size_y * 1.1
                trk[ID_trk][6] = kuang_size_z

            if kuang_lenght < 5 and trk[ID_trk][7] not in [0, 2, 5, 6]:
                trk[ID_trk][2] = kuang_size_x
                trk[ID_trk][3] = kuang_size_y
                trk[ID_trk][6] = kuang_size_z

            if kuang_lenght >= 5 and trk[ID_trk][7] not in [0, 2, 5, 6]:
                trk[ID_trk][2] = kuang_size_x
                trk[ID_trk][3] = kuang_size_y
                trk[ID_trk][6] = kuang_size_z
    Kuang_size_w.clear()  # 目标框修正完毕，释放列表内存
    Kuang_size_l.clear()  # 目标框修正完毕，释放列表内存
    Kuang_size_h.clear()  # 目标框修正完毕，释放列表内存
    return X, Y, trk, kuang_numbers

#/*
#修正类别
#/*
# 计算该ID下最多的class类别
def class_fitting(class_label, trk, class_numbers):
    class_dict = {}
    class_key_max = 0  # 每个key值的长度
    class_label_max_ = 0  # 最多label的class类别
    for class_i in class_label:
        if class_i in class_dict:
            class_dict[class_i].append(class_i)
        if class_i not in class_dict:
            class_dict[class_i] = []
            class_dict[class_i].append(class_i)
    for class_key in class_dict.keys():
        if len(class_dict[class_key]) > class_key_max:
            class_key_max = len(class_dict[class_key])
            class_label_max_ = class_dict[class_key][0]
    # 目标ID类别修正，修正原则为，个别类别跳变，其修正为该ID下大多数的类别
    for class_error in range(len(trk)):
        if trk[class_error][7] != class_label_max_:
            class_numbers += 1
            # trk[class_error][12] += 1  # 是否存在修正标志位
            trk[class_error][7] = class_label_max_
    class_dict.clear()  # 类别修正完毕，释放列表内存
    class_label.clear()  # 类别修正完毕，释放列表内存
    return class_label_max_, trk, class_numbers

# # # 遍历所有帧的轨迹，对点云去除地面点，查看一条轨迹中的位置是否存在丢帧情况，若存在丢帧情况，根据前后帧求一个中间的box，查看box中有无点云
# # # 若存在点云，判断点云数量，点云数量大于一定阈值，算该帧为漏检目标，添加进轨迹链中
def DZ_Fitting(all_trk_ID_det, frame_boxs_ori, all_trk_fram):
    diu_frame = 0
    all_trk_ID_det_ = copy.deepcopy(all_trk_ID_det)
    for i in all_trk_ID_det_.keys():
        if all_trk_ID_det_[i][0][7] in [0, 2, 5, 6, 1, 3]:
            for j in range(len(all_trk_ID_det_[i]) - 2):
                frame_dev = all_trk_ID_det_[i][j + 1][13] - all_trk_ID_det_[i][j][13]
                d1 = all_trk_ID_det_[i][j + 1][0] - all_trk_ID_det_[i][j][0]
                d2 = all_trk_ID_det_[i][j + 1][1] - all_trk_ID_det_[i][j][1]
                d = math.sqrt(d1 * d1 + d2 * d2)
                # if frame_dev > 1 and d >= 1:
                #     path = "./data_save/DZ/" + str(int(all_trk_ID_det_[i][j + 1][9])) + "-" + str(int(all_trk_ID_det_[i][j + 1][7])) + ".csv"
                #     np.savetxt(path, all_trk_ID_det_[i], delimiter=',')
                if frame_dev > 1 and d < 2.5:
                    Dev = len(all_trk_ID_det_[i]) - j # 丢帧的位置
                    # path = "./data_save/DZ/" + str(int(all_trk_ID_det_[i][j + 1][9])) + "-" + str(int(all_trk_ID_det_[i][j + 1][7])) + ".csv"
                    # np.savetxt(path, all_trk_ID_det_[i], delimiter=',')
                    # if abs(all_trk_ID_det_[i][j][4] - 270) < 2 or abs(all_trk_ID_det_[i][j][4] - 90) < 2:
                    if abs(all_trk_ID_det_[i][j + 1][4] - all_trk_ID_det_[i][j][4]) < 8:
                        diu_frame += 1
                        print("该轨迹存在丢帧：丢帧ID=", all_trk_ID_det_[i][0][9], "  丢帧间隔=", frame_dev, "   丢帧的距离长度=", d, "   丢帧的帧在轨迹中的位置=", Dev)
                        x_dev1 = (all_trk_ID_det_[i][j][0] - all_trk_ID_det_[i][j - 1][0])
                        y_dev1 = (all_trk_ID_det_[i][j][1] - all_trk_ID_det_[i][j - 1][1])

                        x_dev2 = (all_trk_ID_det_[i][j + 1][0] - all_trk_ID_det_[i][j + 2][0])
                        y_dev2 = (all_trk_ID_det_[i][j + 1][1] - all_trk_ID_det_[i][j + 2][1])
                        D2 = math.sqrt(x_dev2 * x_dev2 + y_dev2 * y_dev2)  # 后
                        D1 = math.sqrt(x_dev1 * x_dev1 + y_dev1 * y_dev1)  # 前

                        x_dev = (all_trk_ID_det_[i][j + 1][0] - all_trk_ID_det_[i][j][0]) / frame_dev
                        y_dev = (all_trk_ID_det_[i][j + 1][1] - all_trk_ID_det_[i][j][1]) / frame_dev

                        if D1 < 0.1 and D2 < 0.1:  # 动静判断
                            for k in range(int(frame_dev)):
                                box_add = True
                                trk = copy.deepcopy(all_trk_ID_det_[i][j])
                                if k == 0:
                                    continue
                                trk[0] = trk[0]
                                trk[1] = trk[1]
                                trk[11] = 300
                                trk[13] = int(all_trk_ID_det_[i][j][13] + k)
                                for trk_box in frame_boxs_ori[int(all_trk_ID_det_[i][j][13] + k)]:  #
                                    iou = Iou_Two_Box(trk_box, trk)
                                    if iou > 0.1:
                                        box_add = False
                                if box_add:
                                    all_trk_ID_det[i].append(trk)
                                    all_trk_fram[trk[13]].append(trk[9])
                                if box_add == False:
                                    continue
                        if D1 >= 0.1 and D2 >= 0.1:  # 动静判断
                            for k in range(int(frame_dev)):
                                box_add = True
                                trk = copy.deepcopy(all_trk_ID_det_[i][j])
                                if k == 0:
                                    continue
                                trk[0] = trk[0] + k * x_dev
                                trk[1] = trk[1] + k * y_dev
                                trk[11] = 300
                                trk[13] = int(all_trk_ID_det_[i][j][13] + k)
                                for trk_box in frame_boxs_ori[int(all_trk_ID_det_[i][j][13] + k)]:
                                    iou = Iou_Two_Box(trk_box, trk)
                                    if iou > 0:
                                        box_add = False
                                # for trk_box in frame_boxs_ori[int(all_trk_ID_det_[i][j][13] + k - 1)]:  #
                                #     iou = Iou_Two_Box(trk_box, trk)
                                #     if iou > 0:
                                #         box_add = False
                                if box_add:
                                    all_trk_ID_det[i].append(trk)
                                    all_trk_fram[trk[13]].append(trk[9])
                                if box_add == False:
                                    continue
                        if D1 >= 0.1 and D2 < 0.1:  # 动静判断
                            for k in range(int(frame_dev)):
                                box_add = True
                                trk = copy.deepcopy(all_trk_ID_det_[i][j + 1])
                                if k == 0:
                                    continue
                                trk[0] = trk[0]
                                trk[1] = trk[1]
                                trk[11] = 300
                                trk[13] = int(all_trk_ID_det_[i][j][13] + k)
                                for trk_box in frame_boxs_ori[int(all_trk_ID_det_[i][j][13] + k)]:
                                    iou = Iou_Two_Box(trk_box, trk)
                                    if iou > 0:
                                        box_add = False
                                # for trk_box in frame_boxs_ori[int(all_trk_ID_det_[i][j][13] + k - 1)]:  #
                                #     iou = Iou_Two_Box(trk_box, trk)
                                #     if iou > 0:
                                #         box_add = False
                                if box_add:
                                    all_trk_ID_det[i].append(trk)
                                    all_trk_fram[trk[13]].append(trk[9])
                                if box_add == False:
                                    continue
                        if D1 < 0.1 and D2 >= 0.1:  # 动静判断
                            for k in range(int(frame_dev)):
                                box_add = True
                                trk = copy.deepcopy(all_trk_ID_det_[i][j])
                                if k == 0:
                                    continue
                                trk[0] = trk[0]
                                trk[1] = trk[1]
                                trk[11] = 300
                                trk[13] = int(all_trk_ID_det_[i][j][13] + k)
                                for trk_box in frame_boxs_ori[int(all_trk_ID_det_[i][j][13] + k)]:
                                    iou = Iou_Two_Box(trk_box, trk)
                                    if iou > 0:
                                        box_add = False
                                # for trk_box in frame_boxs_ori[int(all_trk_ID_det_[i][j][13] + k - 1)]:  #
                                #     iou = Iou_Two_Box(trk_box, trk)
                                #     if iou > 0:
                                #         box_add = False
                                if box_add:
                                    all_trk_ID_det[i].append(trk)
                                    all_trk_fram[trk[13]].append(trk[9])
                                if box_add == False:
                                    continue
    all_trk_ID_det_.clear()
    return all_trk_ID_det, all_trk_fram, diu_frame

def DZ_Fitting_new(all_trk_ID_det, frame_boxs_ori, all_trk_fram):
    diu_frame = 0
    all_trk_ID_det_ = copy.deepcopy(all_trk_ID_det)
    for i in all_trk_ID_det_.keys():
        if all_trk_ID_det_[i][0][7] in [0, 1]:
            for j in range(len(all_trk_ID_det_[i]) - 2):
                frame_dev = all_trk_ID_det_[i][j + 1][13] - all_trk_ID_det_[i][j][13]
                d1 = all_trk_ID_det_[i][j + 1][0] - all_trk_ID_det_[i][j][0]
                d2 = all_trk_ID_det_[i][j + 1][1] - all_trk_ID_det_[i][j][1]
                d = math.sqrt(d1 * d1 + d2 * d2)
                # if frame_dev > 1 and d >= 1:
                #     path = "./data_save/DZ/" + str(int(all_trk_ID_det_[i][j + 1][9])) + "-" + str(int(all_trk_ID_det_[i][j + 1][7])) + ".csv"
                #     np.savetxt(path, all_trk_ID_det_[i], delimiter=',')
                if frame_dev > 1 and d < 10:
                    Dev = len(all_trk_ID_det_[i]) - j # 丢帧的位置
                    # path = "./data_save/DZ/" + str(int(all_trk_ID_det_[i][j + 1][9])) + "-" + str(int(all_trk_ID_det_[i][j + 1][7])) + ".csv"
                    # np.savetxt(path, all_trk_ID_det_[i], delimiter=',')
                    # if abs(all_trk_ID_det_[i][j][4] - 270) < 2 or abs(all_trk_ID_det_[i][j][4] - 90) < 2:
                    if abs(all_trk_ID_det_[i][j + 1][4] - all_trk_ID_det_[i][j][4]) < 8:
                        diu_frame += 1
                        print("该轨迹存在丢帧：丢帧ID=", all_trk_ID_det_[i][0][9], "  丢帧间隔=", frame_dev, "   丢帧的距离长度=", d, "   丢帧的帧在轨迹中的位置=", Dev)
                        x_dev1 = (all_trk_ID_det_[i][j][0] - all_trk_ID_det_[i][j - 1][0])
                        y_dev1 = (all_trk_ID_det_[i][j][1] - all_trk_ID_det_[i][j - 1][1])

                        x_dev2 = (all_trk_ID_det_[i][j + 1][0] - all_trk_ID_det_[i][j + 2][0])
                        y_dev2 = (all_trk_ID_det_[i][j + 1][1] - all_trk_ID_det_[i][j + 2][1])
                        D2 = math.sqrt(x_dev2 * x_dev2 + y_dev2 * y_dev2)  # 后
                        D1 = math.sqrt(x_dev1 * x_dev1 + y_dev1 * y_dev1)  # 前

                        x_dev = (all_trk_ID_det_[i][j + 1][0] - all_trk_ID_det_[i][j][0]) / frame_dev
                        y_dev = (all_trk_ID_det_[i][j + 1][1] - all_trk_ID_det_[i][j][1]) / frame_dev

                        box_add = True
                        if D1 < 0.1 and D2 < 0.1:  # 动静判断
                            for k in range(int(frame_dev)):
                                trk = copy.deepcopy(all_trk_ID_det_[i][j])
                                if k == 0:
                                    continue
                                trk[0] = trk[0]
                                trk[1] = trk[1]
                                trk[11] = 300
                                trk[13] = int(all_trk_ID_det_[i][j][13] + k)
                                for trk_box in frame_boxs_ori[int(all_trk_ID_det_[i][j][13] + k)]:  #
                                    iou = Iou_Two_Box(trk_box, trk)
                                    if iou > 0.1:
                                        box_add = False
                                if box_add:
                                    all_trk_ID_det[i].append(trk)
                                    all_trk_fram[trk[13]].append(trk[9])
                                if box_add == False:
                                    continue
                        # if D1 > 0.1 and D2 > 0.1:  # 动静判断
                        #     for k in range(int(frame_dev)):
                        #         trk = copy.deepcopy(all_trk_ID_det_[i][j])
                        #         if k == 0:
                        #             continue
                        #         trk[0] = trk[0] + k * x_dev
                        #         trk[1] = trk[1] + k * y_dev
                        #         trk[11] = 300
                        #         trk[13] = int(all_trk_ID_det_[i][j][13] + k)
                        #         for trk_box in frame_boxs_ori[int(all_trk_ID_det_[i][j][13] + k)]:
                        #             iou = Iou_Two_Box(trk_box, trk)
                        #             if iou > 0:
                        #                 box_add = False
                        #         for trk_box in frame_boxs_ori[int(all_trk_ID_det_[i][j][13] + k - 1)]:  #
                        #             iou = Iou_Two_Box(trk_box, trk)
                        #             if iou > 0:
                        #                 box_add = False
                        #         if box_add:
                        #             all_trk_ID_det[i].append(trk)
                        #             all_trk_fram[trk[13]].append(trk[9])
                        #         if box_add == False:
                        #             continue
                        # if D1 > 0.1 and D2 < 0.1:  # 动静判断
                        #     for k in range(int(frame_dev)):
                        #         trk = copy.deepcopy(all_trk_ID_det_[i][j + 1])
                        #         if k == 0:
                        #             continue
                        #         trk[0] = trk[0]
                        #         trk[1] = trk[1]
                        #         trk[11] = 300
                        #         trk[13] = int(all_trk_ID_det_[i][j][13] + k)
                        #         for trk_box in frame_boxs_ori[int(all_trk_ID_det_[i][j][13] + k)]:
                        #             iou = Iou_Two_Box(trk_box, trk)
                        #             if iou > 0:
                        #                 box_add = False
                        #         for trk_box in frame_boxs_ori[int(all_trk_ID_det_[i][j][13] + k - 1)]:  #
                        #             iou = Iou_Two_Box(trk_box, trk)
                        #             if iou > 0:
                        #                 box_add = False
                        #         if box_add:
                        #             all_trk_ID_det[i].append(trk)
                        #             all_trk_fram[trk[13]].append(trk[9])
                        #         if box_add == False:
                        #             continue
                        # if D1 < 0.1 and D2 > 0.1:  # 动静判断
                        #     for k in range(int(frame_dev)):
                        #         trk = copy.deepcopy(all_trk_ID_det_[i][j])
                        #         if k == 0:
                        #             continue
                        #         trk[0] = trk[0]
                        #         trk[1] = trk[1]
                        #         trk[11] = 300
                        #         trk[13] = int(all_trk_ID_det_[i][j][13] + k)
                        #         for trk_box in frame_boxs_ori[int(all_trk_ID_det_[i][j][13] + k)]:
                        #             iou = Iou_Two_Box(trk_box, trk)
                        #             if iou > 0:
                        #                 box_add = False
                        #         for trk_box in frame_boxs_ori[int(all_trk_ID_det_[i][j][13] + k - 1)]:  #
                        #             iou = Iou_Two_Box(trk_box, trk)
                        #             if iou > 0:
                        #                 box_add = False
                        #         if box_add:
                        #             all_trk_ID_det[i].append(trk)
                        #             all_trk_fram[trk[13]].append(trk[9])
                        #         if box_add == False:
                        #             continue
    all_trk_ID_det_.clear()
    return all_trk_ID_det, all_trk_fram, diu_frame

#/*
#根据车道的大致航向角进行修正
#/
def Line_Angel_Fitting(all_trk_ID_det, region_list):
    ang = np.loadtxt("config/config_ang.txt")
    ang1 = ang[0]
    ang2 = ang[1]
    ang3 = ang[2]
    list_shiru_right1_f = region_list[0][0]
    list_shiru_right2_f = region_list[0][1]
    list_shiru_right3_be = region_list[0][2]
    list_shiru_right4_be = region_list[0][3]

    # list_shiru_right1_f = [region_list[0][0][0], region_list[0][0][1]]
    # list_shiru_right2_f = [region_list[0][1][0], region_list[0][1][1]]
    # list_shiru_right3_be = [region_list[0][2][0], region_list[0][2][1]]
    # list_shiru_right4_be = [region_list[0][3][0], region_list[0][3][1]]

    list_shiru_left4_f = region_list[1][3]
    list_shiru_left3_f = region_list[1][2]
    list_shiru_left2_be = region_list[1][1]
    list_shiru_left1_be = region_list[1][0]
    #
    list_down1 = region_list[2][0]
    list_down2 = region_list[2][1]
    list_down3 = region_list[2][2]
    list_down4 = region_list[2][3]


    for i in all_trk_ID_det.keys():
        if all_trk_ID_det[i][0][7] in [4, 1, 3]:
            continue
        for j in range(len(all_trk_ID_det[i])):
            q1 = Point_In_kuang(all_trk_ID_det[i][j][0], all_trk_ID_det[i][j][1], list_shiru_right1_f,
                                list_shiru_right2_f, list_shiru_right3_be, list_shiru_right4_be)
            q2 = Point_In_kuang(all_trk_ID_det[i][j][0], all_trk_ID_det[i][j][1], list_shiru_left1_be,
                                list_shiru_left2_be, list_shiru_left3_f, list_shiru_left4_f)
            q3 = Point_In_kuang(all_trk_ID_det[i][j][0], all_trk_ID_det[i][j][1], list_down1, list_down2, list_down3,
                                list_down4)
            if q1 == True:
                all_trk_ID_det[i][j][4] = ang1
            if q2 == True:
                all_trk_ID_det[i][j][4] = ang2
            if q3 == True:
                if all_trk_ID_det[i][j][4] < 200 and all_trk_ID_det[i][j][4] > 150:
                    all_trk_ID_det[i][j][4] = ang3
                else:
                    all_trk_ID_det[i][j][4] = 2

    return all_trk_ID_det

def savgol_filter_xy(all_trk_ID_det, window_length, polyorder):   # x 拟合数据，窗口宽度，拟合阶数
    for i in all_trk_ID_det.keys():
        x = []
        y = []
        for j in range(len(all_trk_ID_det[i])):
            x.append(all_trk_ID_det[i][j][0])
            y.append(all_trk_ID_det[i][j][1])
        tmp_x = np.array(x).T
        tmp_y = np.array(y).T
        if len(all_trk_ID_det[i]) < 9:
            continue
        tmp_smoothx = savgol_filter(tmp_x, window_length, polyorder)
        tmp_smoothy = savgol_filter(tmp_y, window_length, polyorder)
        for j in range(len(all_trk_ID_det[i])):
            all_trk_ID_det[i][j][0] = tmp_smoothx[j]
            all_trk_ID_det[i][j][1] = tmp_smoothy[j]
    return all_trk_ID_det








if __name__=='__main__':
    x = [1,2,34,5,6,7,0,5,8,23,6,432,3]
    tem = np.array(x).T
    tmp_smooth = savgol_filter_xy(tem, 3, 2)
    print (tmp_smooth)