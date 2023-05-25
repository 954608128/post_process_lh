import copy
import math

from Point_In_Kuang import Point_In_kuang
from compute_iou import Iou_Two_Box
Rads_cov = 180 / math.pi
PI_rads = math.pi / 180

list_shiru_right1_f = [50.052, 15.91]
list_shiru_right2_f = [50.2129, 2.78]
list_shiru_right3_be = [97.45, 15.02]
list_shiru_right4_be = [96.6, 2.39]

list_shiru_left3_f = [-29.99, 27.33]
list_shiru_left4_f = [-20.45, 5.83]
list_shiru_left2_be = [-85.75, 5.73]
list_shiru_left1_be = [-85.99, 27.19]


########去除虚检的轨迹#################
def No_Xujian(all_trk_ID_det):
    all_trk_ID_det_quxujian = copy.deepcopy(all_trk_ID_det)
    all_trk_ID_det.clear()
    for i in all_trk_ID_det_quxujian.keys():  # 遍历字典all_trk_ID
        trk_ID__ = all_trk_ID_det_quxujian[i]  # 取得key值下的轨迹
        if trk_ID__[0][7] in [0, 1]:
            d1 = abs(trk_ID__[len(trk_ID__) - 1][0] - trk_ID__[0][0])
            d2 = abs(trk_ID__[len(trk_ID__) - 1][1] - trk_ID__[0][1])
            d = math.sqrt(d1 * d1 + d2 * d2)
            if d > 5:
                all_trk_ID_det[i] = trk_ID__
                # if len(trk_ID__) > 50:
                #     all_trk_ID_det[i] = trk_ID__
                # if len(trk_ID__) <= 50:
                #     all_trk_ID_det[i] = trk_ID__
                    # xujian_yichang = 0
                    # speed_max = 0
                    # speed_max_sup = 0
                    # speed_min = 0
                    # for j in range(len(trk_ID__)):
                    #     q2 = Point_In_kuang(trk_ID__[j][0], trk_ID__[j][1], list_shiru_left1_be,
                    #                          list_shiru_left2_be, list_shiru_left3_f, list_shiru_left4_f)
                    #     q1 = Point_In_kuang(trk_ID__[j][0], trk_ID__[j][1], list_shiru_right1_f,
                    #                         list_shiru_right2_f, list_shiru_right3_be, list_shiru_right4_be)
                    #     if q1 == True:
                    #         if abs(trk_ID__[j][4] - 91) < 8 or abs(abs(trk_ID__[j][4] - 270) < 8):
                    #             xujian_yichang += 1
                    #     if q2 == True:
                    #         if abs(trk_ID__[j][4] - 91) < 8 or abs(abs(trk_ID__[j][4] - 270) < 8):
                    #             xujian_yichang += 1
                    #     if j + 1 < len(trk_ID__):
                    #         dx = (trk_ID__[j + 1][0] - trk_ID__[j][0])
                    #         dy = (trk_ID__[j + 1][1] - trk_ID__[j][1])
                    #         speed = math.sqrt(dx * dx + dy * dy) / (trk_ID__[j + 1][13] - trk_ID__[j][13])
                    #         if speed > 1.0:
                    #             speed_max += 1
                    #         if speed > 3:
                    #             speed_max_sup += 1
                    #         if speed <= 1.0:
                    #             speed_min += 1
                    # if speed_min == 0:
                    #     spee_bilv = 1
                    # if speed_min > 0:
                    #     spee_bilv = speed_max / speed_min # 判断该轨迹的速度变化异常情况的比率，比率大则判断轨迹异常
                    # if xujian_yichang > 1 and spee_bilv < 0.5 and speed_max_sup <= 3:
                    #     all_trk_ID_det[i] = trk_ID__
        else:
            d1 = abs(trk_ID__[len(trk_ID__) - 1][0] - trk_ID__[0][0])
            d2 = abs(trk_ID__[len(trk_ID__) - 1][1] - trk_ID__[0][1])
            d = math.sqrt(d1 * d1 + d2 * d2)
            if d > 0.5:
               all_trk_ID_det[i] = trk_ID__
    return all_trk_ID_det

def No_ID_Change(all_trk_ID_det):
    ID_number = 0
    ############找寻ID跳变###################################
    all_trk_ID_det_IDtiaobain = copy.deepcopy(all_trk_ID_det)
    all_trk_ID_det_IDtiaobain_ = copy.deepcopy(all_trk_ID_det)
    list_id = [-1]
    list_id_ = {}
    for i in all_trk_ID_det_IDtiaobain_.keys():
        trk_ID__ = all_trk_ID_det_IDtiaobain_[i]
        if i in list_id:
            continue
        d = math.sqrt(trk_ID__[len(trk_ID__) - 1][0] * trk_ID__[len(trk_ID__) - 1][0] + trk_ID__[len(trk_ID__) - 1][1] * trk_ID__[len(trk_ID__) - 1][1])
        if d < 80 and trk_ID__[0][7] in [0, 2, 5, 6, 1, 3]:
            for j in all_trk_ID_det_IDtiaobain.keys():
                if j in list_id:
                    continue
                trk_ID1__ = all_trk_ID_det_IDtiaobain[j]
                d1 = math.sqrt(trk_ID1__[0][0] * trk_ID1__[0][0] + trk_ID1__[0][1] * trk_ID1__[0][1])
                if abs(d1 - d) < 2:
                    d2_x = trk_ID__[len(trk_ID__) - 1][0] - trk_ID1__[0][0]
                    d2_y = trk_ID__[len(trk_ID__) - 1][1] - trk_ID1__[0][1]
                    d = math.sqrt(d2_x * d2_x + d2_y * d2_y)
                    if d < 2 and trk_ID1__[0][13] - trk_ID__[len(trk_ID__) - 1][13] >= 1 and trk_ID1__[0][13] - trk_ID__[len(trk_ID__) - 1][13] < 10 and trk_ID1__[0][7] == trk_ID__[len(trk_ID__) - 1][7]:
                        list_id_[i] = [i, j]
                        if i not in list_id:
                            list_id.append(i)
                        if j not in list_id:
                            list_id.append(j)
                        break
    for key in list_id_.keys():
        value = all_trk_ID_det.pop(list_id_[key][1])
        ID_number += 1
        for trk in value:
            trk[7] = all_trk_ID_det[list_id_[key][0]][0][7]
            trk[9] = all_trk_ID_det[list_id_[key][0]][0][9]
            all_trk_ID_det[list_id_[key][0]].append(trk)

    return all_trk_ID_det, ID_number


def No_ID_Changen_new(all_trk_ID_det, tracker_news, all_trk_fram):
    list_id_done = []
    trk_news = copy.deepcopy(all_trk_ID_det)
    for i in all_trk_ID_det.keys():
        if all_trk_ID_det[i][0][13] == -1:
            continue
        print("车辆断链修正", i)
        dit_distance_id = {}           # 临时存储找到的id，用于对比那个id更合适
        dit_iou_id = {}
        dx = all_trk_ID_det[i][len(all_trk_ID_det[i]) - 1][0]
        dy = all_trk_ID_det[i][len(all_trk_ID_det[i]) - 1][1]
        d = math.sqrt(dx * dx + dy * dy)          # 计算轨迹末尾距离原点的距离

        dx_dev = dx - all_trk_ID_det[i][len(all_trk_ID_det[i]) - 2][0]
        dy_dev = dy - all_trk_ID_det[i][len(all_trk_ID_det[i]) - 2][1]
        d_dev = math.sqrt(dx_dev * dx_dev + dy_dev * dy_dev)
        # if i == 829:
        #     print("找到881ID号", i)
        if d < 70:     # 距离小于70，可能存在断链
            list_id_done.append(i)
            frame = all_trk_ID_det[i][len(all_trk_ID_det[i]) - 1][13]
            if frame > len(tracker_news):
                continue
            n = 0
            box_tiaobian = all_trk_ID_det[i][len(all_trk_ID_det[i]) - 1]        # 跳变轨迹的末端位置box
            while True:
                continue_seach = False
                if int(frame + n + 1) > len(tracker_news) - 1:
                    break
                for box in tracker_news[int(frame + n + 1)]:
                    if all_trk_ID_det[box[9]][0][13] == -1:                     # 已经找过的轨迹
                        continue
                    if all_trk_ID_det[box[9]][0][13] != int(frame + n + 1):     # 该box不是轨迹的起始位置
                        continue
                    iou = Iou_Two_Box(box, box_tiaobian)  # 计算两个box的iou
                    dxj = box[0] - box_tiaobian[0]
                    dyj = box[1] - box_tiaobian[1]
                    dj = math.sqrt(dxj * dxj + dyj * dyj)     # 计算连个box的距离
                    ID_ori = box_tiaobian[9]
                    ID_xiu = box[9]
                    if iou > 0.1:                             # 若两个box的iou大于0.2，这该两个box可能是同一个ID
                        # for id in range(len(all_trk_ID_det[ID_xiu])):   # 遍历轨迹，将ID修正过来
                        #     all_trk_ID_det[ID_xiu][id][9] = ID_ori
                        # continue_seach = True
                        # break
                        print("统计计算iou")
                        dit_iou_id[ID_xiu] = iou
                    if d_dev < 0.08:
                        d_dev = 0.15
                    if iou <= 0.1 and dj < (n+1) * (d_dev + 0.1):             # 若iou小于0.2，距离小于8m，判断找到的box对应的轨迹是否具有断链
                        # 计算找的box与跳变位置的box间的航向
                        dis_len = (dxj * dxj + dyj * dyj) ** 0.5
                        dis_angle = 0
                        if dis_len > 0.2:
                            # 转到y负轴
                            dis_angle = math.acos(dxj / dis_len) * Rads_cov - 180
                            if dyj > 0:
                                dis_angle = 90 - dis_angle
                            if dyj <= 0:
                                dis_angle = 90 - (360 - dis_angle)
                            dis_angle = (dis_angle % 360)
                        if abs(dis_angle - box[4]) < 8 or abs(dis_angle + 180 - box[4]) < 8:           # 若航向角相差不大，说明在一条航线上，并在行驶方向上
                            print("统计计算距离")
                            dit_distance_id[ID_xiu] = dj
                if continue_seach:
                    dit_distance_id.clear()
                    dit_iou_id.clear()
                    print("结束修正", i, "开始下一个轨迹断链修正", i + 1)
                    break

                n = n + 1
                max_iou = 0          # 找到最大iou对应的id
                for iou_id in dit_iou_id.keys():
                    if dit_iou_id[iou_id] > max_iou:
                        max_iou = dit_iou_id[iou_id]
                for iou_id in dit_iou_id.keys():
                    if dit_iou_id[iou_id] == max_iou:
                        for id in range(len(all_trk_ID_det[iou_id])):   # 遍历轨迹，将ID修正过来
                            trk = copy.deepcopy(all_trk_ID_det[iou_id][id])
                            trk[9] = ID_ori
                            if ID_ori not in all_trk_fram[trk[13]]:
                                all_trk_fram[trk[13]].append(ID_ori)
                            all_trk_ID_det[ID_ori].append(trk)
                            all_trk_ID_det[iou_id][id][13] = -1
                            print("距离断链修正")
                        m = 0
                        while True:
                            dx_cn = all_trk_ID_det[ID_ori][len(all_trk_ID_det[ID_ori]) - 1][0]
                            dy_cn = all_trk_ID_det[ID_ori][len(all_trk_ID_det[ID_ori]) - 1][1]
                            d_cn = math.sqrt(dx_cn * dx_cn + dy_cn * dy_cn)
                            cn = True
                            if d_cn < 70 and cn and m < 6:
                                all_trk_ID_det, tracker_news, all_trk_fram, all_trk_ID_det[ID_ori], cn = find_more(
                                    all_trk_ID_det, tracker_news, all_trk_fram, all_trk_ID_det[ID_ori], m)
                                m = m + 1
                            else:
                                break
                        continue_seach = True
                        break
                if continue_seach:
                    dit_distance_id.clear()
                    dit_iou_id.clear()
                    break
                if not continue_seach:
                    mind = 500
                    for did in dit_distance_id.keys():
                        if dit_distance_id[did] < mind:
                            mind = dit_distance_id[did]
                    for did in dit_distance_id.keys():
                        if dit_distance_id[did] == mind:
                            for id in range(len(all_trk_ID_det[did])):  # 遍历轨迹，将ID修正过来
                                trk = copy.deepcopy(all_trk_ID_det[did][id])
                                trk[9] = ID_ori
                                if ID_ori not in all_trk_fram[trk[13]]:
                                    all_trk_fram[trk[13]].append(ID_ori)
                                all_trk_ID_det[ID_ori].append(trk)
                                all_trk_ID_det[did][id][13] = -1
                                print("距离断链修正")
                            m = 0
                            while True:
                                dx_cn = all_trk_ID_det[ID_ori][len(all_trk_ID_det[ID_ori]) - 1][0]
                                dy_cn = all_trk_ID_det[ID_ori][len(all_trk_ID_det[ID_ori]) - 1][1]
                                d_cn = math.sqrt(dx_cn * dx_cn + dy_cn * dy_cn)
                                cn = True
                                if d_cn < 70 and cn and m < 6:
                                    all_trk_ID_det, tracker_news, all_trk_fram, all_trk_ID_det[ID_ori], cn= find_more(
                                        all_trk_ID_det, tracker_news, all_trk_fram, all_trk_ID_det[ID_ori], m)
                                    m = m + 1
                                else:
                                    break
                            continue_seach = True
                            break
                if continue_seach:
                    dit_distance_id.clear()
                    dit_iou_id.clear()
                    print("结束修正", i, "开始下一个轨迹断链修正", i + 1)
                    break

    return all_trk_ID_det, True, all_trk_fram

def find_more(all_trk_ID_det, tracker_news, all_trk_fram, sigel_trk, m):
    dx = sigel_trk[len(sigel_trk) - 1][0]
    dy = sigel_trk[len(sigel_trk) - 1][1]
    dx_dev = dx - sigel_trk[len(sigel_trk) - 2][0]
    dy_dev = dy - sigel_trk[len(sigel_trk) - 2][1]
    d_dev = math.sqrt(dx_dev * dx_dev + dy_dev * dy_dev)

    dit_distance_id = {}  # 临时存储找到的id，用于对比那个id更合适
    dit_iou_id = {}
    frame = sigel_trk[len(sigel_trk) - 1][13]
    if frame > len(tracker_news):
        return all_trk_ID_det, tracker_news, all_trk_fram, sigel_trk, False
    n = 0
    box_tiaobian = sigel_trk[len(sigel_trk) - 1]  # 跳变轨迹的末端位置box
    while True:
        continue_seach = False
        if int(frame + n + 1) > len(tracker_news) - 1:
            break
        for box in tracker_news[int(frame + n + 1)]:
            if all_trk_ID_det[box[9]][0][13] == -1:  # 已经找过的轨迹
                continue
            if all_trk_ID_det[box[9]][0][13] != int(frame + n + 1):  # 该box不是轨迹的起始位置
                continue
            iou = Iou_Two_Box(box, box_tiaobian)  # 计算两个box的iou
            dxj = box[0] - box_tiaobian[0]
            dyj = box[1] - box_tiaobian[1]
            dj = math.sqrt(dxj * dxj + dyj * dyj)  # 计算连个box的距离
            ID_ori = box_tiaobian[9]
            ID_xiu = box[9]
            if iou > 0.1:  # 若两个box的iou大于0.2，这该两个box可能是同一个ID
                # for id in range(len(all_trk_ID_det[ID_xiu])):   # 遍历轨迹，将ID修正过来
                #     all_trk_ID_det[ID_xiu][id][9] = ID_ori
                # continue_seach = True
                # break
                print("统计计算iou")
                dit_iou_id[ID_xiu] = iou
            if d_dev < 0.08:
                d_dev = 0.15
            if iou <= 0.1 and dj < (n + 1) * (d_dev + 0.1):  # 若iou小于0.2，距离小于8m，判断找到的box对应的轨迹是否具有断链
                # 计算找的box与跳变位置的box间的航向
                dis_len = (dxj * dxj + dyj * dyj) ** 0.5
                dis_angle = 0
                if dis_len > 0.2:
                    # 转到y负轴
                    dis_angle = math.acos(dxj / dis_len) * Rads_cov - 180
                    if dyj > 0:
                        dis_angle = 90 - dis_angle
                    if dyj <= 0:
                        dis_angle = 90 - (360 - dis_angle)
                    dis_angle = (dis_angle % 360)
                if abs(dis_angle - box[4]) < 8 or abs(dis_angle + 180 - box[4]) < 8:  # 若航向角相差不大，说明在一条航线上，并在行驶方向上
                    print("统计计算距离")
                    dit_distance_id[ID_xiu] = dj
        if continue_seach:
            dit_distance_id.clear()
            dit_iou_id.clear()
            print("结束修正,继续查找！", m)
            break
        n = n + 1
        max_iou = 0  # 找到最大iou对应的id
        for iou_id in dit_iou_id.keys():
            if dit_iou_id[iou_id] > max_iou:
                max_iou = dit_iou_id[iou_id]
        for iou_id in dit_iou_id.keys():
            if dit_iou_id[iou_id] == max_iou:
                for id in range(len(all_trk_ID_det[iou_id])):  # 遍历轨迹，将ID修正过来
                    trk = copy.deepcopy(all_trk_ID_det[iou_id][id])
                    trk[9] = ID_ori
                    if ID_ori not in all_trk_fram[trk[13]]:
                        all_trk_fram[trk[13]].append(ID_ori)
                    all_trk_ID_det[ID_ori].append(trk)
                    all_trk_ID_det[iou_id][id][13] = -1
                    print("iou断链修正")
                continue_seach = True
                break
        if continue_seach:
            dit_distance_id.clear()
            dit_iou_id.clear()
            break
        if not continue_seach:
            mind = 500
            for did in dit_distance_id.keys():
                if dit_distance_id[did] < mind:
                    mind = dit_distance_id[did]
            for did in dit_distance_id.keys():
                if dit_distance_id[did] == mind:
                    for id in range(len(all_trk_ID_det[did])):  # 遍历轨迹，将ID修正过来
                        trk = copy.deepcopy(all_trk_ID_det[did][id])
                        trk[9] = ID_ori
                        if ID_ori not in all_trk_fram[trk[13]]:
                            all_trk_fram[trk[13]].append(ID_ori)
                        all_trk_ID_det[ID_ori].append(trk)
                        all_trk_ID_det[did][id][13] = -1
                        print("距离断链修正")
                    continue_seach = True
                    break
        if continue_seach:
            dit_distance_id.clear()
            dit_iou_id.clear()
            print("结束修正,继续查找！", m)
            break
    return all_trk_ID_det, tracker_news, all_trk_fram, sigel_trk, True

# def No_ID_Changen_ew1(all_trk_ID_det, tracker_news):
#     for i in range(len(tracker_news)):
#         for j in range(len(tracker_news[i])):
#             if all_trk_ID_det[tracker_news[i][j][9]][len(all_trk_ID_det[tracker_news[i][j][9]]) - 1][13] == i:    # 这个目标是该轨迹的最后一个位置
#                 ID = tracker_news[i][j][9]
#                 frame = i
#                 dx = all_trk_ID_det[ID][len(all_trk_ID_det[ID]) - 1][0]
#                 dy = all_trk_ID_det[ID][len(all_trk_ID_det[ID]) - 1][1]
#                 d = math.sqrt(dx * dx + dy * dy)
#                 if d < 70:
#                     while True:
#                         dit_distance_id = {}  # 临时存储找到的id，用于对比那个id更合适
#                         dit_iou_id = {}
#                         frame = frame + 1
#                         for j in range(len(tracker_news[frame])):
#                             IDJ = tracker_news[frame][j][9]
#                             if all_trk_ID_det[IDJ][0][13] == frame:
#                                 box_1 = all_trk_ID_det[ID][len(all_trk_ID_det[ID]) - 1]
#                                 box_2 = all_trk_ID_det[IDJ][0]
#                                 iou = Iou_Two_Box(box_1, box_2)  # 计算两个box的iou
#                                 dxj = box_2[0] - box_1[0]
#                                 dyj = box_2[1] - box_1[1]
#                                 dj = math.sqrt(dxj * dxj + dyj * dyj)
#                                 if iou > 0.2:
#                                     dit_iou_id[IDJ] = iou
#                                 else:
#                                     dis_len = (dxj * dxj + dyj * dyj) ** 0.5
#                                     dis_angle = 0
#                                     if dis_len > 0.6:
#                                         # 转到y负轴
#                                         dis_angle = math.acos(dxj / dis_len) * Rads_cov - 180
#                                         if dyj > 0:
#                                             dis_angle = 90 - dis_angle
#                                         if dyj <= 0:
#                                             dis_angle = 90 - (360 - dis_angle)
#                                         dis_angle = (dis_angle % 360)
#                                     if abs(dis_angle - box_1[4]) < 4:  # 若航向角相差不大，说明在一条航线上，并在行驶方向上
#                                         print("统计计算距离")
#                                         dit_distance_id[IDJ] = dj
#                         max_iou = 0  # 找到最大iou对应的id
#                         for iou_id in dit_iou_id.keys():
#                             if dit_iou_id[iou_id] > max_iou:
#                                 max_iou = dit_iou_id[iou_id]




