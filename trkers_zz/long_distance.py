import copy
import math
from compute_iou import Iou_Two_Box

#1、点少，造成聚类很大误差；2、地面点，造成聚类很大误差；3、停车，造成判断上的很大误差；
#当前帧中进行聚类，目标从当前+1帧开始搜索，当搜索到最近的ID轨迹时，赋予目标该ID和相关属性
def long_distance_yunduan_add_tingche(all_trk_ID_det, all_trk_fram, frame_boxs_ori):
    all_trk_ID = copy.deepcopy(all_trk_ID_det)
    all_trks_frames = copy.deepcopy(all_trk_fram)
    for ID_key in all_trk_ID.keys():
        print("远端驶入!", ID_key)
        x_f = all_trk_ID[ID_key][len(all_trk_ID[ID_key]) - 1][0]
        y_f = all_trk_ID[ID_key][len(all_trk_ID[ID_key]) - 1][1]
        x_d = all_trk_ID[ID_key][0][0]
        y_d = all_trk_ID[ID_key][0][1]

        distance_n = math.sqrt((x_f - x_d) * (x_f - x_d) + (y_f - y_d) * (y_f - y_d))

        # if distance_n < 15:
        #     continue
        if len(all_trk_ID[ID_key]) < 10:
            continue

        distance = math.sqrt(x_d * x_d + y_d * y_d)
        x_d1 = all_trk_ID[ID_key][1][0]
        y_d1 = all_trk_ID[ID_key][1][1]
        distance1 = math.sqrt(x_d1 * x_d1 + y_d1 * y_d1)

        x_d2 = all_trk_ID[ID_key][2][0]
        y_d2 = all_trk_ID[ID_key][2][1]
        distance2 = math.sqrt(x_d2 * x_d2 + y_d2 * y_d2)

        x_d3 = all_trk_ID[ID_key][3][0]
        y_d3 = all_trk_ID[ID_key][3][1]
        distance3 = math.sqrt(x_d3 * x_d3 + y_d3 * y_d3)

        x_d4 = all_trk_ID[ID_key][4][0]
        y_d4 = all_trk_ID[ID_key][4][1]
        distance4 = math.sqrt(x_d4 * x_d4 + y_d4 * y_d4)

        x_d5 = all_trk_ID[ID_key][5][0]
        y_d5 = all_trk_ID[ID_key][5][1]
        distance5 = math.sqrt(x_d5 * x_d5 + y_d5 * y_d5)

        if all_trk_ID[ID_key][0][7] in [0, 2, 5, 6]:
            yuanduan_yuzhi = 20  # 起始位置到原点的距离小于的阈值
            yuanduan_yuzhi_ = 5   # 起始的位置到原点距离大于阈值
            yuanduan_stop = 0.0   #判断是否停车的阈值，小于阈值停车，大于阈值行驶
        if all_trk_ID[ID_key][0][7] in [1, 3]:
            continue
            yuanduan_yuzhi = 10
            yuanduan_yuzhi_ = 5
            yuanduan_stop = 0.0
        if all_trk_ID[ID_key][0][7] in [4]:
            continue

        if distance < yuanduan_yuzhi and distance > yuanduan_yuzhi_:  # 若轨迹第一个位置的距离distance小于60，则该ID在60m外存在漏检，检测无
            if (distance5 - distance) >= 1.5:  # 判断为驶出
                continue
            if abs(distance2 - distance) > yuanduan_stop:              # 判断行驶
                frame = all_trk_ID[ID_key][0][13] - 1    # 查找前一帧点云
                if frame < 0:                            # 跳过第一帧，第一帧无上一帧漏检
                    continue
                x_be = all_trk_ID[ID_key][0][0]
                y_be = all_trk_ID[ID_key][0][1]
                x_dev = (all_trk_ID[ID_key][3][0] - all_trk_ID[ID_key][0][0]) / 3.0
                y_dev = (all_trk_ID[ID_key][3][1] - all_trk_ID[ID_key][0][1]) / 3.0
                for iter in range(50):
                    box_add = True
                    if frame - iter < 0:
                        break
                    else:
                        x_be = x_be - x_dev
                        y_be = y_be - y_dev
                        trk_curr = copy.deepcopy(all_trk_ID[ID_key][0])
                        trk_curr[0] = x_be
                        trk_curr[1] = y_be
                        trk_curr[11] = 500
                        trk_curr[13] = int(frame - iter)
                        # all_trks_frames[int(frame - iter)].insert(0, ID_key)
                        for trk_box in frame_boxs_ori[int(frame - iter)]:
                            iou = Iou_Two_Box(trk_box, trk_curr)
                            if iou > 0:
                                box_add = False
                            d_yuejie = math.sqrt(x_be * x_be + y_be * y_be)
                            if d_yuejie > 80:
                                box_add = False
                        # if int(frame - iter) - 1 < 0:
                        #     break
                        # for trk_box in frame_boxs_ori[int(frame - iter) - 1]:
                        #     iou = Iou_Two_Box(trk_box, trk_curr)
                        #     if iou > 0:
                        #         box_add = False
                        if box_add:
                            all_trks_frames[int(frame - iter)].append(ID_key)
                            all_trk_ID[ID_key].append(trk_curr)
                        if box_add == False:
                            break
    for ID_key in all_trk_ID.keys():
        print("远端驶出!", ID_key)
        x_f = all_trk_ID[ID_key][len(all_trk_ID[ID_key]) - 1][0]
        y_f = all_trk_ID[ID_key][len(all_trk_ID[ID_key]) - 1][1]
        x_d = all_trk_ID[ID_key][0][0]
        y_d = all_trk_ID[ID_key][0][1]
        distance_n = math.sqrt((x_f - x_d) * (x_f - x_d) + (y_f - y_d) * (y_f - y_d))
        if distance_n < 15:
            continue
        distance = math.sqrt(x_f * x_f + y_f * y_f)
        x_d1 = all_trk_ID[ID_key][len(all_trk_ID[ID_key]) - 1][0]
        y_d1 = all_trk_ID[ID_key][len(all_trk_ID[ID_key]) - 1][1]
        distance1 = math.sqrt(x_d1 * x_d1 + y_d1 * y_d1)

        x_d2 = all_trk_ID[ID_key][len(all_trk_ID[ID_key]) - 2][0]
        y_d2 = all_trk_ID[ID_key][len(all_trk_ID[ID_key]) - 2][1]
        distance2 = math.sqrt(x_d2 * x_d2 + y_d2 * y_d2)

        x_d3 = all_trk_ID[ID_key][len(all_trk_ID[ID_key]) - 3][0]
        y_d3 = all_trk_ID[ID_key][len(all_trk_ID[ID_key]) - 3][1]
        distance3 = math.sqrt(x_d3 * x_d3 + y_d3 * y_d3)

        x_d4 = all_trk_ID[ID_key][len(all_trk_ID[ID_key]) - 4][0]
        y_d4 = all_trk_ID[ID_key][len(all_trk_ID[ID_key]) - 4][1]
        distance4 = math.sqrt(x_d4 * x_d4 + y_d4 * y_d4)

        x_d5 = all_trk_ID[ID_key][len(all_trk_ID[ID_key]) - 5][0]
        y_d5 = all_trk_ID[ID_key][len(all_trk_ID[ID_key]) - 5][1]
        distance5 = math.sqrt(x_d5 * x_d5 + y_d5 * y_d5)

        if all_trk_ID[ID_key][0][7] in [0, 2, 5, 6]:
            yuanduan_yuzhi = 20  # 结束位置到原点的距离小于的阈值
            yuanduan_yuzhi_ = 5  # 起始的位置到原点距离大于的距离
            yuanduan_stop = 0.0  # 判断是否停车的阈值，小于阈值停车，大于阈值行驶
        if all_trk_ID[ID_key][0][7] in [1, 3]:
            continue
            yuanduan_yuzhi = 20
            yuanduan_yuzhi_ = 5
            yuanduan_stop = 0.0
        if all_trk_ID[ID_key][0][7] in [4]:
            continue
        if distance < yuanduan_yuzhi and distance > yuanduan_yuzhi_:  # 若distance小于60，则该ID在60m外存在漏检，检测无
            if distance - distance5 < 0.6:  # 驶入
                continue
            if abs(distance - distance2) > yuanduan_stop:  # 判断行驶
                frame = all_trk_ID[ID_key][len(all_trk_ID[ID_key]) - 1][13] + 1  # 查找后一帧点云
                if frame > len(frame_boxs_ori) - 1:  # 跳过最后一帧
                    continue
                x_be = all_trk_ID[ID_key][len(all_trk_ID[ID_key]) - 1][0]
                y_be = all_trk_ID[ID_key][len(all_trk_ID[ID_key]) - 1][1]
                x_dev = (all_trk_ID[ID_key][len(all_trk_ID[ID_key]) - 3][0] - all_trk_ID[ID_key][len(all_trk_ID[ID_key]) - 1][0]) / 3.0
                y_dev = (all_trk_ID[ID_key][len(all_trk_ID[ID_key]) - 3][1] - all_trk_ID[ID_key][len(all_trk_ID[ID_key]) - 1][1]) / 3.0
                for iter in range(50):
                    box_add = True
                    if frame + iter > len(frame_boxs_ori) - 1:
                        break
                    else:
                        x_be = x_be - x_dev
                        y_be = y_be - y_dev
                        trk_curr = copy.deepcopy(all_trk_ID[ID_key][0])
                        trk_curr[0] = x_be
                        trk_curr[1] = y_be
                        trk_curr[11] = 500
                        trk_curr[13] = int(frame + iter)
                        for trk_iter in range(iter):
                            for trk_box in frame_boxs_ori[int(frame + trk_iter)]:
                                iou = Iou_Two_Box(trk_box, trk_curr)
                                if iou > 0:
                                    box_add = False
                            if int(frame + trk_iter) + 1 > len(frame_boxs_ori) - 1:
                                break
                            for trk_box in frame_boxs_ori[int(frame + trk_iter) + 1]:
                                iou = Iou_Two_Box(trk_box, trk_curr)
                                if iou > 0:
                                    box_add = False
                                d_yuejie = math.sqrt(x_be * x_be + y_be * y_be)
                                if d_yuejie > 80:
                                    box_add = False
                        if box_add:
                            all_trks_frames[int(frame + iter)].append(ID_key)
                            all_trk_ID[ID_key].append(trk_curr)
                        if box_add == False:
                            break
    return all_trks_frames, all_trk_ID

#根据bmp图抠出背景，其中包含平移（25.5， 14.0）,该平移为检测的平移，这里同步
def QuChu_Bmp(tempoint, img, t_det):
    im_size=img.shape[0]
    img_corner = im_size // 2
    dist2pixel = im_size // 200
    xindex = math.ceil((tempoint[2] / 100.0 - t_det[0]) * dist2pixel) + img_corner-1
    lineindex = math.ceil((tempoint[3] / 100.0 + t_det[1]) * dist2pixel) + img_corner-1
    key_bool = False

    if (xindex >= im_size) or (lineindex >= im_size) or (xindex <= 0) or (lineindex <= 0):
        key_bool = False
    elif img[lineindex, xindex, 2] == 255:
        key_bool = False
    else:
        key_bool = True
    return key_bool

#判断添加的box是否与当前帧其他box有相交iou
def Check_500_Iou1(tracker_news):
    trks_news = []
    for i in range(len(tracker_news)):
        print("剔除相交的框！")
        list_boxs = []
        list_boxs_5_3 = []
        for j in range(len(tracker_news[i])):
            if tracker_news[i][j][11] == 500 or tracker_news[i][j][11] == 300:
                list_boxs_5_3.append(tracker_news[i][j])
            else:
                list_boxs.append(tracker_news[i][j])
        for n in list_boxs_5_3:
            iou_number = 0
            for m in list_boxs:
                print("查找相交的框！")
                iou = Iou_Two_Box(n, m)
                if iou > 0.03:
                    iou_number += 1
                    break
            if iou_number == 0:
                list_boxs.append(n)
        trks_news.append(list_boxs)
    return trks_news