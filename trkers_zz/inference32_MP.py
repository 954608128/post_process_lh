# coding:utf-8
import sys
from queue import Queue
# from multiprocessing import Process
# from multiprocessing import Queue
import torch.multiprocessing as mp
import numpy as np
import os
import math
from correct_detection import CoreectionDtection_frame_x_y
from long_distance import QuChu_Bmp
from new_sort_1_7 import Sort1
from PIL import Image
import copy
from real_time_function import cache_bmp

Rads_cov = 180 / math.pi
PI_rads = math.pi / 180
all_trackers = {}#所有历史帧及当前帧的轨迹，根据ID号进行存储，每个ID号对应一个目标的轨迹链
all_trackers_fram = {}#保存所有帧的对应的ID
all_trackers_ID = {}#保存所有ID对应的位置
all_trackers_ID_fram = {}#保存所有ID对应的位置

global frame_number
frame_number = 0  #帧的序号

pcap_fram_data = []
frame_boxs_ori = []

global key_fram
key_fram = 0

frames_trk = []                  #全部轨迹
frames_point = []                #全部点云
Time_stamp = []                  #存入时间戳
# # * 海淀路口
# region_list = [[[22.052, 16.91, -3], [22.2129, 2.78, -3], [97.45, 17.02, -3], [96.6, 2.39, -3]],       # 右边上
#                [[-48.99, 1.75, -3],  [-51.2, -17.3, -3], [-21.99, 2.2, -3], [-19.45, -18.83, -3]],    # 左边下
#                [[-13.677, -31.6, -3], [-17.24, -76.63, -3], [11.2, -33.97, -3], [9.52, -79.39, -3]]]  # 下边
# t_det = [25.5, 14.0]             #检测结果的平移

# # # * 8号门
# region_list = [[[25.2, -9.7, -3], [23.8, -21.78, -3], [70.91, -12.42, -3], [70, -23, -3]],       # 右边上
#                [[-63.93, -19.2, -3],  [-58.2, -34.3, -3], [-13.99, -19.9, -3], [-14.45, -31, -3]],    # 左边下
#                [[0, 0, -3], [0, 0, -3], [0, 0, -3], [0, 0, -3]]]  # 下边
# t_det = [0, 0]             #检测结果的平移

det_news = []

#*/
# 读取路径下的检测结果
# */
def Reader_Detection_Result(path, Sort_data, Time_data, file_numbers,t_det, region_list):
    path_list = os.listdir(path)
    path_list.sort(key=lambda x: int(x.split('.')[0]))
    #######读入bmp图#############
    # bmp_path = r'crop.bmp'
    # 海淀bmp图
    bmp_path = r'5lidar_BMP/sz-crop.bmp'

    # 8haomen bmp图
    # bmp_path = r'crop.bmp'

    img = Image.open(bmp_path)
    img = np.array(img)
    img = cache_bmp(img)
    for i in range(len(path_list)):
        load_path = path + path_list[i]
        print("读取检测结果路径：", load_path)
        de_result = np.loadtxt(load_path, dtype=float, delimiter=",")
        list_frame = []

        for j in range(len(de_result)):
            list_re = []
            #对检测范围进行限制
            if abs(de_result[j][2]) > (80 + 25.5) * 100 or abs(de_result[j][3]) > (75 - 14.0) * 100:    ##haidian
            # if abs(de_result[j][2]) > 80 * 100 or abs(de_result[j][3]) > 75 * 100:
                continue
            #对检测范围进行限制，根据bmp
            key_bmp = QuChu_Bmp(de_result[j], img, t_det)
            if not key_bmp:
                continue
            x_re = de_result[j][2] / 100.0
            y_re = de_result[j][3] / 100.0
            z_re = de_result[j][4] / 100.0
            l_re = de_result[j][7] / 100.0
            w_re = de_result[j][8] / 100.0
            h_re = de_result[j][9] / 100.0
            #voxel输出角度为x正轴逆时针旋转，这里转到y负轴顺时针
            # angel_re = de_result[j][6] * Rads_cov
            angel_re = de_result[j][6]
            if angel_re < 270:
                angel_re = 270 - angel_re
            else:
                angel_re = 630 - angel_re
            label_re = de_result[j][1]

            list_re.append(x_re)         # 0
            list_re.append(y_re)         # 1
            list_re.append(z_re)         # 2
            list_re.append(l_re)         # 3
            list_re.append(w_re)         # 4
            list_re.append(h_re)         # 5
            list_re.append(angel_re)     # 6
            list_re.append(label_re)     # 7

            #将一帧中的每个box添加进列表
            list_re_2_array = np.array(list_re)
            list_frame.append(list_re_2_array)
            list_re.clear()


        # for j in range(len(de_result)):
        #     list_re = []
        #     #对检测范围进行限制
        #     if abs(de_result[j][0][0]) > 80 or abs(de_result[j][0][1]) > 75:
        #         continue
        #     #对检测范围进行限制，根据bmp
        #     key_bmp = QuChu_Bmp(de_result[j][0], img, t_det)
        #     if not key_bmp:
        #         continue
        #     x_re = de_result[j][0][0]
        #     y_re = de_result[j][0][1]
        #     z_re = de_result[j][0][2]
        #     l_re = de_result[j][0][3]
        #     w_re = de_result[j][0][4]
        #     h_re = de_result[j][0][5]
        #     #voxel输出角度为x正轴逆时针旋转，这里转到y负轴顺时针
        #     angel_re = de_result[j][0][6] * Rads_cov
        #     if angel_re < 270:
        #         angel_re = 270 - angel_re
        #     else:
        #         angel_re = 630 - angel_re
        #     label_re = de_result[j][1]
        #
        #     list_re.append(x_re)         # 0
        #     list_re.append(y_re)         # 1
        #     list_re.append(z_re)         # 2
        #     list_re.append(l_re)         # 3
        #     list_re.append(w_re)         # 4
        #     list_re.append(h_re)         # 5
        #     list_re.append(angel_re)     # 6
        #     list_re.append(label_re)     # 7
        #
        #     #将一帧中的每个box添加进列表
        #     list_re_2_array = np.array(list_re)
        #     list_frame.append(list_re_2_array)
        #     list_re.clear()
        #将一帧添加进队列
        list_frame_ = np.array(list_frame)
        list_frame.clear()
        boxes_for_sort_ = list_frame_[:, [0, 1, 4, 3, 6, 2, 5, 7]]
        # boxes_for_sort = np.array(list_frame)
        # boxes_for_sort = np.array(boxes_for_sort_)
        Sort_data.put(boxes_for_sort_)
        Time_data.put(0.1)
        file_numbers.put(int(len(path_list)))
        # #点云转正北方向
        # for point_i in range(len(in_points)):
            # x1, y1, z1 = Rotation_(in_points[point_i][0],  in_points[point_i][1],  in_points[point_i][2])
            # in_points[point_i][0] = x1
            # in_points[point_i][1] = y1
            # in_points[point_i][2] = z1
            #顺时针转
            # in_points[point_i][0] = point_x[0] * math.cos(yaw) + point_x[1] * math.sin(yaw)
            # in_points[point_i][1] = point_x[1] * math.cos(yaw) - point_x[0] * math.sin(yaw)
            # mer_move_x = move_x * math.cos(RadAngle) + move_y * math.sin(RadAngle) + mer_x
            # mer_move_y = move_y * math.cos(RadAngle) - move_x * math.sin(RadAngle) + mer_y
            #逆时针转
            # in_points[point_i][0] = point_x[0] * math.cos(yaw) - point_x[1] * math.sin(yaw)
            # in_points[point_i][1] = point_x[1] * math.sin(yaw) + point_x[0] * math.cos(yaw)

def Reader_Detection_Result_SZ(path, Sort_data, Time_data, file_numbers,t_det, region_list):
    path_list = os.listdir(path)
    path_list.sort(key=lambda x: int(x.split('.')[0]))
    #######读入bmp图#############
    # bmp_path = r'crop.bmp'
    # 海淀bmp图
    bmp_path = r'5lidar_BMP/sz-crop.bmp'

    # 8haomen bmp图
    # bmp_path = r'crop.bmp'

    img = Image.open(bmp_path)
    img = np.array(img)
    img = cache_bmp(img)
    for i in range(len(path_list)):
        load_path = path + path_list[i]
        print("读取检测结果路径：", load_path)
        de_result = np.loadtxt(load_path, dtype=float, delimiter=",")
        list_frame = []

        for j in range(len(de_result)):
            list_re = []
            #对检测范围进行限制
            if abs(de_result[j][2]) > (100) or abs(de_result[j][3]) > (100) :    ##haidian
            # if abs(de_result[j][2]) > 80 * 100 or abs(de_result[j][3]) > 75 * 100:
                continue
            #对检测范围进行限制，根据bmp
            key_bmp = QuChu_Bmp(de_result[j], img, t_det)
            if not key_bmp:
                continue
            x_re = de_result[j][0] 
            y_re = de_result[j][1]
            z_re = de_result[j][2]
            l_re = de_result[j][4]
            w_re = de_result[j][3]
            h_re = de_result[j][5]
            #voxel输出角度为x正轴逆时针旋转，这里转到y负轴顺时针
            # angel_re = de_result[j][6] * Rads_cov
            angel_re = 3/2*math.pi - (de_result[j][6] * 180 / math.pi)
            
            label_re = de_result[j][7]

            list_re.append(x_re)         # 0
            list_re.append(y_re)         # 1
            list_re.append(z_re)         # 2
            list_re.append(l_re)         # 3
            list_re.append(w_re)         # 4
            list_re.append(h_re)         # 5
            list_re.append(angel_re)     # 6
            list_re.append(label_re)     # 7

            #将一帧中的每个box添加进列表
            list_re_2_array = np.array(list_re)
            list_frame.append(list_re_2_array)
            list_re.clear()


        # for j in range(len(de_result)):
        #     list_re = []
        #     #对检测范围进行限制
        #     if abs(de_result[j][0][0]) > 80 or abs(de_result[j][0][1]) > 75:
        #         continue
        #     #对检测范围进行限制，根据bmp
        #     key_bmp = QuChu_Bmp(de_result[j][0], img, t_det)
        #     if not key_bmp:
        #         continue
        #     x_re = de_result[j][0][0]
        #     y_re = de_result[j][0][1]
        #     z_re = de_result[j][0][2]
        #     l_re = de_result[j][0][3]
        #     w_re = de_result[j][0][4]
        #     h_re = de_result[j][0][5]
        #     #voxel输出角度为x正轴逆时针旋转，这里转到y负轴顺时针
        #     angel_re = de_result[j][0][6] * Rads_cov
        #     if angel_re < 270:
        #         angel_re = 270 - angel_re
        #     else:
        #         angel_re = 630 - angel_re
        #     label_re = de_result[j][1]
        #
        #     list_re.append(x_re)         # 0
        #     list_re.append(y_re)         # 1
        #     list_re.append(z_re)         # 2
        #     list_re.append(l_re)         # 3
        #     list_re.append(w_re)         # 4
        #     list_re.append(h_re)         # 5
        #     list_re.append(angel_re)     # 6
        #     list_re.append(label_re)     # 7
        #
        #     #将一帧中的每个box添加进列表
        #     list_re_2_array = np.array(list_re)
        #     list_frame.append(list_re_2_array)
        #     list_re.clear()
        #将一帧添加进队列
        list_frame_ = np.array(list_frame)
        list_frame.clear()
        boxes_for_sort_ = list_frame_[:, [0, 1, 4, 3, 6, 2, 5, 7]]
        # boxes_for_sort = np.array(list_frame)
        # boxes_for_sort = np.array(boxes_for_sort_)
        Sort_data.put(boxes_for_sort_)
        Time_data.put(0.1)
        file_numbers.put(int(len(path_list)))
        # #点云转正北方向
        # for point_i in range(len(in_points)):
            # x1, y1, z1 = Rotation_(in_points[point_i][0],  in_points[point_i][1],  in_points[point_i][2])
            # in_points[point_i][0] = x1
            # in_points[point_i][1] = y1
            # in_points[point_i][2] = z1
            #顺时针转
            # in_points[point_i][0] = point_x[0] * math.cos(yaw) + point_x[1] * math.sin(yaw)
            # in_points[point_i][1] = point_x[1] * math.cos(yaw) - point_x[0] * math.sin(yaw)
            # mer_move_x = move_x * math.cos(RadAngle) + move_y * math.sin(RadAngle) + mer_x
            # mer_move_y = move_y * math.cos(RadAngle) - move_x * math.sin(RadAngle) + mer_y
            #逆时针转
            # in_points[point_i][0] = point_x[0] * math.cos(yaw) - point_x[1] * math.sin(yaw)
            # in_points[point_i][1] = point_x[1] * math.sin(yaw) + point_x[0] * math.cos(yaw)

def Reader_Detection_Result_csv(path, Sort_data, Time_data, file_numbers):
    path_list = os.listdir(path)
    path_list.sort(key=lambda x: int(x.split('.')[0]))
    #######读入bmp图#############
    # bmp_path = r'crop.bmp'
    bmp_path = r'5lidar_BMP/crop_haidian-new.bmp'
    # bmp_path = r'5lidar_BMP/crop.bmp'
    img = Image.open(bmp_path)
    img = np.array(img)
    img = cache_bmp(img)
    for i in range(len(path_list)):
        load_path = path + path_list[i]
        print("读取检测结果路径：", load_path)
        with open(load_path, encoding='utf-8') as f:
            de_result = np.loadtxt(f, delimiter=",")

        # de_result = np.load(load_path, allow_pickle=True)
        list_frame = []
        for j in range(len(de_result)):
            list_re = []
            #对检测范围进行限制
            # if abs(de_result[j][2]) > 80 or abs(de_result[j][3]) > 75:
            #     continue
            #对检测范围进行限制，根据bmp
            # key_bmp = QuChu_Bmp(de_result[j][0], img, t_det)
            # if not key_bmp:
            #     continue
            x_re = de_result[j][2] / 100
            y_re = de_result[j][3] / 100
            z_re = de_result[j][4] / 100
            l_re = de_result[j][7] / 100
            w_re = de_result[j][8] / 100
            h_re = de_result[j][9] / 100
            #voxel输出角度为x正轴逆时针旋转，这里转到y负轴顺时针
            angel_re = de_result[j][6]
            # if angel_re < 270:
            #     angel_re = 270 - angel_re
            # else:
            #     angel_re = 630 - angel_re
            label_re = int(de_result[j][1])
            if label_re == 0:
                label_re = 1
            elif label_re == 2 or label_re == 6 or label_re == 5:
                label_re = 0
            elif label_re == 4:
                label_re = 3
            else:
                label_re = 2
            list_re.append(x_re)
            list_re.append(y_re)
            list_re.append(z_re)
            list_re.append(l_re)
            list_re.append(w_re)
            list_re.append(h_re)
            list_re.append(angel_re)
            list_re.append(label_re)

            list_re.append(de_result[j][5])
            list_re.append(de_result[j][0])
            list_re.append(0)
            list_re.append(0)
            list_re.append(0)

            #将一帧中的每个box添加进列表
            list_re_2_array = np.array(list_re)
            list_frame.append(list_re_2_array)
            list_re.clear()
        #将一帧添加进队列
        list_frame_ = np.array(list_frame)
        list_frame.clear()
        boxes_for_sort_ = list_frame_[:, [0, 1, 3, 4, 6, 2, 5, 7, 8, 9, 10, 11, 12]]
        # boxes_for_sort = np.array(list_frame)
        # boxes_for_sort = np.array(boxes_for_sort_)
        Sort_data.put(boxes_for_sort_)
        Time_data.put(0.1)
        file_numbers.put(int(len(path_list)))

#/*
#全局修正轨迹
# /*
def detection_h_l(trackers, frame_number, Time_data, jidongche, fperson, person, frames, boxes_for_sort, t_det, region_list):
    frames_trk.append(trackers)#存全部轨迹tracker
    Time_stamp.append(float(Time_data.get()))
    global key_fram
    trackers = np.concatenate((trackers, frame_number * np.ones((len(trackers), 1))), axis=1)
    frame_boxs_ori.append(trackers)
    # det_news.append(boxes_for_sort)
    for i in range(len(trackers)):#字典，每个frame下的所有ID
        if frame_number in all_trackers_fram:
            all_trackers_fram[frame_number].append(trackers[i][9])
        if frame_number not in all_trackers_fram:
            all_trackers_fram[frame_number] = []
            all_trackers_fram[frame_number].append(trackers[i][9])

        #每个ID下的所有trackers
        if trackers[i][9] in all_trackers_ID:  # 判断ID=trackers[i][9]为键值是否在all_trackers（历史轨迹链中）,如果在历史轨迹链中,加入到轨迹链中
            all_trackers_ID[trackers[i][9]].append(trackers[i])
        if trackers[i][9] not in all_trackers_ID:  # 判断ID=trackers[i][9]为键值是否在all_trackers（历史轨迹链中）,如果不在历史轨迹链中,创建新的轨迹链，加入到轨迹链中
            all_trackers_ID[trackers[i][9]] = []
            all_trackers_ID[trackers[i][9]].append(trackers[i])
    print("prosess:读取数据，跟踪预测，分类!", key_fram, frame_number)
    if key_fram >= frames - 1:
        all_trackers_ID_copy = copy.deepcopy(all_trackers_ID)
        all_trackers_ID.clear()
        all_trackers_fram_copy = copy.deepcopy(all_trackers_fram)
        all_trackers_fram.clear()
        frame_boxs_ori_copy = copy.deepcopy(frame_boxs_ori)
        frame_boxs_ori.clear()
        CoreectionDtection_frame_x_y(all_trackers_ID_copy, t_det, all_trackers_fram_copy, Time_stamp, jidongche, fperson, person, frame_boxs_ori_copy, region_list)
        print("*************** all done! **************************")
        sys.exit(0)
    key_fram += 1

#/*
#目标全轨迹修正
#/*
def sort_result(Sort_data, Time_data, file_numbers, t_det, region_list):
    global frame_number
    frame_number = 0
    mot_tracker = Sort1()#修改后的跟踪与预测
    # mot_tracker = Sort()#之前的跟踪和预测
    jidongche = 0
    person = 0
    fperson = 0
    Sort_data_ori = []
    while True:
        if Sort_data.empty():
            continue
        boxes_for_sort = Sort_data.get()
        # boxes_for_sort1_ = copy.deepcopy(boxes_for_sort1)
        #统计检测目标个数
        for i in range(len(boxes_for_sort)):
            if boxes_for_sort[i][7] in [0, 2, 5, 6]:
                jidongche += 1
            if boxes_for_sort[i][7] in [1, 3]:
                fperson += 1
            if boxes_for_sort[i][7] in [4]:
                person += 1
        # in_points = Filter_Ground(in_points)  # 过滤地面点，因为在检测之前加入了地面点滤除，这里不打开地面点滤除
        #无轨迹修正和角度修正
        # trackers, speeds, matched, unmatched_dets, unmatched_trks, dets, trks = mot_tracker.update(boxes_for_sort)  # 将所有的跟踪到的物体都使用其进行跟踪,状态量x,y,宽,长,角度,z,高度
        #进行轨迹修正和角度修正
        trackers = mot_tracker.update(boxes_for_sort, frame_number)
        # import pdb;pdb.set_trace()
        # 轨迹异常检测
        frames = file_numbers.get()
        detection_h_l(trackers, frame_number, Time_data, jidongche, fperson, person, frames, boxes_for_sort, t_det, region_list)
        frame_number += 1


if __name__=='__main__':
    mp.set_start_method('spawn', force=True)
    # save_dir = r'/home/wanji/Documents/wjv4/second.pytorch/save_result/'
    #海淀路口
    # filename_pcd = r'/media/wanji/data1/12-7-3/bmp/pcd-bmp/'
    # path = r'/media/wanji/NO.5/张涵/haidian_frame/20211126124617/detect_csv/'
    path = r'D:/post_process/trkers_zz/data/20230414072511_0/'
    # # 8 号门
    # filename_pcd = r'/media/wanji/data1/8号门1-20/20211216091542/pcd/'
    # path = r'/media/wanji/data1/8号门1-20/trk_npy/'
    region_list = []
    t_det = []
    #### 读取初始化部分 ##############
    # load_path1 = 'config/config_jx.txt'  # 区域划分配置
    load_path2 = 'config/config_t.txt'  # 位移划分配置
    load_path1 = 'config/config_jx.txt'  # 区域划分配置
    load_path2 = 'config/config_t.txt'


    t_list = np.loadtxt(load_path2)
    t_det.append(t_list[0])
    t_det.append(t_list[1])

    de_result = np.loadtxt(load_path1)
    point = []
    region = []
    for i in range(len(de_result)):
        if (i + 1) % 3 != 0:
            point.append((de_result[i] - 25.5)) # haidian
            # point.append((de_result[i]))
        if (i + 1) % 3 == 0:
            point.append((de_result[i] + 14.0))   # haidian
            # point.append((de_result[i]))
            point1 = copy.deepcopy(point)
            region.append(point1)
            point = []
        if (i + 1) % 12 == 0:
            region1 = copy.deepcopy(region)
            region_list.append(region1)
            region = []

    Point_data = Queue()
    Time_data = Queue()
    Sort_data = Queue()
    file_numbers = Queue()

    # Point_data = []
    # Time_data = []
    # Sort_data = []
    # file_numbers = []
    # 禁用多线程
    # second = Process(target=Reader_Detection_Result, args=(path, Sort_data, Time_data, file_numbers, t_det, region_list))
    # sort_view = Process(target=sort_result, args=(Sort_data, Time_data, file_numbers, t_det, region_list))

    second = Reader_Detection_Result_SZ(path, Sort_data, Time_data, file_numbers, t_det, region_list)
    sort_view = sort_result(Sort_data, Time_data, file_numbers, t_det, region_list)

    second.start()
    sort_view.start()

    second.join()
    sort_view.join()
