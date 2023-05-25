#武汉测试保存格式文件

import numpy as np
from XYZ_Lon_Lat import XYZ_To_BLH
#基站的经纬度和y轴与正北方向的夹角，顺时针
lon = 116.287
lat = 40.0517
# rotation = 160
rotation = 0

def Wuhan_Test_Save_YZ(tracker_news, time_):
    result_frames = []
    for i in range(len(tracker_news)):
        for j in range(len(tracker_news[i])):
            result_frame = []
            # 帧序号，t0，t1，厂商编号，目标ID，类型，经度，纬度，速度，航向角，长度，宽度，高度/(时间1100，厂商编号1100)
            result_frame.append(tracker_news[i][j][13])  # frame
            result_frame.append(time_[i])  # time
            result_frame.append(1100)
            result_frame.append(1100)
            result_frame.append(tracker_news[i][j][9])  # ID
            if tracker_news[i][j][7] in [1, 2]:
                result_frame.append(int(1))
            if tracker_news[i][j][7] in [3]:
                result_frame.append(int(2))
            if tracker_news[i][j][7] in [4]:
                result_frame.append(int(3))
            # lon_, lat_ = XYZ_To_BLH(lon, lat, tracker_news[i][j][0], tracker_news[i][j][1], rotation)
            # result_frame.append(lon_)
            # result_frame.append(lat_)
            result_frame.append(tracker_news[i][j][0])  # x
            result_frame.append(tracker_news[i][j][1])  # y
            result_frame.append(tracker_news[i][j][5])  # z
            result_frame.append(tracker_news[i][j][8])  # speed
            angle_to_N = tracker_news[i][j][4]
            if angle_to_N < 180:
                angle_to_N += 180
            else:
                angle_to_N = angle_to_N - 180
            result_frame.append(angle_to_N)  # angle
            result_frame.append(tracker_news[i][j][3])  # l
            result_frame.append(tracker_news[i][j][2])  # w
            result_frame.append(tracker_news[i][j][6])  # h
            result_frame_ = np.array(result_frame)
            result_frames.append(result_frame_)
            print("保存了目标ID！", result_frame[4])
    np.savetxt("./data_save/result_frames_yz.csv", result_frames, fmt="%f", delimiter=",")
    print("结果保存完成!")

def Wuhan_Test_Save(tracker_news, time_):
    result_frames = []
    for i in range(len(tracker_news)):
        for j in range(len(tracker_news[i])):
            result_frame = []
            # 帧序号，t0，t1，厂商编号，目标ID，类型，经度，纬度，速度，航向角，长度，宽度，高度/(时间1100，厂商编号1100)
            result_frame.append(tracker_news[i][j][13])  # frame
            result_frame.append(time_[i])  # time
            result_frame.append(1100)
            result_frame.append(1100)
            result_frame.append(tracker_news[i][j][9])  # ID
            if tracker_news[i][j][7] in [1, 2]:
                result_frame.append(int(1))
            if tracker_news[i][j][7] in [3]:
                result_frame.append(int(2))
            if tracker_news[i][j][7] in [4]:
                result_frame.append(int(3))
            lon_, lat_ = XYZ_To_BLH(lon, lat, tracker_news[i][j][0], tracker_news[i][j][1], rotation)
            result_frame.append(lon_)
            result_frame.append(lat_)
            result_frame.append(tracker_news[i][j][8])  # speed
            angle_to_N = tracker_news[i][j][4]
            if angle_to_N < 180:
                angle_to_N += 180
            else:
                angle_to_N = angle_to_N - 180
            result_frame.append(angle_to_N)  # angle
            result_frame.append(tracker_news[i][j][3])  # l
            result_frame.append(tracker_news[i][j][2])  # w
            result_frame.append(tracker_news[i][j][6])  # h
            result_frame_ = np.array(result_frame)
            result_frames.append(result_frame_)
            print("保存了目标ID！", result_frame[4])
    np.savetxt("./data_save/result_frames.csv", result_frames, fmt="%f", delimiter=",")
    print("结果保存完成!")
    # result_frames.clear()


def Changandaxue_Test_Save(tracker_news, time_):
    for i in range(len(tracker_news)):
        result_frames = []
        for j in range(len(tracker_news[i])):
            result_frame = []
            # 帧序号，t0，t1，厂商编号，目标ID，类型，经度，纬度，速度，航向角，长度，宽度，高度/(时间1100，厂商编号1100)
            result_frame.append(tracker_news[i][j][13])  # frame
            result_frame.append(tracker_news[i][j][7])  # class
            x = tracker_news[i][j][0] * 100
            result_frame.append(x)  # x
            y = tracker_news[i][j][1] * 100
            result_frame.append(y)  # y
            z = tracker_news[i][j][5] * 100
            result_frame.append(z)  # z
            l = tracker_news[i][j][3] * 100
            result_frame.append(l)  # l
            w = tracker_news[i][j][2] * 100
            result_frame.append(w)  # w
            h = tracker_news[i][j][6] * 100
            result_frame.append(h)  # h
            angle_to_N = tracker_news[i][j][4]
            if angle_to_N < 270:
                angle_to_N = 270 - angle_to_N
            else:
                angle_to_N = 630 - angle_to_N
            angle_to_N = angle_to_N
            result_frame.append(angle_to_N)  # angle

            # if tracker_news[i][j][7] in [1, 2]:
            #     result_frame.append(int(1))
            # if tracker_news[i][j][7] in [3]:
            #     result_frame.append(int(2))
            # if tracker_news[i][j][7] in [4]:
            #     result_frame.append(int(3))
            # result_frame.append(tracker_news[i][j][8])  # speed
            # result_frame.append(tracker_news[i][j][9])  # ID
            # result_frame.append(0.0)  # 置信度
            result_frame_ = np.array(result_frame)
            result_frames.append(result_frame_)
        path = "./data_save/20210825102646_result/" + str(int(i + 1)) + ".csv"
        print("保存了目标帧！", i)
        if i % 10 == 0:
           np.savetxt(path, result_frames, fmt="%f", delimiter=",")
        result_frames.clear()
    print("结果保存完成!")
    # result_frames.clear()


def Changandaxue_Test_Save_biaoqian(tracker_news, time_, t_det):
    for i in range(len(tracker_news)):
        result_frames = []
        for j in range(len(tracker_news[i])):
            result_frame = []
            result_frame.append(tracker_news[i][j][9])  # ID             0
            # result_frame.append(tracker_news[i][j][13])  # frame
            result_frame.append(tracker_news[i][j][7])  # class          1
            x = tracker_news[i][j][0] * 100 - t_det[0] * 100
            result_frame.append(x)  # x                                  2
            y = tracker_news[i][j][1] * 100 + t_det[1] * 100
            result_frame.append(y)  # y                                  3
            z = tracker_news[i][j][5] * 100
            result_frame.append(z)  # z                                  4
            result_frame.append(tracker_news[i][j][8] * 100)  # speed    5
            angle_to_N = tracker_news[i][j][4]
            if angle_to_N < 270:
                angle_to_N = 270 - angle_to_N
            else:
                angle_to_N = 630 - angle_to_N
            result_frame.append((angle_to_N % 360))  # angle                     6
            l = tracker_news[i][j][3] * 100
            w = tracker_news[i][j][2] * 100
            result_frame.append(l)  # l                                  7
            result_frame.append(w)  # w                                  8
            h = tracker_news[i][j][6] * 100
            result_frame.append(h)  # h                                  9
            result_frame.append(0)  # #0不遮挡                            10
            result_frame.append(1)  # 遮挡层度                            11
            result_frame.append(tracker_news[i][j][10])  # 置信度         12
            result_frame.append(0)  # 信息来源                            13
            result_frame.append(0)  # 异常属性                            14
            result_frame_ = np.array(result_frame)
            result_frames.append(result_frame_)
        path = "./data_save/save/" + str(int(i)).zfill(4) + ".csv"
        print("保存了目标帧！", i)
        np.savetxt(path, result_frames, fmt="%f", delimiter=",")
        result_frames.clear()
    print("结果保存完成!")
    # result_frames.clear()


def det_save_result(tracker, t_det, frame_number):
    tracker_news = []
    list_det = []
    for ori in tracker:
        list_det_box = []
        for ori_i in ori:
           list_det_box.append(ori_i)

        list_det.append(list_det_box)
    tracker_news.append(list_det)

    for i in range(len(tracker_news)):
        result_frames = []
        for j in range(len(tracker_news[i])):
            result_frame = []
            result_frame.append(tracker_news[i][j][9])  # ID             0
            # result_frame.append(tracker_news[i][j][13])  # frame
            result_frame.append(tracker_news[i][j][7])  # class          1
            x = tracker_news[i][j][0] * 100 - t_det[0] * 100
            result_frame.append(x)  # x                                  2
            y = tracker_news[i][j][1] * 100 + t_det[1] * 100
            result_frame.append(y)  # y                                  3
            z = tracker_news[i][j][5] * 100
            result_frame.append(z)  # z                                  4
            result_frame.append(tracker_news[i][j][8] * 100)  # speed    5
            angle_to_N = tracker_news[i][j][4]
            if angle_to_N < 270:
                angle_to_N = 270 - angle_to_N
            else:
                angle_to_N = 630 - angle_to_N
            result_frame.append((angle_to_N % 360))  # angle                     6
            l = tracker_news[i][j][3] * 100
            w = tracker_news[i][j][2] * 100
            result_frame.append(w)  # l                                  7
            result_frame.append(l)  # w                                  8
            h = tracker_news[i][j][6] * 100
            result_frame.append(h)  # h                                  9
            result_frame.append(0)  # #0不遮挡                            10
            result_frame.append(1)  # 遮挡层度                            11
            result_frame.append(tracker_news[i][j][10])  # 置信度         12
            result_frame.append(0)  # 信息来源                            13
            result_frame.append(0)  # 异常属性                            14
            result_frame_ = np.array(result_frame)
            result_frames.append(result_frame_)
        path = "./data_save/det_save_result/" + str(int(frame_number)) + ".csv"
        print("保存了目标帧！", frame_number)
        np.savetxt(path, result_frames, fmt="%f", delimiter=",")
        result_frames.clear()
    print("结果保存完成!")