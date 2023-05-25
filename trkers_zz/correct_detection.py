import copy
import math
from Curve_fiting import Huadong_Pinjunfa, Dis_det, DJ_sure2, kuang_fiting, \
    class_fitting, DZ_Fitting, Line_Angel_Fitting
from ID_Change_Xujian import No_ID_Change, No_ID_Changen_new
from Save_Wuhan_Test import Changandaxue_Test_Save_biaoqian
# from kdtree import kdtree_xujian, kdtree_xujian_new, kdtree_xujian_test
from long_distance import long_distance_yunduan_add_tingche, Check_500_Iou1

PI_rads = math.pi / 180
Rads_cov = 180.0 / math.pi

#虚检计数
global xujian_numbers
xujian_numbers = 0

global true_numbers  #真值判断计数
true_numbers = 0

global true_numbers_jidongche  #真值判断计数,机动车
true_numbers_jidongche = 0

global true_numbers_pe  #真值判断计数，行人
true_numbers_pe = 0

global true_numbers_fpe  #真值判断计数，非机动车
true_numbers_fpe = 0

global kuang_numbers  #框计数
kuang_numbers = 0

global class_numbers  #类别计数
class_numbers = 0

global save_key
save_key = 0

def sanjaioxingmianji(x, y, p1, p2):
    d1 = math.sqrt((x - p1[0]) * (x - p1[0]) + (y - p1[1]) * (y - p1[1]))
    d2 = math.sqrt((x - p2[0]) * (x - p2[0]) + (y - p2[1]) * (y - p2[1]))
    d3 = math.sqrt((p2[0] - p1[0]) * (p2[0] - p1[0]) + (p2[1] - p1[1]) * (p2[1] - p1[1]))
    d = (d1 + d2 + d3) / 2.0
    s = d * (d - d1) * (d - d2) * (d - d3)
    if s > 0:
       s = s ** 0.5
       return s, d
    else:
        return 10000, d

def CoreectionDtection_frame_x_y(all_trk_ID_copy, t_det, all_trk_fram, time_, jidongche, fperson, person, frame_boxs_ori, region_list):
    global xujian_numbers  # 虚检计数
    xujian_numbers = 0
    all_trk_ID_det = {}
    for i in all_trk_ID_copy.keys():  # 遍历字典all_trk_ID
        trk_ID__ = all_trk_ID_copy[i]  # 取得key值下的轨迹
        if len(trk_ID__) > 10:
            all_trk_ID_det[i] = Huadong_Pinjunfa(trk_ID__)
            all_trk_ID_det[i] = trk_ID__
        else:
            xujian_numbers += len(trk_ID__)
    global true_numbers  # 真值判断计数
    global kuang_numbers  # 框计数
    global class_numbers  # 类别计数
    global true_numbers_jidongche  # 真值判断计数,机动车
    global true_numbers_pe  # 真值判断计数，行人
    global true_numbers_fpe  # 真值判断计数，非机动车
    global save_key
    save_key = 0

    Trk_number = 0 #全部目标数量
    numbers_jidongche = 0 #机动车目标数量
    numbers_pe = 0 #行人目标数量
    numbers_fpe = 0 #非机动车目标数量
    #真值判断，class，位置，航向角，框的变化，是否虚检，是否漏检
    for i in all_trk_ID_det.keys(): #遍历字典all_trk_ID
        print("修正！", i)
        # all_trk_ID_det[i] = all_trk_ID_det[i] #取得key值下的轨迹
        Kuang_size_w = []  #用于框的大小修正列表
        Kuang_size_l = []  # 用于框的大小修正列表
        Kuang_size_h = []  # 用于框的大小修正列表
        X = []#位置曲线拟合坐标x
        Y = []#位置曲线拟合坐标y
        frame_plot = []#记录帧号
        angle_y = []#角度曲线拟合坐标
        class_label = []#判断该ID下类别的最多类别，用于修正类别
        # if len(all_trk_ID_det[i]) > 4:
        #     d1_x = abs(all_trk_ID_det[i][0][0] - all_trk_ID_det[i][1][0])
        #     d1_y = abs(all_trk_ID_det[i][0][1] - all_trk_ID_det[i][1][1])
        #     d2_x = abs(all_trk_ID_det[i][1][0] - all_trk_ID_det[i][2][0])
        #     d2_y = abs(all_trk_ID_det[i][1][1] - all_trk_ID_det[i][2][1])
        #     d3_x = abs(all_trk_ID_det[i][2][0] - all_trk_ID_det[i][3][0])
        #     d3_y = abs(all_trk_ID_det[i][2][1] - all_trk_ID_det[i][3][1])
        #     d4_x = abs(all_trk_ID_det[i][3][0] - all_trk_ID_det[i][4][0])
        #     d4_y = abs(all_trk_ID_det[i][3][1] - all_trk_ID_det[i][4][1])
        #     if math.sqrt(all_trk_ID_det[i][0][0] * all_trk_ID_det[i][0][0] + all_trk_ID_det[i][1][0] * all_trk_ID_det[i][1][0]) > 35:
        #         if d1_x > 1 or d1_y > 1:
        #             all_trk_ID_det[i][0][0] = (all_trk_ID_det[i][0][0] + all_trk_ID_det[i][1][0]) / 2.0
        #             all_trk_ID_det[i][0][1] = (all_trk_ID_det[i][0][1] + all_trk_ID_det[i][1][1]) / 2.0
        #         if d2_x > 1 or d2_y > 1:
        #             all_trk_ID_det[i][1][0] = (all_trk_ID_det[i][1][0] + all_trk_ID_det[i][2][0]) / 2.0
        #             all_trk_ID_det[i][1][1] = (all_trk_ID_det[i][1][1] + all_trk_ID_det[i][2][1]) / 2.0
        #         if d3_x > 1 or d3_y > 1:
        #             all_trk_ID_det[i][2][0] = (all_trk_ID_det[i][2][0] + all_trk_ID_det[i][3][0]) / 2.0
        #             all_trk_ID_det[i][2][1] = (all_trk_ID_det[i][2][1] + all_trk_ID_det[i][3][1]) / 2.0
        #         if d4_x > 1 or d4_y > 1:
        #             all_trk_ID_det[i][3][0] = (all_trk_ID_det[i][3][0] + all_trk_ID_det[i][4][0]) / 2.0
        #             all_trk_ID_det[i][3][1] = (all_trk_ID_det[i][3][1] + all_trk_ID_det[i][4][1]) / 2.0

        for ID_trk in range(len(all_trk_ID_det[i])):#将轨迹的x值和y值取出，进行曲线拟合
            # 真值计数，包含全部目标，机动车目标，行人目标，非机动车目标***************
            print("修正单个轨迹！", ID_trk)
            Trk_number += 1
            if all_trk_ID_det[i][ID_trk][7] in [0, 2, 5, 6]:
                numbers_jidongche += 1
            if all_trk_ID_det[i][ID_trk][7] in [1, 3]:
                numbers_pe += 1
            if all_trk_ID_det[i][ID_trk][7] in [4]:
                numbers_fpe += 1
            #****************************************

            X.append(all_trk_ID_det[i][ID_trk][0]) #存入x坐标
            Y.append(all_trk_ID_det[i][ID_trk][1]) #存入y坐标
            frame_plot.append(all_trk_ID_det[i][ID_trk][13]) #存入帧号
            Kuang_size_w.append(all_trk_ID_det[i][ID_trk][2])
            Kuang_size_l.append(all_trk_ID_det[i][ID_trk][3])
            Kuang_size_h.append(all_trk_ID_det[i][ID_trk][6])
            angle_y.append(all_trk_ID_det[i][ID_trk][4]) #存入角度
            class_label.append(all_trk_ID_det[i][ID_trk][7]) #存入类别

        #再一次通过轨迹修正航向角,由于框的中心点存在抖动的情况，通过相邻位置的向量进行航向角进行修正存在较大误差，采用一定间隔的位置向量
        # for curve_i in range(3):
        #    X, Y, angle_y = Curve_fiting(frame_plot, X, Y, angle_y)

        #/*
        # 角度修正，转到雷达负y轴
        #/*
        all_trk_ID_det[i], angle_y = DJ_sure2(all_trk_ID_det[i], angle_y)

        #/*
        # 修正目标框
        #/*
        #*****************************************************************************对框求一个均值
        X, Y, all_trk_ID_det[i], kuang_numbers = kuang_fiting(Kuang_size_w, Kuang_size_l, Kuang_size_h, X, Y, all_trk_ID_det[i], kuang_numbers)

        #/*
        #修正类别
        #/*
        # 计算该ID下最多的class类别
        class_label_max_, all_trk_ID_det[i], class_numbers = class_fitting(class_label, all_trk_ID_det[i], class_numbers)

        # #曲线拟合，根据帧号，x坐标和y坐标分别进行曲线拟合
        # for curve_i in range(3):
        #    X, Y, angle_y = Curve_fiting(frame_plot, X, Y, angle_y)
        # # x曲线拟合
        # z3 = np.polyfit(frame_plot, X, 20)
        # p3 = np.poly1d(z3)
        # # y曲线拟合
        # z4 = np.polyfit(frame_plot, Y, 20)
        # p4 = np.poly1d(z4)
        # # if save_key:
        # #     path1 = "./data_save/lines_pictures/" + str(int(i)) + "x.csv"
        # #     path2 = "./data_save/lines_pictures/" + str(int(i)) + "y.csv"
        # #     path3 = "./data_save/lines_pictures/" + str(int(i)) + "frame_plot.csv"
        # #     np.savetxt(path3, frame_plot)
        # #     np.savetxt(path1, X)
        # #     np.savetxt(path2, Y)
        # # 角度的曲线拟合
        # # z_angle = np.polyfit(frame_plot, angle_y, 20)
        # # p_angle = np.poly1d(z_angle)
        # # if save_key:
        # #     ##角度拟合保存查看效果
        # #     path_angle2 = "./data_save/lines_pictures/" + str(int(i)) + "angle.csv"
        # #     np.savetxt(path_angle2, angle_y)
        #
        # #/*
        # # 判断是否是虚检，若是虚检，
        # # 则直接continue，不进行修正,
        # # 当ID的类别为大车1,小车2时，
        # # 判断轨迹的首尾位移距离
        # #/*
        # if class_label_max_ in [0, 1]:
        #     distance_ID = math.sqrt(math.pow((all_trk_ID_det[i][0][0] - all_trk_ID_det[i][len(all_trk_ID_det[i]) - 1][0]), 2) + math.pow(
        #         (all_trk_ID_det[i][0][1] - all_trk_ID_det[i][len(all_trk_ID_det[i]) - 1][1]), 2))
        #     if (distance_ID < 3):#若首尾距离小于阈值,直接continue
        #         xujian_numbers += 1
        #         continue
        # #/*
        # # 当ID的类别为,自行车3,行人4时，判断轨迹的首尾位移距离
        # #/*
        # if class_label_max_ in [2, 3]:
        #     distance_ID = math.sqrt(math.pow((all_trk_ID_det[i][0][0] - all_trk_ID_det[i][len(all_trk_ID_det[i]) - 1][0]), 2) + math.pow(
        #         (all_trk_ID_det[i][0][1] - all_trk_ID_det[i][len(all_trk_ID_det[i]) - 1][1]), 2))
        #     if (distance_ID < 0.6):  # 若首尾距离小于阈值,直接continue
        #         xujian_numbers += 1
        #         continue
        # #/*
        # # 轨迹中的每个目标的置信度判断
        # #/
        # for j in range(len(all_trk_ID_det[i])):#根据曲线拟合判断轨迹目标
        #     if j == 0:
        #         continue
        #     if j == len(all_trk_ID_det[i]):
        #         break
        #     #角度异常检测
        #     # angel_mean = all_trk_ID_det[i][j][4]
        #     # angel_dev_ = abs(p_angle(frame_plot[j]) - angle_y[j])
        #     angel_dev = abs(all_trk_ID_det[i][j - 1][4] - all_trk_ID_det[i][j][4])
        #
        #     #class异常检测
        #     # class_dev = all_trk_ID_det[i][j][7] - class_label_max_
        #
        #     #位置异常检测
        #     dev_x = abs(p3(frame_plot[j]) - X[j])
        #     dev_y = abs(p4(frame_plot[j]) - Y[j])
        #     y_dev = math.sqrt(dev_x * dev_x + dev_y * dev_y)
        #     #根据类别设置不同的距离阈值
        #     y_dev_yuzhi = 0.0
        #     if all_trk_ID_det[i][j][7] in [0, 1]:#参考值54km每小时
        #         y_dev_yuzhi = 1.5
        #     if all_trk_ID_det[i][j][7] in [3]:#行人5km每小时，参考7.2km每小时
        #         y_dev_yuzhi = 0.5
        #     if all_trk_ID_det[i][j][7] in [2]:#自行车20km每小时，参考21km每小时
        #         y_dev_yuzhi = 0.5
        #     #保存位置变化较大的ID轨迹，分析拐弯处地方的问题
        #     d_start = math.sqrt(all_trk_ID_det[i][0][0] * all_trk_ID_det[i][0][0] + all_trk_ID_det[i][0][1] * all_trk_ID_det[i][0][1])#起始帧位置距离
        #     d_final = math.sqrt(all_trk_ID_det[i][len(all_trk_ID_det[i]) - 1][0] * all_trk_ID_det[i][len(all_trk_ID_det[i]) - 1][0] + all_trk_ID_det[i][len(all_trk_ID_det[i]) - 1][1] * all_trk_ID_det[i][len(all_trk_ID_det[i]) - 1][1])#结束帧位置距离
        #     d_guog = math.sqrt((all_trk_ID_det[i][j][0] - all_trk_ID_det[i][j - 1][0]) * (all_trk_ID_det[i][j][0] - all_trk_ID_det[i][j - 1][0]) + (all_trk_ID_det[i][j][1] - all_trk_ID_det[i][j - 1][1]) * (all_trk_ID_det[i][j][1] - all_trk_ID_det[i][j - 1][1]))
        #     frame_guog = all_trk_ID_det[i][j][13] - all_trk_ID_det[i][j - 1][13]
        #     if y_dev < y_dev_yuzhi + 2 and d_guog < y_dev_yuzhi * frame_guog:
        #         if all_trk_ID_det[i][j][7] in [0, 1]:
        #             if d_start > 30 or d_final > 30:
        #                 if angel_dev < 8 or angel_dev > 360 - 8:
        #                    all_trk_ID_det[i][j][10] += 1
        #                    #真值计数，包含全部目标，机动车目标，行人目标，非机动车目标
        #                    true_numbers += 1
        #                    if all_trk_ID_det[i][j][7] in [0, 1]:
        #                        true_numbers_jidongche += 1
        #         if all_trk_ID_det[i][j][7] in [2, 3]:
        #             if d_start > 3 or d_final > 3:
        #                 if angel_dev < 8 or angel_dev > 360 - 8:
        #                     all_trk_ID_det[i][j][10] += 1
        #                     # 真值计数，包含全部目标，机动车目标，行人目标，非机动车目标
        #                     true_numbers += 1
        #                     if all_trk_ID_det[i][j][7] in [3]:
        #                         true_numbers_pe += 1
        #                     if all_trk_ID_det[i][j][7] in [2]:
        #                         true_numbers_fpe += 1

    #/*
    #根据车道的大致航向角进行修正
    #/
    all_trk_ID_det = Line_Angel_Fitting(all_trk_ID_det, region_list)

    #/*
    # 去除虚检的轨迹
    # /
    # all_trk_ID_det_quxujian = copy.deepcopy(all_trk_ID_det)
    # all_trk_ID_det.clear()
    # all_trk_ID_det = No_Xujian(all_trk_ID_det_quxujian)

    #/*
    # 找寻ID跳变
    #/
    all_trk_ID_det_quxujian = copy.deepcopy(all_trk_ID_det)
    all_trk_ID_det.clear()
    all_trk_ID_det, ID_number = No_ID_Change(all_trk_ID_det_quxujian)

    #*
    # 计算帧间距离值，用于调试停车情况
    # /
    for i in all_trk_ID_det.keys():
        all_trk_ID_det[i] = Dis_det(all_trk_ID_det[i])
        # all_trk_ID_det[i] = Dis_det_qidong(all_trk_ID_det[i])               #去掉每个轨迹静止时的航向角，将开始移动的位置的航向角赋值给开头

    # #/*
    # # 给轨迹一个置信度得分
    # # /
    # for i in all_trk_ID_det.keys():
    #     Count_C_number = 0.0
    #     for j in range(len(all_trk_ID_det[i])):
    #         # # 位置转正北方向
    #         # x1, y1, z1, ag = Rotation(all_trk_ID_det[i][j][0], all_trk_ID_det[i][j][1], all_trk_ID_det[i][j][5], all_trk_ID_det[i][j][4])
    #         # all_trk_ID_det[i][j][0] = x1
    #         # all_trk_ID_det[i][j][1] = y1
    #         # all_trk_ID_det[i][j][5] = z1
    #         # all_trk_ID_det[i][j][4] = ag #角度转正北方向
    #         if all_trk_ID_det[i][j][10] == 1:#统计可信度为1的目标个数
    #             Count_C_number += 1
    #     socer = Count_C_number / len(all_trk_ID_det[i])#轨迹跟踪链的置信度得分
    #     for j in range(len(all_trk_ID_det[i])):
    #         all_trk_ID_det[i][j][10] = socer          # 将轨迹置信度赋值给10位

    #/*
    # 遍历保存置信度值较低的跟踪链，阈值为50
    #/
    # number_reliable = Save_Trajectory_(all_trk_ID_det)

#********赵伟修改部分
    # diu_frame = 0
    # all_trk_ID_det_ = copy.deepcopy(all_trk_ID_det)
    # for i in all_trk_ID_det_.keys():
    #     if all_trk_ID_det_[i][0][7] in [1, 2]:
    #         for j in range(len(all_trk_ID_det_[i]) - 1):
    #             frame_dev = all_trk_ID_det_[i][j + 1][13] - all_trk_ID_det_[i][j][13]
    #             if frame_dev > 1:
    #                 diu_frame += 1
    #                 print("该轨迹存在丢帧！", all_trk_ID_det_[i][0][9], frame_dev)
    #                 '''
    #                 如果两帧之间距离小于一定的阈值或者前后帧距离很小，说明在缓慢行驶，或者等红灯zhaowei
    #
    #                 '''
    #                 findframe = 0
    #                 Simu_X = []
    #                 Simu_Y = []
    #                 Simu_frame_plot = []
    #                 Simu_Kuang_size_w = []
    #                 Simu_Kuang_size_l = []
    #                 Simu_Kuang_size_h = []
    #                 Simu_Angle = []
    #
    #                 if j > 10:  # 保证点数大于一定的个数，拟合出的轨迹有一定的规律
    #                     maxindex = j + 10
    #                     if (j + 10) >= (len(all_trk_ID_det_[i]) - 1):
    #                         maxindex = len(all_trk_ID_det_[i]) - 1
    #
    #                     for loopin in range(j - 10, maxindex):
    #                         Simu_X.append(all_trk_ID_det_[i][loopin][0])  # 存入x坐标
    #                         Simu_Y.append(all_trk_ID_det_[i][loopin][1])  # 存入y坐标
    #                         Simu_frame_plot.append(all_trk_ID_det_[i][loopin][13])  # 存入帧号
    #                         Simu_Kuang_size_w.append(all_trk_ID_det_[i][loopin][2])
    #                         Simu_Kuang_size_l.append(all_trk_ID_det_[i][loopin][3])
    #                         Simu_Kuang_size_h.append(all_trk_ID_det_[i][loopin][6])
    #                         Simu_Angle.append(all_trk_ID_det_[i][loopin][4])  # 存入角度
    #
    #                     # 拟合轨迹
    #                     TrackLineX = np.polyfit(Simu_frame_plot, Simu_X, 20)  # 20 最高阶数
    #                     TrackLineSimuX = np.poly1d(TrackLineX)
    #
    #                     TrackLineY = np.polyfit(Simu_frame_plot, Simu_Y, 20)
    #                     TrackLineSimuY = np.poly1d(TrackLineY)
    #
    #                     TrackLineAngel = np.polyfit(Simu_frame_plot, Simu_Angle, 20)
    #                     TrackLineSimuAngel = np.poly1d(TrackLineAngel)
    #
    #                     for breakpointloop in range(int(all_trk_ID_det_[i][j][13]) + 1,
    #                                                 int(all_trk_ID_det_[i][j + 1][13])):
    #                         trk = copy.deepcopy(all_trk_ID_det_[i][j])
    #                         trk[0] = TrackLineSimuX(breakpointloop)  # x点模拟坐标，其他类似
    #                         trk[1] = TrackLineSimuY(breakpointloop)  # 拟合Y坐标
    #                         trk[4] = TrackLineSimuAngel(breakpointloop)  # 拟合航向角
    #                         trk[13] = breakpointloop
    #                         all_trk_ID_det[i].append(trk)
    #                         all_trk_fram[trk[13]].append(trk[9])
    #
    #                     # Simu_frame_plot = range(int(all_trk_ID_det_[i][j][13]) + 1,
    #                     #                             int(all_trk_ID_det_[i][j + 1][13]))
    #
    #                     # simuxdis = np.polyval(TrackLineX,Simu_frame_plot)
    #                     # simuydis = np.polyval(TrackLineY,Simu_frame_plot)
    #                     # simuangdis = np.polyval(TrackLineAngel,Simu_frame_plot)
    #                     # plt.plot(Simu_frame_plot,simuxdis,'s',label='SimuX')
    #                     # plt.plot(Simu_frame_plot,simuydis,'r',label='SimuY')
    #                     # plt.plot(Simu_frame_plot,simuangdis,'b',label=Simu_Angle)
    #                     # plt.show()
    #                 elif j < len(all_trk_ID_det_[i]) - 1:
    #                     SpaceFrame = all_trk_ID_det_[i][j + 1][13] - all_trk_ID_det_[i][j][13]
    #                     SpaceLenth = math.sqrt((all_trk_ID_det_[i][j + 1][0] - all_trk_ID_det_[i][j][0]) ** 2 + (
    #                                 all_trk_ID_det_[i][j + 1][1] - all_trk_ID_det_[i][j][1]) ** 2)
    #                     EstimateSpeed_1 = math.sqrt(
    #                         (all_trk_ID_det_[i][j + 2][0] - all_trk_ID_det_[i][j + 1][0]) ** 2 + (
    #                                     all_trk_ID_det_[i][j + 2][1] - all_trk_ID_det_[i][j + 1][1]) ** 2) / (
    #                                                   all_trk_ID_det_[i][j + 2][13] - all_trk_ID_det_[i][j + 1][13])
    #                     EstimateSpeed_2 = math.sqrt(
    #                         (all_trk_ID_det_[i][j + 1][0] - all_trk_ID_det_[i][j][0]) ** 2 + (
    #                                 all_trk_ID_det_[i][j + 1][1] - all_trk_ID_det_[i][j][1]) ** 2) / (
    #                                               all_trk_ID_det_[i][j + 1][13] - all_trk_ID_det_[i][j][13])
    #                     EstimateSpeed = (EstimateSpeed_2 + EstimateSpeed_1) / 2
    #                     SpaceAngle = abs(all_trk_ID_det_[i][j + 1][4] - all_trk_ID_det_[i][j][4])
    #                     Estimate_AngleSpeed = ((all_trk_ID_det_[i][j + 2][4] - all_trk_ID_det_[i][j + 1][4]) / (
    #                                 all_trk_ID_det_[i][j + 2][13] - all_trk_ID_det_[i][j + 1][13]) + (
    #                                                        all_trk_ID_det_[i][j + 1][4] - all_trk_ID_det_[i][j][4]) / (
    #                                                        all_trk_ID_det_[i][j + 1][13] - all_trk_ID_det_[i][j][
    #                                                    13])) / 2
    #
    #                     for spacepos in range(int(all_trk_ID_det_[i][j][13]) + 1, int(all_trk_ID_det_[i][j + 1][13])):
    #                         trk = copy.deepcopy(all_trk_ID_det_[i][j])
    #                         trk[4] = trk[4] + Estimate_AngleSpeed * (spacepos - (all_trk_ID_det_[i][j][13] + 1))
    #                         if abs(trk[4] - all_trk_ID_det_[i][j][4]) >= SpaceAngle:
    #                             spaceaver = (abs(trk[4] - all_trk_ID_det_[i][j][4]) - SpaceAngle) / (
    #                                         all_trk_ID_det_[i][j + 1][13] - spacepos)
    #                             trk[4] = trk[4] - spaceaver
    #
    #                         trk[0] = trk[0] + EstimateSpeed * math.cos((270 - trk[4]) * PI_rads) * (
    #                                 spacepos - (all_trk_ID_det_[i][j][13] + 1))
    #                         trk[1] = trk[1] + EstimateSpeed * math.sin((270 - trk[4]) * PI_rads) * (
    #                                 spacepos - (all_trk_ID_det_[i][j][13] + 1))
    #
    #                         trk[13] = spacepos
    #                         if math.sqrt((trk[0] - all_trk_ID_det_[i][j][0]) ** 2 + (
    #                                 trk[1] - all_trk_ID_det_[i][j][1]) ** 2) >= SpaceLenth:
    #                             lenthever = (math.sqrt((trk[0] - all_trk_ID_det_[i][j][0]) ** 2 + (
    #                                         trk[1] - all_trk_ID_det_[i][j][1]) ** 2) - SpaceLenth) / (
    #                                                     all_trk_ID_det_[i][j + 1][13] - spacepos)
    #                             trk[0] = trk[0] - abs(lenthever * math.cos((270 - trk[4]) * PI_rads))
    #                             trk[1] = trk[1] - abs(lenthever * math.cos((270 - trk[4]) * PI_rads))
    #                         all_trk_ID_det[i].append(trk)
    # all_trk_ID_det_.clear()
    # all_trk_ID_det_ = copy.deepcopy(all_trk_ID_det)
    # all_trk_ID_det.clear()

    # 滑动加权平滑滤波
    all_trk_ID_det_ = copy.deepcopy(all_trk_ID_det)
    all_trk_ID_det.clear()
    for i in all_trk_ID_det_.keys():
        all_trk_ID_det[i] = Huadong_Pinjunfa(all_trk_ID_det_[i])

    # all_trk_ID_det = savgol_filter_xy(all_trk_ID_det, 7, 5)

    # #统计远端修正回来多少目标
    # yuanduan_ob_num = []
    # for key in all_trk_ID_det.keys():
    #     yuanduan_ob_num_ = []
    #     number = len(all_trk_ID_det[key]) - len(all_trks_id[key])
    #     if number > 0:
    #         yuanduan_ob_num_.append(all_trk_ID_det[key][0][9])
    #         yuanduan_ob_num_.append(number)
    #         yuanduan_ob_num.append(yuanduan_ob_num_)
    # all_trks_id.clear()

    # *********************从frame提取ID，再从ID中提取trackers***************#

    tracker_news = []
    for frames in all_trk_fram.keys():
        print("将轨迹分类到对应帧中！", frames)
        tracker_new = []
        for frame_ID in all_trk_fram[frames]:
            if frame_ID in all_trk_ID_det.keys():
                for i in all_trk_ID_det[frame_ID]:
                    if i[13] == frames:
                        if i[10] >= 0:  # 标志位控制，等于1时只要检测正确的目标，等于0时只要检测错误的目标
                            tracker_new.append(i)
        tracker_news.append(tracker_new)
    # tracker_news = Check_500_Iou1(tracker_news)
    # tracker_news, all_trk_ID_det, all_trk_fram = kdtree_xujian_new(tracker_news, all_trk_ID_det, all_trk_fram)

    all_trk_ID_det, _, all_trk_fram = No_ID_Changen_new(all_trk_ID_det, tracker_news, all_trk_fram)
    # 遍历所有帧的轨迹，对点云去除地面点，查看一条轨迹中的位置是否存在丢帧情况，若存在丢帧情况，根据前后帧求一个中间的box，查看box中有无点云
    # 若存在点云，判断点云数量，点云数量大于一定阈值，算该帧为漏检目标，添加进轨迹链中
    all_trk_ID_det, all_trk_fram, diu_frame = DZ_Fitting(all_trk_ID_det, frame_boxs_ori, all_trk_fram)

    # ******************* 远端加入目标box ****************************** #
    all_trks_id = copy.deepcopy(all_trk_ID_det)
    all_trk_fram_ = copy.deepcopy(all_trk_fram)
    all_trk_fram.clear()
    all_trk_ID_det.clear()
    all_trk_fram, all_trk_ID_det = long_distance_yunduan_add_tingche(all_trks_id, all_trk_fram_, frame_boxs_ori)
    all_trk_fram_.clear()

    # # 添加框-new
    # tracker_news1 = []
    # for frames in all_trk_fram.keys():
    #     print("将轨迹分类到对应帧中！", frames)
    #     tracker_new = []
    #     for key in all_trk_ID_det.keys():
    #         for i in all_trk_ID_det[key]:
    #             if i[13] == frames:
    #                 tracker_new.append(i)
    #     tracker_news1.append(tracker_new)
    tracker_news1 = []
    for frames in all_trk_fram.keys():
        print("将轨迹分类到对应帧中！", frames)
        tracker_new = []
        for frame_ID in all_trk_fram[frames]:
            if frame_ID in all_trk_ID_det.keys():
                for i in all_trk_ID_det[frame_ID]:
                    if i[13] == frames:
                        tracker_new.append(i)
        tracker_news1.append(tracker_new)
    # tracker_news1 = kdtree_xujian(tracker_news1, all_trk_ID_det)
    tracker_news1 = Check_500_Iou1(tracker_news1)

    #******************** 武汉测试保存格式文件,验证可视化 ********************#
    # Wuhan_Test_Save_YZ(tracker_news, time_)
    #******************** 武汉测试保存格式文件,验证可视化 ********************#
    # Wuhan_Test_Save(tracker_news, time_)

    # ******************** 长安大学测试保存格式文件,验证可视化 ********************#
    Changandaxue_Test_Save_biaoqian(tracker_news, time_, t_det)
    # Changandaxue_Test_Save(tracker_news, time_)
    # return tracker_news1
    # #******************** 可视化部分 ********************#
    # Visual_Object(tracker_news)

