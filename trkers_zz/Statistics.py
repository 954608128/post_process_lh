import numpy as np
from Basic_Information import Basic_information_fusion
from RealTarget import RealTargetInfo
import compute_iou
import math

class_num = 4           # 类别数
iou_Threshold = 0.3     # iou阈值
dis_Threshold = [2.0, 2.0, 1.5, 1.5]     # 距离阈值,大车和小车为2米,非机动车和行人为1.5米
man_area_coord = [[-35, -17, -14, -6],
                  [-48, -38, 4, 28],
                  [-34, -18, 36, 43],
                  [-17, -7, 0, 31]]

class Statistics:
    """统计跟踪算法测试项之类"""
    def __init__(self, area_coord, b_area):
        # 字典
        self.track_save = {}                                # 储存轨迹用来画
        self.track_save_test = {}
        self.start_position = {}                            # 起始位置统计
        self.end_position = {}                              # 终止位置统计
        self.start_zone = {}                                # 起始区域
        self.end_zone = {}                                  # 终止区域
        self.track_long = {}                                # 跟踪链长度
        self.track_long_test = {}
        self.class_stat = {}                                # 该ID出现类别次数
        self.class_stat_test = {}
        self.final_class_stat1 = {}                         # 记录该ID的大类类别, 0-非机动车，1-机动车，2-行人
        self.final_class_stat2 = {}                         # 记录该ID的小类类别
        self.frame_stat = {}                                # 记录该ID出现的帧数
        self.frame_stat_test = {}
        self.frame_detec_prop = {}                          # 记录该ID的每秒检出率
        self.road_dis_in = [{} for _ in range(21)]          # 记录不同道路起止据测试区距离
        self.road_dis_out = [{} for _ in range(21)]         # 记录不同道路起止据测试区距离
        self.class_wrong_frame = {}                         # 类别错误详细帧
        self.class_wrong_type = {}                          # 类别错误的详细类别
        self.zone_point = {}                                # 记录区域内每个ID的坐标
        self.zone_track = {}                                # 记录区域内每个ID的轨迹
        self.zone_start = {}                                # 区域内开始点坐标
        self.zone_end = {}                                  # 区域内结束点坐标
        self.real_track_frame = {}                          # 真值目标ID及对应帧,by zb
        self.part_mismatch = {}                             # 某真值目标中部分帧未匹配
        self.id_only_type_jump = {}                         # ID全程不变但类型有跳变,key为真值目标ID,ualue
        self.id_jump = {}                                   # ID跳变
        self.real_type = {}
        self.frame_test_id_type = {}                        # 每帧各待测目标的ID及类别
        self.class_fake = {}
        self.class_fake_class = {}                          # 虚检目标的类别#
        # 列表
        self.car_in = [[], [], []]                          # 驶入车辆
        self.car_out = [[], [], []]                         # 驶出车辆
        self.broken_id = [[], [], []]                       # 断链ID
        self.effective_id = []
        self.broken_stat = []                               # 断链详情
        self.class_judge = [[], [], []]                     # 该ID类别(统计中出现类别次数最多的默认为该ID类别)
        self.frame_prop = [[], [], []]                      # 不同类别每秒检出率
        self.drive_in_20 = []                               # 机动车满足驶入起点据测试区20m
        self.drive_out_50 = []                              # 机动车满足驶出终点据测试区50m
        self.drive_both_20_50 = []                          # 机动车满足驶入驶出条件
        self.class_wrong_stat = []                          # 类别错误统计
        self.listRealTarget = []                            # 真值中的每个目标为一个列表元素
        self.eachframetesttargetmatch = []                  # 当前待测帧某目标是否已有匹配,列表嵌套列表
        self.eachframetesttrajsave = []                     # 当前待测帧目标轨迹已存储
        self.each_frame_real_in_area = []                   # 当前帧真值目标是否在区域内
        self.each_frame_test_in_area = []                   # 当前帧待测目标是否在区域内
        self.each_frame_real_in_man_area = []               # 当前帧真值目标是否在行人区域内
        self.each_frame_test_in_man_area = []               # 当前帧待测目标是否在行人区域内
        self.list_none_match = []                           # 待测中均没能匹配的真值目标
        self.id_jump_info = []                              # ID跳变跟踪详情
        self.type_jump_info = []                            # 类型跳变详情
        self.part_mismatch_info = []                         # 部分帧为匹配详情
        self.class_wrong_info = []                          # 识别但分类错误的详情,即误检
        self.class_miss_info = []                           # 漏检详情
        self.class_fake_info = []                           # 虚检详情

        # 数组
        self.track_two_len = np.zeros(3)                    # 机动车/非机动车/行人平均跟踪链长度
        self.frame_everymin = np.zeros(3)                   # 机动车/非机动车/行人每秒检出
        self.road_in_prop = np.zeros((21, 3))               # 机动车驶入 0-25/25-50/50-～ 的数量比例
        self.road_out_prop = np.zeros((21, 3))              # 机动车驶出 0-25/25-50/50-～ 的数量比例
        self.class_right = np.zeros((3, 2))                 # 各大类类别对的帧数/总帧数
        self.final_class_right = np.zeros(3)                # 最终类别准确率
        self.zone_track_num = np.zeros(2)                   # 区域内满足轨迹长度数量/总数
        self.track_success_num = np.zeros((class_num,), dtype=np.int)  # 各类目标跟踪成功的数量
        self.track_success_num_loose = np.zeros((class_num,), dtype=np.int)  # 各类目标非严格跟踪成功的数量,匹配数达到80%即可，不考虑类型
        self.track_success_num_60 = np.zeros((class_num,), dtype=np.int)  # 各类目标跟踪成功的数量,匹配数达到60%即可，且类型唯一
        self.track_num = np.zeros((class_num,), dtype=np.int)       # 各类目标的数量
        self.track_success_rate = np.zeros(class_num)               # 各类跟踪成功率
        self.track_success_rate_loose = np.zeros(class_num)         # 不严格的跟踪成功率
        self.track_success_rate_60 = np.zeros(class_num)            # 匹配数60%的跟踪成功率
        self.track_length_real = np.zeros(class_num)                # 各类真值跟踪链长度,仅针对能跟踪成功的目标
        self.track_length_test = np.zeros(class_num)                # 各类待测跟踪链长度,仅针对能跟踪成功的目标
        self.num_recognition_right = np.zeros(class_num)            # 正确识别的样本数
        self.num_recognition_sort_wrong = np.zeros(class_num)       # 识别到但分类错误的样本数
        self.num_wrong_in_man_area = np.zeros((1,), dtype=np.int)                    # 在行人区域内误检的样本数
        self.num_miss_in_man_area = np.zeros((1,), dtype=np.int)                     # 在行人区域内漏检的样本数
        self.num_fake_in_man_area = np.zeros((1,), dtype=np.int)                     # 在行人区域内虚检的样本数
        self.num_real_in_man_area = np.zeros((1,), dtype=np.int)                     # 在行人区域内真值样本数
        self.num_test_in_man_area = np.zeros((1,), dtype=np.int)                     # 在行人区域内待测样本数
        self.num_right_in_man_area = np.zeros((1,), dtype=np.int)                    # 行人区域内正确识别的样本数
        self.test_right_not_in_man_area = np.zeros((1,), dtype=np.int)               # 真值在行人区域内但待测不在区域内的样本数
        self.num_recognition_right_Veh = np.zeros((1,), dtype=np.int)            # 机动车识别准确数量
        self.num_miss = np.zeros(class_num)                         # 漏检的样本数
        # self.num_wrong = np.zeros(class_num)                        # 误检的样本数,与num_recognition_sort_wrong一样
        self.num_fake = np.zeros(class_num)                         # 虚检的样本数
        self.rate_recognition_right = np.zeros(class_num)           # 识别准确率
        self.rate_sort_right = np.zeros(class_num)                  # 分类准确率
        self.rate_miss = np.zeros(class_num)                        # 漏检率
        self.rate_wrong = np.zeros(class_num)                       # 误检率
        self.rate_fake = np.zeros(class_num)                        # 虚检率
        self.rate_recognition_right_Veh = np.zeros(1)               # 机动车分类准确率
        self.num_real_target = np.zeros(class_num)                  # 真值目标样本数
        self.num_test_target = np.zeros(class_num)                  # 检测样本数
        # 阈值
        self.axis_limit_car = area_coord                      # 测试区域范围
        self.area_coord_all = area_coord
        # 变量
        self.num_real_target_all = 0
        self.num_test_target_all = 0
        self.num_recognition_right_all = 0
        self.num_recognition_sort_wrong_all = 0
        self.num_miss_all = 0
        # self.num_wrong_all = 0
        self.num_fake_all = 0
        self.rate_recognition_right_all = 0.0
        self.rate_sort_right_all = 0.0
        self.rate_wrong_all = 0.0
        self.rate_fake_all = 0.0
        self.rate_miss_all = 0.0
        self.rate_miss_in_man_area = 0.0
        self.rate_wrong_in_man_area = 0.0
        self.rate_fake_in_man_area = 0.0
        self.class_num = 0                                  # 统计分类的总数
        self.class_ok_num = 0                               # 分类正确的数量
        self.b_area = b_area

    def frame_data_match(self, real_data, test_data, frame_id, frame_index):
        """每帧数据进行iou匹配,以帧为单位"""
        b_in_area = False
        b_in_man_area = False
        real_len = len(real_data)
        test_len = len(test_data)
        real_box = np.zeros((real_len, 8))        # X,Y,W/2,L/2,角度,Z,H,class
        test_box = np.zeros((test_len, 8))
        for index in range(real_len):
            real_box[index][0] = real_data[index][2]            # x
            real_box[index][1] = real_data[index][3]            # y
            real_box[index][3] = real_data[index][8]        # w,长和宽互换,且不再除2,原为[2]是w/2,[3]是l/2
            real_box[index][2] = real_data[index][7]        # l
            real_box[index][4] = math.radians(real_data[index][6])  # 角度
            real_box[index][5] = real_data[index][4]            # z
            real_box[index][6] = real_data[index][9]            # h
            real_box[index][7] = real_data[index][1]            # class
        for index in range(test_len):
            test_box[index][0] = test_data[index][2]            # x
            test_box[index][1] = test_data[index][3]            # y
            test_box[index][3] = test_data[index][8]       # w
            test_box[index][2] = test_data[index][7]       # l
            test_box[index][4] = math.radians(test_data[index][6])  # 角度
            test_box[index][5] = test_data[index][4]            # z
            test_box[index][6] = test_data[index][9]            # h
            test_box[index][7] = test_data[index][1]            # class
        frame_iou = np.zeros((real_len, test_len))
        frame_iou = compute_iou.rotate_nms_cc(real_box, test_box)   # m*n的矩阵
        # 在上述矩阵中,找每一行的iou最大值,若大于阈值,则该位置对应的真值和待测目标即匹配
        iou_max = frame_iou.max(1)      # 找出每一行的iou最大值
        test_target_index = -1
        for real_idx in range(real_len):
            self.each_frame_real_in_area[frame_index].append(False)
            self.each_frame_real_in_man_area[frame_index].append(False)
        for test_idx in range(test_len):
            self.each_frame_test_in_area[frame_index].append(False)
            self.each_frame_test_in_man_area[frame_index].append(False) # 初始化 目标是否在检测区域内
        test_index = -1
        for testTarget in test_data:
            test_index += 1
            TestTarget = Basic_information_fusion(testTarget, frame_id)
            b_in_area = self.is_in_area(TestTarget)  # 当前目标是否在区域内
            self.each_frame_test_in_area[frame_index][test_index] = b_in_area
            if not self.b_area and TestTarget.class_fs == 3:    # 统计全部数据时，仅行人类型考虑是否在行人区域内，其余类型不考虑，默认为不在
                b_in_man_area = self.is_in_man_area(TestTarget)
                self.each_frame_test_in_man_area[frame_index][test_index] = b_in_man_area
                if b_in_man_area:
                    self.num_test_in_man_area += 1  # 在行人区域内待测样本数
            # 记录待测中每帧各目标的id及类别
            if frame_index not in self.frame_test_id_type.keys():
                self.frame_test_id_type.update({frame_index: [[TestTarget.id_fs, TestTarget.class_fs]]})
            else:
                self.frame_test_id_type[frame_index].append([TestTarget.id_fs, TestTarget.class_fs])
            if self.eachframetesttrajsave[frame_index][test_index] == 0:  # 当前待测目标轨迹尚未保存
                if (not self.b_area) or (self.b_area and b_in_area):  # 不含检测区域或含检测区域且在区域内  not b_area 表示不规定检测区域, 出现的目标都统计
                    self.test_track_save(TestTarget) # 待测目标轨迹保存
                    self.eachframetesttrajsave[frame_index][test_index] = 1
                    self.num_test_target[TestTarget.class_fs] += 1
        for real_idx in range(real_len):
            is_match = False
            if iou_max[real_idx] >= iou_Threshold: # 当前真值目标按照IOU匹配到待测目标
                for test_idx in range(test_len):
                    if frame_iou[real_idx][test_idx] == iou_max[real_idx]:
                        test_target_index = test_idx # 找到真值目标的待测目标索引
                        if self.eachframetesttargetmatch[frame_index][test_idx] == 0:  # 只有尚未匹配成功的目标才参与当前匹配
                            # if (not self.b_area) or (self.b_area and self.each_frame_test_in_area[frame_index][test_idx]):
                                # 发现有的区域内真值目标也有与之匹配的待测目标，但判断出待测目标不在区域内，即认为不匹配, 故去掉对能匹配的待测目标是否在区域内的判断
                            self.eachframetesttargetmatch[frame_index][test_target_index] = 1
                            is_match = True
                        break
            RealTarget = Basic_information_fusion(real_data[real_idx], frame_id)  # frame_id表示帧序号
            if not self.b_area and RealTarget.class_fs == 3:
                b_in_man_area = self.is_in_man_area(RealTarget)
                self.each_frame_real_in_man_area[frame_index][real_idx] = b_in_man_area
                if b_in_man_area:
                    self.num_real_in_man_area += 1
            b_in_area = self.is_in_area(RealTarget)
            self.each_frame_real_in_area[frame_index][real_idx] = b_in_area
            if (not self.b_area) or (self.b_area and b_in_area):
                if not is_match:
                    # 如果当前真值目标没有IOU匹配的,则在剩余未匹配的待测目标中找中心点距离小于阈值的,满足则也是匹配
                    for test_idx in range(test_len):
                        if self.eachframetesttargetmatch[frame_index][test_idx] == 0:  # 只有尚未匹配成功的目标才参与当前匹配
                            test_target = Basic_information_fusion(test_data[test_target_index], frame_id)
                            dis = np.sqrt((RealTarget.x - test_target.x)**2 + (RealTarget.y - test_target.y)**2)
                            if dis <= dis_Threshold[RealTarget.class_fs]:  # 通过距离匹配
                                self.eachframetesttargetmatch[frame_index][test_target_index] = 1
                                is_match = True     # 此时的匹配仅是在IOU和距离上做匹配，尚未判断类型
                                test_target_index = test_idx     # 此句和下句为新增加，03/18
                                break
                #  保存当前真值目标的基本信息
                self.num_real_target[RealTarget.class_fs] += 1
                self.real_track_save(RealTarget)  # 当前真值目标轨迹保存
                self.Position_stat(RealTarget)  # 记录起止点
                if RealTarget.id_fs not in self.real_track_frame.keys():
                    self.real_track_frame.update({RealTarget.id_fs: [RealTarget.frame]})
                else:
                    self.real_track_frame[RealTarget.id_fs].append(RealTarget.frame)
                if len(self.listRealTarget) == 0:  # 每个真值目标信息只保存一次(第一次)
                    self.listRealTarget.append(RealTargetInfo())
                    self.listRealTarget[-1].target_id = RealTarget.id_fs
                    self.listRealTarget[-1].type = RealTarget.class_fs
                else:
                    b_target_exist = False  # 真值目标按照ID判断,第二次出现不做处理
                    for listTarget in self.listRealTarget:
                        if listTarget.target_id == RealTarget.id_fs:
                            b_target_exist = True
                            break
                    if not b_target_exist:  # 是新的真值目标
                        self.listRealTarget.append(RealTargetInfo())
                        self.listRealTarget[-1].target_id = RealTarget.id_fs
                        self.listRealTarget[-1].type = RealTarget.class_fs
                if RealTarget.id_fs not in self.real_type.keys():
                    self.real_type.update({RealTarget.id_fs: RealTarget.class_fs})
                for listTarget in self.listRealTarget:
                    if listTarget.target_id == RealTarget.id_fs:
                        index = self.listRealTarget.index(listTarget)
                        self.listRealTarget[index].real_frame_num += 1
                        break
                if is_match:
                    #   当前真值目标有能够匹配的待测目标
                    TestTarget = Basic_information_fusion(test_data[test_target_index], frame_id)
                    self.listRealTarget[index].dict_realtestmatch.update(
                        {RealTarget.frame: [TestTarget.id_fs, TestTarget.class_fs]})
                    self.listRealTarget[index].matchNum += 1
                    # 在已匹配的目标中,判断类别,统计分类正确率
                    if RealTarget.class_fs == 0 or RealTarget.class_fs == 1:    # 统计机动车识别准确数，0变1,1变0也算识别准确
                        if TestTarget.class_fs == 0 or TestTarget.class_fs == 1:
                            self.num_recognition_right_Veh += 1
                    if RealTarget.class_fs == TestTarget.class_fs:
                        self.num_recognition_right[RealTarget.class_fs] += 1  # 准确识别的样本数
                        if not self.b_area and RealTarget.class_fs == 3:
                            if b_in_man_area:
                                self.num_right_in_man_area += 1
                                if not self.is_in_man_area(TestTarget):
                                    self.test_right_not_in_man_area += 1
                    else:
                        self.num_recognition_sort_wrong[RealTarget.class_fs] += 1  # 识别但分类错误的样本数
                        # self.num_wrong[RealTarget.class_fs] += 1
                        if not self.b_area and RealTarget.class_fs == 3:
                            if b_in_man_area:
                                self.num_wrong_in_man_area += 1     # 在行人区域内误检数
                        # 保存识别但分类错误的帧
                        if RealTarget.id_fs not in self.class_wrong_frame.keys():
                            self.class_wrong_frame.update({RealTarget.id_fs: [RealTarget.frame]})
                        else:
                            self.class_wrong_frame[RealTarget.id_fs].append(RealTarget.frame)
                        # 保存识别但分类错误的类别
                        if RealTarget.id_fs not in self.class_wrong_type.keys():
                            self.class_wrong_type.update({RealTarget.id_fs: [TestTarget.class_fs]})
                        else:
                            if TestTarget.class_fs not in self.class_wrong_type[RealTarget.id_fs]:
                                self.class_wrong_type[RealTarget.id_fs].append(TestTarget.class_fs)
                else:
                    #   当前真值目标没有能够匹配的待测目标
                    self.listRealTarget[index].dict_realtestmatch.update({RealTarget.frame: [-1, -1]})
                    # 该真值目标的当前帧未匹配(部分缺失)
                    if RealTarget.id_fs not in self.part_mismatch.keys():
                        self.part_mismatch.update({RealTarget.id_fs: [RealTarget.frame]})
                    else:
                        self.part_mismatch[RealTarget.id_fs].append(RealTarget.frame)
                    self.num_miss[RealTarget.class_fs] += 1  # 漏检的样本数
                    if not self.b_area and RealTarget.class_fs == 3:
                        if b_in_man_area:
                            self.num_miss_in_man_area += 1  # 在行人区域内漏检数
        aaa = 0

    def is_in_area(self, target):
        """判断当前目标是否在检测区域内"""
        area_coord = self.area_coord_all[target.class_fs]
        if target.class_fs == 4:
            print('Error!目标类别为4\n')
        if area_coord[0] <= target.x <= area_coord[1] and area_coord[2] <= target.y <= area_coord[3]:
            return True
        else:
            return False

    def is_in_man_area(self, target):
        """判断目标是否在斑马线的4个行人区域里"""
        if man_area_coord[0][0] <= target.x <= man_area_coord[0][1] and man_area_coord[0][2] <= target.y <= man_area_coord[0][3]:
            return True
        elif man_area_coord[1][0] <= target.x <= man_area_coord[1][1] and man_area_coord[1][2] <= target.y <= man_area_coord[1][3]:
            return True
        elif man_area_coord[2][0] <= target.x <= man_area_coord[2][1] and man_area_coord[2][2] <= target.y <= man_area_coord[2][3]:
            return True
        elif man_area_coord[3][0] <= target.x <= man_area_coord[3][1] and man_area_coord[3][2] <= target.y <= man_area_coord[3][3]:
            return True
        else:
            return False

    def real_test_match(self, realtargetdata, testdata, frameindex):
        """当前真值和当前待测帧所有目标进行匹配"""
        RealTarget = Basic_information_fusion(realtargetdata, frameindex)    # realdata[frameindex]表示帧序号
        self.num_real_target[RealTarget.class_fs] += 1
        self.real_track_save(RealTarget)  # 当前真值目标轨迹保存
        self.Position_stat(RealTarget)  # 记录起止点
        if RealTarget.id_fs not in self.real_track_frame.keys():
            self.real_track_frame.update({RealTarget.id_fs: [RealTarget.frame]})
        else:
            self.real_track_frame[RealTarget.id_fs].append(RealTarget.frame)
        if len(self.listRealTarget) == 0:
            self.listRealTarget.append(RealTargetInfo())
            self.listRealTarget[-1].target_id = RealTarget.id_fs
            self.listRealTarget[-1].type = RealTarget.class_fs
        else:
            b_target_exist = False
            for listTarget in self.listRealTarget:
                if listTarget.target_id == RealTarget.id_fs:
                    b_target_exist = True
                    break
            if not b_target_exist:                              # 是新的真值目标
                self.listRealTarget.append(RealTargetInfo())
                self.listRealTarget[-1].target_id = RealTarget.id_fs
                self.listRealTarget[-1].type = RealTarget.class_fs
        if RealTarget.id_fs not in self.real_type.keys():
            self.real_type.update({RealTarget.id_fs: RealTarget.class_fs})
        for listTarget in self.listRealTarget:
            if listTarget.target_id == RealTarget.id_fs:
                index = self.listRealTarget.index(listTarget)
                self.listRealTarget[index].real_frame_num += 1
                break
        # 找与当前真值目标匹配的待测目标
        b_ismatch = False
        test_index = -1
        for testTarget in testdata:
            test_index += 1
            if self.eachframetesttargetmatch[frameindex][test_index] == 0:  # 只有尚未匹配成功的目标才参与当前匹配
                TestTarget = Basic_information_fusion(testTarget, frameindex)
                if self.eachframetesttrajsave[frameindex][test_index] == 0:  # 当前待测目标轨迹尚未保存
                    self.test_track_save(TestTarget)
                    self.eachframetesttrajsave[frameindex][test_index] = 1
                    self.num_test_target[TestTarget.class_fs] += 1
                    # 记录待测中每帧各目标的id及类别
                    if frameindex not in self.frame_test_id_type.keys():
                        self.frame_test_id_type.update({frameindex: [[TestTarget.id_fs, TestTarget.class_fs]]})
                    else:
                        self.frame_test_id_type[frameindex].append([TestTarget.id_fs, TestTarget.class_fs])
                b_ismatch = self.is_real_test_match(RealTarget, TestTarget)
                if b_ismatch:
                    self.listRealTarget[index].dict_realtestmatch.update({RealTarget.frame: [TestTarget.id_fs, TestTarget.class_fs]})
                    self.listRealTarget[index].matchNum += 1
                    self.eachframetesttargetmatch[frameindex][test_index] = 1
                    # 在已匹配的目标中,判断类别,统计分类正确率
                    if RealTarget.class_fs == TestTarget.class_fs:
                        self.num_recognition_right[RealTarget.class_fs] += 1        # 准确识别的样本数
                    else:
                        self.num_recognition_sort_wrong[RealTarget.class_fs] += 1   # 识别但分类错误的样本数
                        # 保存识别但分类错误的帧
                        if RealTarget.id_fs not in self.class_wrong_frame.keys():
                            self.class_wrong_frame.update({RealTarget.id_fs: [RealTarget.frame]})
                        else:
                            self.class_wrong_frame[RealTarget.id_fs].append(RealTarget.frame)
                        # 保存识别但分类错误的类别
                        # self.num_wrong[RealTarget.class_fs] += 1
                        if RealTarget.id_fs not in self.class_wrong_type.keys():
                            self.class_wrong_type.update({RealTarget.id_fs: [TestTarget.class_fs]})
                        else:
                            if TestTarget.class_fs not in self.class_wrong_type[RealTarget.id_fs]:
                                self.class_wrong_type[RealTarget.id_fs].append(TestTarget.class_fs)
                    break
        if not b_ismatch:
            self.listRealTarget[index].dict_realtestmatch.update({RealTarget.frame: [-1, -1]})
            # 该真值目标的当前帧未匹配(部分缺失)
            if RealTarget.id_fs not in self.part_mismatch.keys():
                self.part_mismatch.update({RealTarget.id_fs: [RealTarget.frame]})
            else:
                self.part_mismatch[RealTarget.id_fs].append(RealTarget.frame)
            self.num_miss[RealTarget.class_fs] += 1     # 漏检的样本数

    def is_real_test_match(self, realtarget, testtarget):
        """判断当前真值目标和当前待测目标是否为同一目标"""
        """计算IOU，看是否大于阈值，是则return True"""
        real = np.array([[realtarget.x, realtarget.y,  realtarget.width, realtarget.length, math.radians(realtarget.angle), realtarget.z, realtarget.height, realtarget.class_fs]]) #X,Y,W,L,角度,Z,H,class
        test = np.array([[testtarget.x, testtarget.y,  testtarget.width, testtarget.length, math.radians(testtarget.angle), testtarget.z, testtarget.height, testtarget.class_fs]]) #X,Y,W,L,角度,Z,H,class
        iou = compute_iou.Iou_Two_Box(real, test)
        if iou >= iou_Threshold:
            return True
        else:
            return False

    def Statistics_Rate(self):
        """统计各种率"""
        for realTarget in self.listRealTarget:
            realTarget_index = self.listRealTarget.index(realTarget)
            b_target_fst_data = True
            fst_id_num = 0
            fst_id = 0
            if realTarget.matchNum != 0:
                for value in realTarget.dict_realtestmatch.values():
                    if value[0] != -1:
                        if b_target_fst_data:
                            b_target_fst_data = False
                            fst_id = value[0]
                            fst_id_num = 1
                        else:
                            if value[0] == fst_id:
                                fst_id_num += 1
                        if value[0] not in self.listRealTarget[realTarget_index].list_test_id:
                            self.listRealTarget[realTarget_index].list_test_id.append(value[0])
                        if value[1] not in self.listRealTarget[realTarget_index].list_test_type:
                            self.listRealTarget[realTarget_index].list_test_type.append(value[1])
                if len(self.listRealTarget[realTarget_index].list_test_id) == 1:
                    # 属于跟踪范畴,再讨论跟踪成功还是失败
                    if fst_id_num == len(realTarget.dict_realtestmatch):  # 跟踪成功,条件考虑是否要调整
                        if len(self.listRealTarget[realTarget_index].list_test_type) == 1:     # 类型数量为1,
                            self.track_success_num_60[realTarget.type] += 1     # 也是60%匹配
                            if realTarget.type == self.listRealTarget[realTarget_index].list_test_type[0]:  # 分类也对
                                self.track_success_num[realTarget.type] += 1
                                self.listRealTarget[realTarget_index].b_track_success = True
                                self.track_success_num_loose[realTarget.type] += 1
                            else:
                                self.id_only_type_jump.update({realTarget.target_id: [realTarget.list_test_type]})
                        else:
                            # self.track_fail_num += 1
                            self.id_only_type_jump.update({realTarget.target_id: [realTarget.list_test_type]})
                    else:
                        # 找出首帧和尾帧,通过关系来确定哪段没跟上
                        # list_real_frame = [x for x in realTarget.dict_realtestmatch.keys()]
                        # list_test_frame = [realTarget.dict_realtestmatch[x][0] for x in realTarget.dict_realtestmatch.keys()]
                        # test_begin_frame = list_test_frame[0]
                        # test_end_frame = list_test_frame[-1]
                        # real_begin_frame = list_real_frame[0]
                        # real_end_frame = list_real_frame[-1]
                        # if test_begin_frame < real_begin_frame and test_end_frame == real_end_frame:
                        #     self.listRealTarget[realTarget_index].track_type = '首端没跟上'
                        # if test_begin_frame == real_begin_frame and test_end_frame < real_end_frame:
                        #     self.listRealTarget[realTarget_index].track_type = '尾端没跟上'
                        # if test_begin_frame == real_begin_frame and test_end_frame == real_end_frame:
                        #     self.listRealTarget[realTarget_index].track_type = '中间没跟上'
                        # if test_begin_frame == real_begin_frame and test_end_frame == real_end_frame:
                        #     self.listRealTarget[realTarget_index].track_type = '首尾没跟上'
                        # 下面判断是不是非严格跟踪成功，ID唯一且匹配数达到80%，不管类型
                        if realTarget.matchNum >= 0.8 * len(realTarget.dict_realtestmatch):
                            self.track_success_num_loose[realTarget.type] += 1
                        # 下面判断另一种跟踪成功，ID唯一且匹配数达到60%，类型唯一
                        if realTarget.matchNum >= 0.6 * len(realTarget.dict_realtestmatch):
                            if len(self.listRealTarget[realTarget_index].list_test_type) == 1:  # 类型数量为1,
                                # if realTarget.type == self.listRealTarget[realTarget_index].list_test_type[0]:
                                self.track_success_num_60[realTarget.type] += 1
                if len(self.listRealTarget[realTarget_index].list_test_id) > 1:
                    # 属于ID跳变
                    self.listRealTarget[realTarget_index].track_type = 'ID跳变'
                    if realTarget.target_id not in self.id_jump.keys():
                        self.id_jump.update({realTarget.target_id: self.listRealTarget[realTarget_index].list_test_id})
            else:
                self.list_none_match.append([realTarget.target_id, self.listRealTarget[realTarget_index].type,
                                             self.start_position[realTarget.target_id][2],
                                             self.end_position[realTarget.target_id][2], '全程无匹配'])
                self.listRealTarget[realTarget_index].track_type = '全程无匹配'
            self.track_num[realTarget.type] += 1
        #  对刚才的数据做汇总处理
        print('\n')
        for i in range(len(self.track_success_rate)):
            if self.track_num[i] != 0:
                self.track_success_rate[i] = float(self.track_success_num[i]) / self.track_num[i]
                self.track_success_rate_loose[i] = float(self.track_success_num_loose[i]) / self.track_num[i]
                self.track_success_rate_60[i] = float(self.track_success_num_60[i]) / self.track_num[i]
            else:
                self.track_success_rate[i] = 0
                self.track_success_rate_loose[i] = 0
                self.track_success_rate_60[i] = 0
            print('类别%d的严格跟踪成功率为:%.3f' % (i, self.track_success_rate[i]))
            print('类别%d的(0.8匹配)跟踪成功率为:%.3f' % (i, self.track_success_rate_loose[i]))
            print('类别%d的(0.6匹配)跟踪成功率为:%.3f' % (i, self.track_success_rate_60[i]))
        if len(self.id_jump) == 0:
            self.id_jump_info.append([None] * 6)
        else:
            for key, value in self.id_jump.items():
                self.id_jump_info.append([key, self.real_type[key], self.start_position[key][2],
                                          self.end_position[key][2], 'id跳变', value])
        if len(self.id_only_type_jump) == 0:
            self.type_jump_info.append([None] * 7)
        else:
            test_id = []
            for key, value in self.id_only_type_jump.items():
                for target in self.listRealTarget:
                    if target.target_id == key:
                        test_id = target.list_test_id
                self.type_jump_info.append([key, self.real_type[key], self.start_position[key][2],
                                            self.end_position[key][2], test_id, '类型跳变', value])
        if len(self.part_mismatch) == 0:
            self.part_mismatch_info.append([None] * 7)
        else:
            test_id = []
            for key, value in self.part_mismatch.items():
                for target in self.listRealTarget:
                    if target.target_id == key:
                        test_id = target.list_test_id
                self.part_mismatch_info.append([key, self.real_type[key], self.start_position[key][2],
                                                self.end_position[key][2], test_id, '部分帧未匹配', value])
        qqq = 0

    def real_track_save(self, data):
        """储存每个真值目标ID的历史位置"""
        if data.id_fs not in self.track_save.keys():
            self.track_save.update({data.id_fs: [[data.x, data.y, data.z]]})
            self.track_long.update({data.id_fs: 0})
            self.class_stat.update({data.id_fs: np.zeros(7)})
            self.frame_stat.update({data.id_fs: 1})
        else:
            self.class_stat[data.id_fs][data.class_fs] += 1
            self.frame_stat[data.id_fs] += 1
            det_dis = np.sqrt((data.x - self.track_save[data.id_fs][-1][0])**2 +
                              (data.y - self.track_save[data.id_fs][-1][1])**2)
            if det_dis >= 0.5:          # 剔除一些小抖动造成的位置
                self.track_save[data.id_fs].append([data.x, data.y, data.z])
                self.track_long[data.id_fs] += det_dis          # 计算跟踪链长度,两帧间sqrt((x2-x1)^2+(y2-y1)^2)

    def test_track_save(self, data):
        """储存每个待测ID的历史位置"""
        if data.id_fs not in self.track_save_test.keys():
            self.track_save_test.update({data.id_fs: [[data.x, data.y, data.z]]})
            self.track_long_test.update({data.id_fs: 0})
            self.class_stat_test.update({data.id_fs: np.zeros(7)})
            self.frame_stat_test.update({data.id_fs: 1})
        else:
            self.class_stat_test[data.id_fs][data.class_fs] += 1
            self.frame_stat_test[data.id_fs] += 1
            det_dis = np.sqrt((data.x - self.track_save_test[data.id_fs][-1][0])**2 +
                              (data.y - self.track_save_test[data.id_fs][-1][1])**2)
            if det_dis >= 0.5:          # 剔除一些小抖动造成的位置
                self.track_save_test[data.id_fs].append([data.x, data.y, data.z])
                self.track_long_test[data.id_fs] += det_dis          # 计算跟踪链长度,两帧间sqrt((x2-x1)^2+(y2-y1)^2)

    def cal_track_length(self):
        """统计跟踪链长度,只统计跟踪成功的目标,分类别统计"""
        real_length = [[], [], [], []]
        test_length = [[], [], [], []]
        type_idx = -1
        for key, value in self.track_long.items():
            if value != 0:
                for idx in range(len(self.listRealTarget)):
                    if key == self.listRealTarget[idx].target_id:
                        type_idx = self.listRealTarget[idx].type
                        if self.listRealTarget[idx].b_track_success:
                            real_length[type_idx].append(value)
                            for k in self.track_long_test.keys():
                                if k == self.listRealTarget[idx].list_test_id[0]:
                                    test_length[type_idx].append(self.track_long_test[k])
        for i in range(class_num):
            if len(real_length[i]) == 0:
                self.track_length_real[i] = 0
            else:
                self.track_length_real[i] = np.mean(real_length[i])
            if len(test_length[i]) == 0:
                self.track_length_test[i] = 0
            else:
                self.track_length_test[i] = np.mean(test_length[i])

    def cal_sort_index(self):
        """统计分类相关指标"""
        num_not_in_man_area = 0
        frame_index = -1
        for frame in self.eachframetesttargetmatch:
            frame_index += 1
            target_index = -1
            for target in frame:    # frame是一个列表,内容为一帧待测数据中各目标是否匹配的标志位,为0或1
                target_index += 1
                if target == 0:
                    if (not self.b_area) or (self.b_area and self.each_frame_test_in_area[frame_index][target_index]):
                        # 说明是虚检
                        target_type = self.frame_test_id_type[frame_index][target_index][1]
                        self.num_fake[target_type] += 1
                        test_id = self.frame_test_id_type[frame_index][target_index][0]
                        if test_id not in self.class_fake_class.keys():
                            self.class_fake_class.update({test_id: [target_type]})
                        else:
                            if target_type not in self.class_fake_class[test_id]:
                                self.class_fake_class[test_id].append(target_type)
                        if test_id not in self.class_fake.keys():
                            self.class_fake.update({test_id: [frame_index]})
                        else:
                            if frame_index not in self.class_fake[test_id]:
                                self.class_fake[test_id].append(frame_index)
                        if not self.b_area and target_type == 3:
                            if self.each_frame_test_in_man_area[frame_index][target_index]:
                                # 虚检的行人类型在行人区域内
                                self.num_fake_in_man_area += 1
        for i in range(class_num):
            self.num_recognition_right_all += self.num_recognition_right[i]
            self.num_recognition_sort_wrong_all += self.num_recognition_sort_wrong[i]
            # self.num_wrong_all += self.num_wrong[i]
            self.num_fake_all += self.num_fake[i]
            self.num_miss_all += self.num_miss[i]
            self.num_real_target_all += self.num_real_target[i]
            self.num_test_target_all += self.num_test_target[i]
            if self.num_real_target[i] == 0:
                self.rate_recognition_right[i] = -1
                self.rate_miss[i] = -1
            else:
                self.rate_recognition_right[i] = self.num_recognition_right[i] / self.num_real_target[i]
                self.rate_miss[i] = self.num_miss[i] / self.num_real_target[i]
            if (self.num_recognition_right[i] + self.num_recognition_sort_wrong[i]) == 0:
                self.rate_sort_right[i] = -1
            else:
                self.rate_sort_right[i] = self.num_recognition_right[i] / (self.num_recognition_right[i] + self.num_recognition_sort_wrong[i])
            if self.num_test_target[i] == 0:
                self.rate_wrong[i] = -1
                self.rate_fake[i] = -1
            else:
                self.rate_wrong[i] = self.num_recognition_sort_wrong[i] / self.num_test_target[i]
                self.rate_fake[i] = self.num_fake[i] / self.num_test_target[i]
            if self.num_real_in_man_area == 0:
                self.rate_miss_in_man_area = 0
                self.rate_wrong_in_man_area = 0
            else:
                self.rate_miss_in_man_area = self.num_miss_in_man_area / self.num_real_in_man_area
                self.rate_wrong_in_man_area = self.num_wrong_in_man_area / self.num_real_in_man_area
            if self.num_test_in_man_area == 0:
                self.rate_fake_in_man_area = 0
            else:
                self.rate_fake_in_man_area = self.num_fake_in_man_area / self.num_test_in_man_area
        self.rate_recognition_right_all = self.num_recognition_right_all / self.num_real_target_all
        self.rate_miss_all = self.num_miss_all / self.num_real_target_all
        self.rate_wrong_all = self.num_recognition_sort_wrong_all / self.num_real_target_all
        self.rate_fake_all = self.num_fake_all / self.num_test_target_all
        self.rate_sort_right_all = self.num_recognition_right_all / (self.num_recognition_right_all + self.num_recognition_sort_wrong_all)
        self.rate_recognition_right_Veh = self.num_recognition_right_Veh / (self.num_real_target[0] + self.num_real_target[1])
        print('待测大车正确识别数量%d' % (self.num_recognition_right[0]))
        print('待测小车正确识别数量%d' % (self.num_recognition_right[1]))
        print('真值大车数量%d' % (self.num_real_target[0]))
        print('真值小车数量%d' % (self.num_real_target[1]))
        print('待测机动车正确识别数量%d' % self.num_recognition_right_Veh)
        # 把分类异常的信息导出来
        # 分类错误(误检)
        if len(self.class_wrong_frame) == 0:
            self.class_wrong_info.append([None] * 6)
        else:
            for key, value in self.class_wrong_frame.items():
                self.class_wrong_info.append([key, self.real_type[key], self.start_position[key][2],
                                              self.end_position[key][2], self.class_wrong_type[key], value])
        # 漏检
        if len(self.part_mismatch) == 0:
            self.class_miss_info.append([None] * 5)
        else:
            for key, value in self.part_mismatch.items():
                self.class_miss_info.append([key, self.real_type[key], self.start_position[key][2],
                                             self.end_position[key][2], value])
        # 虚检
        if len(self.class_fake) == 0:
            self.class_fake_info.append([None] * 3)
        else:
            for key, value in self.class_fake.items():
                self.class_fake_info.append([key, self.class_fake_class[key], value])


    def Track_cache(self, data):
        """储存每个ID的历史位置"""
        if data.id_fs not in self.track_save.keys():
            self.track_save.update({data.id_fs: [[data.x, data.y, data.z]]})
            self.track_long.update({data.id_fs : 0})
            self.class_stat.update({data.id_fs : np.zeros(7)})
            self.frame_stat.update({data.id_fs : 1})
        else:
            self.class_stat[data.id_fs][data.class_fs] += 1
            self.frame_stat[data.id_fs] += 1
            det_dis = np.sqrt((data.x - self.track_save[data.id_fs][-1][0])**2 +
                              (data.y - self.track_save[data.id_fs][-1][1])**2)
            if det_dis >= 1:
                if len(self.track_save[data.id_fs]) == 50:  # 限制只画50帧
                    self.track_save[data.id_fs].pop(0)
                self.track_save[data.id_fs].append([data.x, data.y, data.z])
                self.track_long[data.id_fs] += det_dis          # 计算跟踪链长度,两帧间sqrt((x2-x1)^2+(y2-y1)^2)

    def Position_stat(self, data):
        """统计起止点"""
        self.end_position.update({data.id_fs : [data.x, data.y, data.frame, data.road]})
        if data.id_fs not in self.start_position.keys():
            self.start_position.update({data.id_fs : [data.x, data.y, data.frame, data.road]})

    def Position_judge(self):
        """判断起止点属于什么位置"""
        # blind_area1 = [-7.99422, 6.79018, -7.81213, -2.59325]    # 雷达盲区
        # blind_area2 = [17.6861, 26.5111, -26.0202, -24.4131]     # 路中盲区
        blind_area1 = []    # 雷达盲区
        blind_area2 = []     # 路中盲区

        for k, v in self.start_position.items():
            if v[0] < self.axis_limit_car[self.final_class_stat1[k]][0]:
                self.start_zone.update({k : 'left'})
            elif v[0] > self.axis_limit_car[self.final_class_stat1[k]][1]:
                self.start_zone.update({k : 'right'})
            else:
                if v[1] < self.axis_limit_car[self.final_class_stat1[k]][2]:
                    self.start_zone.update({k : 'down'})
                elif v[1] > self.axis_limit_car[self.final_class_stat1[k]][3]:
                    self.start_zone.update({k : 'up'})
                else:
                    self.start_zone.update({k : 'mid'})
            if self.final_class_stat1[k] in [0, 2]:
                if len(blind_area1):
                    if blind_area1[0] <= v[0] <= blind_area1[1] and blind_area1[2] <= v[1] <= blind_area1[3]:
                        self.start_zone.update({k : 'bld1'})
                if len(blind_area2):
                    if blind_area2[0] <= v[0] <= blind_area2[1] and blind_area2[2] <= v[1] <= blind_area2[3]:
                        self.start_zone.update({k : 'bld2'})
        for k, v in self.end_position.items():
            if v[0] < self.axis_limit_car[self.final_class_stat1[k]][0]:
                self.end_zone.update({k : 'left'})
            elif v[0] > self.axis_limit_car[self.final_class_stat1[k]][1]:
                self.end_zone.update({k : 'right'})
            else:
                if v[1] < self.axis_limit_car[self.final_class_stat1[k]][2]:
                    self.end_zone.update({k : 'down'})
                elif v[1] > self.axis_limit_car[self.final_class_stat1[k]][3]:
                    self.end_zone.update({k : 'up'})
                else:
                    self.end_zone.update({k : 'mid'})
            if self.final_class_stat1[k] in [0, 2]:
                if len(blind_area1):
                    if blind_area1[0] <= v[0] <= blind_area1[1] and blind_area1[2] <= v[1] <= blind_area1[3]:
                        self.end_zone.update({k : 'bld1'})
                if len(blind_area2):
                    if blind_area2[0] <= v[0] <= blind_area2[1] and blind_area2[2] <= v[1] <= blind_area2[3]:
                        self.end_zone.update({k : 'bld2'})

    def Car_in_or_our(self, start_frame, data_len):
        """记录起始、终止、断链ID"""
        temp = []
        for k, v in self.end_position.items():
            # 驶入ID统计
            if (self.start_zone[k] != 'mid' and self.start_zone[k] != self.end_zone[k]) \
                    or (self.start_zone[k] == 'mid' and self.start_position[k][2] < 10):
                # 起始区域不在中间，且起始区域与终止区域不同 or 起始时间在数据前10帧内且起始区域在中间的
                self.car_in[self.final_class_stat1[k]].extend([k])
            # 驶出ID统计
            if self.end_zone[k] != 'mid' and self.start_zone[k] != self.end_zone[k]:
                # 终止区域不在中间，且起始区域与终止区域不同
                self.car_out[self.final_class_stat1[k]].extend([k])
            # 起始帧在开始10帧内，且在中心区域开始的
            if self.start_position[k][2] < start_frame + 10 and self.start_zone[k] == 'mid':
                temp.extend([k])
        # 断链统计
        for hh in range(3):
            for i in self.car_in[hh]:
                if self.end_zone[i] == 'mid' and self.end_position[i][2] < data_len - 10:
                    # ID驶入但未驶出，且结束时间不在数据结束前10帧
                    self.broken_id[hh].extend([i])
        # 有效ID统计
        # self.effective_id = self.car_in.copy()
        # for i in temp:
        #     if i not in self.effective_id:
        #         self.effective_id.extend([i])
        # 断链详情统计
        for hh in range(3):
            for i in self.broken_id[hh]:
                self.broken_stat.append([i, int(self.end_position[i][2] - self.start_position[i][2] + 1),
                                         self.track_long[i], self.start_position[i][2],
                                         self.start_position[i][0], self.start_position[i][1],
                                         self.start_zone[i], self.end_position[i][2],
                                         self.end_position[i][0], self.end_position[i][1],
                                         self.end_zone[i], self.final_class_stat2[i]])

    def Diff_class_tarck(self):
        """机动车/非机动车跟踪链长度统计"""
        temp1, temp2, temp3 = [], [], []
        for k, v in self.class_stat.items():
            if np.argmax(v) == 4:
                self.class_judge[2].append(k)
                self.final_class_stat1.update({k: 2})
            elif (v[1] + v[2] + v[3] + v[4]) >= (v[0] + v[5] + v[6]):  # 非机动车
                self.class_judge[0].append(k)
                self.final_class_stat1.update({k : 0})
            else:
                self.class_judge[1].append(k)
                self.final_class_stat1.update({k : 1})
            temp_class = np.argmax(v)
            self.final_class_stat2.update({k : temp_class})
        for i in self.class_judge[0]:
            # 跟踪链长度
            if self.track_long[i] != 0:
                temp1.append(self.track_long[i])
            # 每帧检出率
            if self.end_position[i][2] - self.start_position[i][2] != 0:
                temp_detec_prop = self.frame_stat[i] / (self.end_position[i][2] - self.start_position[i][2] + 1)
                self.frame_detec_prop.update({i: temp_detec_prop})
                self.frame_prop[0].append(temp_detec_prop)
        for i in self.class_judge[1]:
            # 跟踪链长度
            if self.track_long[i] != 0:
                temp2.append(self.track_long[i])
            # 每帧检出率
            if self.end_position[i][2] - self.start_position[i][2] != 0:
                temp_detec_prop = self.frame_stat[i] / (self.end_position[i][2] - self.start_position[i][2] + 1)
                self.frame_detec_prop.update({i : temp_detec_prop})
                self.frame_prop[1].append(temp_detec_prop)
        for i in self.class_judge[2]:
            # 跟踪链长度
            if self.track_long[i] != 0:
                temp3.append(self.track_long[i])
            # 每帧检出率
            if self.end_position[i][2] - self.start_position[i][2] != 0:
                temp_detec_prop = self.frame_stat[i] / (self.end_position[i][2] - self.start_position[i][2] + 1)
                self.frame_detec_prop.update({i: temp_detec_prop})
                self.frame_prop[2].append(temp_detec_prop)
        # 跟踪链长度
        self.track_two_len[0] = np.mean(temp1)
        self.track_two_len[1] = np.mean(temp2)
        self.track_two_len[2] = np.mean(temp3)
        # 每秒检出率
        self.frame_everymin[0] = np.mean(self.frame_prop[0]) * 10
        self.frame_everymin[1] = np.mean(self.frame_prop[1]) * 10
        self.frame_everymin[2] = np.mean(self.frame_prop[2]) * 10

    def Car_for_20_50(self):
        """机动车20/50m统计"""
        temp_dis_out = []
        xmin, xmax, ymin, ymax = self.axis_limit_car[1]
        for i in self.class_judge[1]:
            # 驶入目标中，驶入方向距离20m以上
            if i in self.car_in[1]:
                temp_x = self.start_position[i][0]
                temp_y = self.start_position[i][1]
                temp_f = self.start_position[i][2]
                temp_r = self.start_position[i][3]
                temp_start_zone = self.start_zone[i]
                if temp_start_zone == 'mid':   # 起始帧5帧以内，不计算为失败的
                    self.drive_in_20.append(i)
                else:
                    if xmin <= temp_x <= xmax:   # 在区域上/下
                        if temp_y >= ymax:  # 在区域上
                            if temp_y - ymax >= 20:
                                self.drive_in_20.append(i)
                            if temp_r <= 20:
                                self.road_dis_in[temp_r].update({i : temp_y - ymax})
                        elif temp_y <= ymin:   # 在区域下
                            if ymin - temp_y >= 20:
                                self.drive_in_20.append(i)
                            if temp_r <= 20:
                                self.road_dis_in[temp_r].update({i : ymin - temp_y})
                        else:
                            pass
                    else:    # 在区域左/右
                        if temp_x >= xmax:   # 在区域右
                            if temp_x - xmax >= 20:
                                self.drive_in_20.append(i)
                            if temp_r <= 20:
                                self.road_dis_in[temp_r].update({i : temp_x - xmax})
                        elif temp_x <= xmin:   # 在区域左
                            if xmin - temp_x >= 20:
                                self.drive_in_20.append(i)
                            if temp_r <= 20:
                                self.road_dis_in[temp_r].update({i : xmin - temp_x})
                        else:
                            pass
            # 驶出目标中，驶出方向距离50m以上
            if i in self.car_out[1]:
                temp_x = self.end_position[i][0]
                temp_y = self.end_position[i][1]
                temp_f = self.end_position[i][2]
                temp_r = self.end_position[i][3]
                temp_end_zone = self.end_zone[i]
                if temp_end_zone == 'mid':   # 终止帧10帧以内，不计算为失败的
                    self.drive_out_50.append(i)
                else:
                    if xmin <= temp_x <= xmax:   # 在区域上/下
                        if temp_y >= ymax:  # 在区域上
                            temp_dis_out.append(temp_y - ymax)
                            if temp_y - ymax >= 50:
                                self.drive_out_50.append(i)
                            if temp_r <= 20:
                                self.road_dis_out[temp_r].update({i : temp_y - ymax})
                        elif temp_y <= ymin:   # 在区域下
                            temp_dis_out.append(ymin - temp_y)
                            if ymin - temp_y >= 50:
                                self.drive_out_50.append(i)
                            if temp_r <= 20:
                                self.road_dis_out[temp_r].update({i: ymin - temp_y})
                        else:
                            pass
                    else:    # 在区域左/右
                        if temp_x >= xmax:   # 在区域右
                            temp_dis_out.append(temp_x - xmax)
                            if temp_x - xmax >= 50:
                                self.drive_out_50.append(i)
                            if temp_r <= 20:
                                self.road_dis_out[temp_r].update({i: temp_x - xmax})
                        elif temp_x <= xmin:   # 在区域左
                            temp_dis_out.append(xmin - temp_x)
                            if xmin - temp_x >= 50:
                                self.drive_out_50.append(i)
                            if temp_r <= 20:
                                self.road_dis_out[temp_r].update({i: xmin - temp_x})
                        else:
                            pass
            # 未断链目标中，均满足20/50m要求的目标
            if i in self.car_in[1] and i in self.car_out[1]:
                if i in self.drive_in_20 and i in self.drive_out_50:
                    self.drive_both_20_50.append(i)
        # print('\n', np.mean(temp_dis_out))

        # 统计每条路不同距离比例
        for i in range(len(self.road_dis_in)):
            if i == 3:
                a = 1
            if len(self.road_dis_in[i]) != 0:
                count_num = np.zeros(3)
                for v in self.road_dis_in[i].values():
                    if 0 < v < 10:
                        count_num[0] += 1
                    elif 10 <= v < 20:
                        count_num[1] += 1
                    else:
                        count_num[2] += 1
                for num in range(3):
                    temp_prop = round(count_num[num] / len(self.road_dis_in[i]), 4)
                    self.road_in_prop[i, num] = temp_prop
        for i in range(len(self.road_dis_out)):
            if len(self.road_dis_out[i]) != 0:
                count_num = np.zeros(3)
                for v in self.road_dis_out[i].values():
                    if 0 < v < 25:
                        count_num[0] += 1
                    elif 25 <= v < 50:
                        count_num[1] += 1
                    else:
                        count_num[2] += 1
                for num in range(3):
                    temp_prop = round(count_num[num] / len(self.road_dis_out[i]), 4)
                    self.road_out_prop[i, num] = temp_prop

    def Class_right_stat(self, data):
        """统计测试区类别准确率，轨迹长度"""
        class_all = self.final_class_stat1[data.id_fs]
        temp_axis = self.axis_limit_car[class_all]
        if temp_axis[0] <= data.x <= temp_axis[1] and temp_axis[2] <= data.y <= temp_axis[3]:
            self.class_right[class_all][1] += 1
            if data.class_huge == class_all:
                self.class_right[class_all][0] += 1
            else:
                if data.id_fs not in self.class_wrong_frame.keys():
                    self.class_wrong_frame.update({data.id_fs : [data.frame]})
                else:
                    self.class_wrong_frame[data.id_fs].append(data.frame)
            if class_all == 0:
                if data.id_fs not in self.zone_start.keys():
                    self.zone_start.update({data.id_fs : [data.x, data.y, data.z]})
                self.zone_end.update({data.id_fs : [data.x, data.y, data.z]})
                if data.id_fs not in self.zone_point.keys():
                    self.zone_point.update({data.id_fs : [data.x, data.y, data.z]})
                    self.zone_track.update({data.id_fs : 0})
                else:
                    det_dis = np.sqrt((data.x - self.zone_point[data.id_fs][0])**2 +
                                      (data.y - self.zone_point[data.id_fs][1])**2)
                    if det_dis >= 0.5:
                        self.zone_track[data.id_fs] += det_dis
                        self.zone_point.update({data.id_fs : [data.x, data.y, data.z]})

    def Class_final(self):
        """测试区内最终大类准确率统计，跟踪链长度比例"""
        for i in range(3):
            temp = round(self.class_right[i][0] / self.class_right[i][1] * 100, 2)
            self.final_class_right[i] = temp

        for j in self.class_wrong_frame.keys():
            self.class_wrong_stat.append([j, self.final_class_stat1[j], self.final_class_stat2[j],
                                          self.start_position[j][2], self.start_position[j][1], self.start_position[j][0],
                                          self.end_position[j][2], self.end_position[j][1], self.end_position[j][0],
                                          self.class_wrong_frame[j]])
        for k in self.zone_start.keys():
            if k in self.car_in[0]:
                det_x = abs(self.zone_start[k][0] - self.zone_end[k][0])
                det_y = abs(self.zone_start[k][1] - self.zone_end[k][1])
                det_tan = det_y / (det_x + 1e-7)
                if self.start_zone[k] in ['left', 'right'] and det_tan <= 10/23:      # det_y <= 10 and det_x > 10:
                    self.zone_track_num[1] += 1
                    if self.zone_track[k] >= 18:
                        self.zone_track_num[0] += 1



