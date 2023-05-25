import numpy as np
# from Target_angle_conversion import Target_angle_conversion

class Basic_information_fusion:
    """融合基本信息"""
    def __init__(self, data, frame):
        self.id_fs = int(data[9])    # id
        self.YC = int(data[10])  # id
        self.class_fs = int(data[7])  # 类别
        self.frame = frame
        self.x = float(data[0])
        self.y = float(data[1])
        self.z = float(data[5])
        self.width = float(data[3])
        self.length = float(data[2])
        self.height = float(data[6])
        self.speed = float(0)
        self.angle = float(data[4])
        if self.angle < 270:
            self.angle = 270 - self.angle
        else:
            self.angle = 630 - self.angle
        self.volume = self.length * self.width * self.height  # 体积
        self.box = xyz_to_8points([self.x, self.y, self.z],
                                  [self.width, self.length, self.height],
                                   self.angle)         # 检测框8角点
        self.color = [1, 1, 1]  # 白色
        # # 类别转化
        # # 1小客车
        # if temp_class_fs == 1:
        #     self.class_fs = 1
        #     self.color = [2, 0, 0]  # 红
        # # 2中巴车
        # elif temp_class_fs == 7:
        #     self.class_fs = 2
        #     self.color = [0, 2, 0]  # 绿
        # # 3大巴车
        # elif temp_class_fs == 3:
        #     self.class_fs = 3
        #     self.color = [0, 0, 2]  # 蓝
        # # 4行人
        # elif temp_class_fs == 4:
        #     self.class_fs = 4
        #     self.color = [2, 2, 0]  # 黄
        # # 5非机动车
        # elif temp_class_fs in [5, 6, 14] or 150 <= temp_class_fs <= 154 or 160 <= temp_class_fs <= 166:
        #     self.class_fs = 5
        #     self.color = [2, 0, 2]  # 紫
        # # 6小货车
        # elif temp_class_fs == 10:
        #     self.class_fs = 6
        #     self.color = [0, 2, 2]  # 青
        # # 7中货车
        # elif temp_class_fs == 11:
        #     self.class_fs = 7
        #     self.color = [2, 1, 1]  #
        # # 8大货车
        # elif temp_class_fs == 2:
        #     self.class_fs = 8
        #     self.color = [1, 2, 1]  #
        # # 9其他
        # else:
        #     self.class_fs = 9
        #     self.color = [2, 2, 2]  # 白


def xyz_to_8points(point_xyz, size, angle):
    """中心点xyz与宽长高，输出8角点xyz"""
    # angle_xyz = Target_angle_conversion(angle)
    angle_xyz = angle
    angle_xyz = angle_xyz / 180 * np.pi
    x1 = size[1] / 2
    x2 = -size[1] / 2
    y1 = size[0] / 2
    y2 = - size[0] / 2
    z1 = point_xyz[2] + size[2] / 2
    z2 = point_xyz[2] - size[2] / 2
    point_temp = [[x1, y2], [x2, y2], [x1, y1], [x2, y1]]
    point_s = []
    for i in point_temp:
        sx = i[0] * np.cos(angle_xyz) - i[1] * np.sin(angle_xyz) + point_xyz[0]
        sy = i[0] * np.sin(angle_xyz) + i[1] * np.cos(angle_xyz) + point_xyz[1]
        point_s.append([sx, sy])
    points = [[point_s[0][0], point_s[0][1], z2], [point_s[0][0], point_s[0][1], z1],
              [point_s[1][0], point_s[1][1], z1], [point_s[1][0], point_s[1][1], z2],
              [point_s[2][0], point_s[2][1], z2], [point_s[2][0], point_s[2][1], z1],
              [point_s[3][0], point_s[3][1], z1], [point_s[3][0], point_s[3][1], z2]]
    return points

class Basic_information_fusion_det:
    """融合基本信息"""
    def __init__(self, data, frame):
        self.id_fs = int(0)    # id
        self.YC = int(0)  # id
        self.class_fs = int(data[7])  # 类别
        self.frame = frame
        self.x = float(data[0])
        self.y = float(data[1])
        self.z = float(data[5])
        self.width = float(data[3])
        self.length = float(data[2])
        self.height = float(data[6])
        self.speed = float(0)
        self.angle = float(data[4])
        if self.angle < 270:
            self.angle = 270 - self.angle
        else:
            self.angle = 630 - self.angle
        self.volume = self.length * self.width * self.height  # 体积
        self.box = xyz_to_8points([self.x, self.y, self.z],
                                  [self.width, self.length, self.height],
                                   self.angle)         # 检测框8角点
        self.color = [0, 1, 0]  # 白色
        # # 类别转化
        # # 1小客车
        # if temp_class_fs == 1:
        #     self.class_fs = 1
        #     self.color = [2, 0, 0]  # 红
        # # 2中巴车
        # elif temp_class_fs == 7:
        #     self.class_fs = 2
        #     self.color = [0, 2, 0]  # 绿
        # # 3大巴车
        # elif temp_class_fs == 3:
        #     self.class_fs = 3
        #     self.color = [0, 0, 2]  # 蓝
        # # 4行人
        # elif temp_class_fs == 4:
        #     self.class_fs = 4
        #     self.color = [2, 2, 0]  # 黄
        # # 5非机动车
        # elif temp_class_fs in [5, 6, 14] or 150 <= temp_class_fs <= 154 or 160 <= temp_class_fs <= 166:
        #     self.class_fs = 5
        #     self.color = [2, 0, 2]  # 紫
        # # 6小货车
        # elif temp_class_fs == 10:
        #     self.class_fs = 6
        #     self.color = [0, 2, 2]  # 青
        # # 7中货车
        # elif temp_class_fs == 11:
        #     self.class_fs = 7
        #     self.color = [2, 1, 1]  #
        # # 8大货车
        # elif temp_class_fs == 2:
        #     self.class_fs = 8
        #     self.color = [1, 2, 1]  #
        # # 9其他
        # else:
        #     self.class_fs = 9
        #     self.color = [2, 2, 2]  # 白


def xyz_to_8points(point_xyz, size, angle):
    """中心点xyz与宽长高，输出8角点xyz"""
    # angle_xyz = Target_angle_conversion(angle)
    angle_xyz = angle
    angle_xyz = angle_xyz / 180 * np.pi
    x1 = size[1] / 2
    x2 = -size[1] / 2
    y1 = size[0] / 2
    y2 = - size[0] / 2
    z1 = point_xyz[2] + size[2] / 2
    z2 = point_xyz[2] - size[2] / 2
    point_temp = [[x1, y2], [x2, y2], [x1, y1], [x2, y1]]
    point_s = []
    for i in point_temp:
        sx = i[0] * np.cos(angle_xyz) - i[1] * np.sin(angle_xyz) + point_xyz[0]
        sy = i[0] * np.sin(angle_xyz) + i[1] * np.cos(angle_xyz) + point_xyz[1]
        point_s.append([sx, sy])
    points = [[point_s[0][0], point_s[0][1], z2], [point_s[0][0], point_s[0][1], z1],
              [point_s[1][0], point_s[1][1], z1], [point_s[1][0], point_s[1][1], z2],
              [point_s[2][0], point_s[2][1], z2], [point_s[2][0], point_s[2][1], z1],
              [point_s[3][0], point_s[3][1], z1], [point_s[3][0], point_s[3][1], z2]]
    return points



class Basic_information_fusion1:
    """融合基本信息"""
    def __init__(self, data, frame):
        self.id_fs = int(data[0])    # id
        self.class_fs = int(data[1])  # 类别
        self.frame = frame
        self.x = float(data[2]) / 100
        self.y = float(data[3]) / 100
        self.z = float(data[4]) / 100
        self.width = float(data[8]) / 100
        self.length = float(data[7]) / 100
        self.height = float(data[9]) / 100
        self.speed = float(data[5])
        self.angle = float(data[6])
        self.volume = self.length * self.width * self.height  # 体积
        self.box = xyz_to_8points([self.x, self.y, self.z],
                                  [self.width, self.length, self.height],
                                   self.angle)         # 检测框8角点
        self.color = [1, 1, 1]  # 白色
        # # 类别转化
        # # 1小客车
        # if temp_class_fs == 1:
        #     self.class_fs = 1
        #     self.color = [2, 0, 0]  # 红
        # # 2中巴车
        # elif temp_class_fs == 7:
        #     self.class_fs = 2
        #     self.color = [0, 2, 0]  # 绿
        # # 3大巴车
        # elif temp_class_fs == 3:
        #     self.class_fs = 3
        #     self.color = [0, 0, 2]  # 蓝
        # # 4行人
        # elif temp_class_fs == 4:
        #     self.class_fs = 4
        #     self.color = [2, 2, 0]  # 黄
        # # 5非机动车
        # elif temp_class_fs in [5, 6, 14] or 150 <= temp_class_fs <= 154 or 160 <= temp_class_fs <= 166:
        #     self.class_fs = 5
        #     self.color = [2, 0, 2]  # 紫
        # # 6小货车
        # elif temp_class_fs == 10:
        #     self.class_fs = 6
        #     self.color = [0, 2, 2]  # 青
        # # 7中货车
        # elif temp_class_fs == 11:
        #     self.class_fs = 7
        #     self.color = [2, 1, 1]  #
        # # 8大货车
        # elif temp_class_fs == 2:
        #     self.class_fs = 8
        #     self.color = [1, 2, 1]  #
        # # 9其他
        # else:
        #     self.class_fs = 9
        #     self.color = [2, 2, 2]  # 白


def xyz_to_8points(point_xyz, size, angle):
    """中心点xyz与宽长高，输出8角点xyz"""
    # angle_xyz = Target_angle_conversion(angle)
    angle_xyz = angle
    angle_xyz = angle_xyz / 180 * np.pi
    x1 = size[1] / 2
    x2 = -size[1] / 2
    y1 = size[0] / 2
    y2 = - size[0] / 2
    z1 = point_xyz[2] + size[2] / 2
    z2 = point_xyz[2] - size[2] / 2
    point_temp = [[x1, y2], [x2, y2], [x1, y1], [x2, y1]]
    point_s = []
    for i in point_temp:
        sx = i[0] * np.cos(angle_xyz) - i[1] * np.sin(angle_xyz) + point_xyz[0]
        sy = i[0] * np.sin(angle_xyz) + i[1] * np.cos(angle_xyz) + point_xyz[1]
        point_s.append([sx, sy])
    points = [[point_s[0][0], point_s[0][1], z2], [point_s[0][0], point_s[0][1], z1],
              [point_s[1][0], point_s[1][1], z1], [point_s[1][0], point_s[1][1], z2],
              [point_s[2][0], point_s[2][1], z2], [point_s[2][0], point_s[2][1], z1],
              [point_s[3][0], point_s[3][1], z1], [point_s[3][0], point_s[3][1], z2]]
    return points