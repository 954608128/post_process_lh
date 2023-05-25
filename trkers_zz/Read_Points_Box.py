import open3d as o3d
import os
import numpy as np
import csv


class Box_label:
    def __init__(self, i):
        # self.class_fs = i[1]
        # self.x_fs = i[2] / 100
        # self.y_fs = i[3] / 100
        # self.z_fs = i[4] / 100
        # self.length = i[7] / 100
        # self.width = i[8] / 100
        # self.height = i[9] / 100
        # self.angle = i[6]
        self.class_fs = i[1]
        self.x_fs = i[2] / 100
        self.y_fs = i[3] / 100
        self.z_fs = i[4] / 100
        self.length = i[5] / 100
        self.width = i[6] / 100
        self.height = i[7] / 100
        self.angle = i[8]

        self.box_point = self.xyz_to_8points([self.x_fs, self.y_fs, self.z_fs],
                                             [self.width, self.length, self.height],
                                             self.angle)
        self.box_link = np.array([[0, 1], [1, 2], [0, 3], [2, 3], [4, 5], [4, 7],
                                  [5, 6], [6, 7], [0, 4], [1, 5], [2, 6], [3, 7]])
        car_head = [2, 0, 0]
        car_body = [0, 2, 0]
        self.box_color = np.array([car_head, car_body, car_body, car_body, car_head, car_body,
                                   car_body, car_body, car_head, car_head, car_body, car_body])

    def xyz_to_8points(self, point_xyz, size, angle):
        """中心点xyz与宽长高，输出8角点xyz"""
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


def Number_moudle(number, xyz, size = 1):
    """0-9的字体, number:int(id), xyz:[x, y, z]"""
    id_all = []
    number = str(number)
    wideth = size / 2    # 字与字间宽度
    org_color = [2, 2, 2]
    for i in range(len(number)):
        id_box = o3d.geometry.LineSet()
        x = xyz[0] + size * i + wideth * i
        y = xyz[1]
        z = xyz[2]
        num = number[i]
        num_points = [[0 + x, 0 + y, 0 + z],
                      [1 * size + x, 0 + y, 0 + z],
                      [0 + x, 1 * size + y, 0 + z],
                      [1 * size + x, 1 * size + y, 0 + z],
                      [0 + x, 2 * size + y, 0 + z],
                      [1 * size + x, 2 * size + y, 0 + z]]
        if num == '0':
            num_point = np.array([num_points[0], num_points[1], num_points[5], num_points[4]])
            num_link = np.array([[0, 1], [1, 2], [2, 3], [3, 0]])
            num_color = np.array([org_color for _ in range(len(num_link))])
        elif num == '1':
            num_point = np.array([[0.5 * size + x, 0 + y, 0 + z], [0.5 * size + x, 2 * size + y, 0 + z]])
            num_link = np.array([[0, 1]])
            num_color = np.array([org_color for _ in range(len(num_link))])
        elif num == '2':
            num_point = np.array(num_points)
            num_link = np.array([[4, 5], [5, 3], [3, 2], [2, 0], [0, 1]])
            num_color = np.array([org_color for _ in range(len(num_link))])
        elif num == '3':
            num_point = np.array(num_points)
            num_link = np.array([[0, 1], [1, 5], [2, 3], [4, 5]])
            num_color = np.array([org_color for _ in range(len(num_link))])
        elif num == '4':
            num_point = np.array(num_points[1:])
            num_link = np.array([[0, 4], [1, 2], [1, 3]])
            num_color = np.array([org_color for _ in range(len(num_link))])
        elif num == '5':
            num_point = np.array(num_points)
            num_link = np.array([[0, 1], [1, 3], [3, 2], [2, 4], [4, 5]])
            num_color = np.array([org_color for _ in range(len(num_link))])
        elif num == '6':
            num_point = np.array(num_points)
            num_link = np.array([[0, 1], [0, 4], [1, 3], [2, 3], [4, 5]])
            num_color = np.array([org_color for _ in range(len(num_link))])
        elif num == '7':
            num_point = np.array([num_points[1], num_points[4], num_points[5]])
            num_link = np.array([[0, 2], [1, 2]])
            num_color = np.array([org_color for _ in range(len(num_link))])
        elif num == '8':
            num_point = np.array(num_points)
            num_link = np.array([[0, 4], [0, 1], [1, 5], [4, 5], [2, 3]])
            num_color = np.array([org_color for _ in range(len(num_link))])
        elif num == '9':
            num_point = np.array(num_points)
            num_link = np.array([[0, 1], [1, 5], [2, 3], [2, 4], [4, 5]])
            num_color = np.array([org_color for _ in range(len(num_link))])
        else:
            num_point = np.zeros((2, 3))
            num_link = np.array([0, 1])
            num_color = np.array([0, 0, 0])
        # 画ID
        id_box.points = o3d.utility.Vector3dVector(num_point)
        id_box.lines = o3d.utility.Vector2iVector(num_link)
        id_box.colors = o3d.utility.Vector3dVector(num_color)
        id_all.append(id_box)
    return id_all


def plot_map_line():
    """-100~100，每隔10m，画地图方格线"""
    num_line = 0
    height = -6
    map_lines_box = np.array([[0, 1]])
    axis_limit = [-80, 40, -30, 50]
    step = 5
    long = ((axis_limit[1] - axis_limit[0]) / step + 1) + ((axis_limit[3] - axis_limit[2]) / step + 1)
    map_line = [o3d.geometry.LineSet() for _ in range(int(long))]
    color = np.array([[0.3, 0.3, 0.3] for _ in range(len(map_lines_box))])
    for x in range(axis_limit[0], axis_limit[1] + step, step):
        points_box = [[x, axis_limit[2], height], [x, axis_limit[3], height]]
        map_line[num_line].points = o3d.utility.Vector3dVector(points_box)
        map_line[num_line].lines = o3d.utility.Vector2iVector(map_lines_box)
        map_line[num_line].colors = o3d.utility.Vector3dVector(color)
        num_line += 1
    for y in range(axis_limit[2], axis_limit[3] + step, step):
        points_box = [[axis_limit[0], y, height], [axis_limit[1], y, height]]
        map_line[num_line].points = o3d.utility.Vector3dVector(points_box)
        map_line[num_line].lines = o3d.utility.Vector2iVector(map_lines_box)
        map_line[num_line].colors = o3d.utility.Vector3dVector(color)
        num_line += 1
    return map_line


def Read_csv(path):
    """读csv文件"""
    csv_read = csv.reader(open(path, encoding='utf_8_sig'))
    data0 = []
    for line in csv_read:
        data0.append(line)
    # str to float
    data = []
    for i in data0:
        temp = [float(x) for x in i]
        data.append(temp)
    return data


def Read_menu(path1, path2):
    """读文件夹，配对检测结果与点云点，输出对应路径"""
    path_pcd = path1
    path_csv = path2
    menu_pcd = os.listdir(path_pcd)
    menu_csv = os.listdir(path_csv)
    menu_pcd = sorted([x for x in menu_pcd if x[-4:] == '.pcd'], key=(lambda f: float(f[:-4])))
    menu_csv = sorted([x for x in menu_csv if x[-4:] == '.csv'], key=(lambda f: float(f[:-4])))
    couple = {}
    for i in menu_csv:
        temp = i[:-4] + '.pcd'
        if temp in menu_pcd:
            couple.update({os.path.join(path2, i) : os.path.join(path1, temp)})
    return couple


def Draw_points_box(Points, Box, ID):
    """画"""
    view_window = o3d.visualization.Visualizer()
    view_window.create_window()
    view_window.add_geometry(Points)
    map_line = plot_map_line()
    for i in Box:
        view_window.add_geometry(i)
    for j in ID:
        view_window.add_geometry(j)
    for k in map_line:
        view_window.add_geometry(k)
    render_option = view_window.get_render_option()
    render_option.point_size = 1
    render_option.background_color = np.asarray([0, 0, 0])
    view_window.run()
    view_window.destroy_window()


def main(path1, path2):
    path_couple = Read_menu(path1, path2)
    count = 0
    for path_csv, path_pcd in path_couple.items():
        if count % read_interval == 0:
            num = 1
            print('**********************************')
            print(path_pcd)
            print(path_csv)
            print('**********************************')
            point = np.asarray(o3d.io.read_point_cloud(path_pcd).points)
            box = Read_csv(path_csv)
            Points = o3d.geometry.PointCloud()
            Points.points = o3d.utility.Vector3dVector(point)
            Box = [o3d.geometry.LineSet() for _ in range(len(box))]
            ID = []
            for i in range(len(box)):
                data_box = Box_label(box[i])
                Box[i].points = o3d.utility.Vector3dVector(data_box.box_point)
                Box[i].lines = o3d.utility.Vector2iVector(data_box.box_link)
                Box[i].colors = o3d.utility.Vector3dVector(data_box.box_color)
                id_box = Number_moudle(int(data_box.class_fs),
                                       [data_box.x_fs - 1, data_box.y_fs + 1, data_box.z_fs + 1],
                                       size=0.5)
                ID.extend(id_box)
                num = num + 1
            Draw_points_box(Points, Box, ID)
        count = count + 1


if __name__ == '__main__':
    """
       可视化点云检测框与点云点
    """
    path1 = '/media/wanji/data1/detection_result1110/result/20210825105459/Radar'    # points
    path2 = '/home/wanji/Documents/wjv4/test/trkers/data_save/20210825105459_result'    # label
    read_interval = 10
    main(path1, path2)


