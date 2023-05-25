import open3d as o3d
import numpy as np
############## by DuWen #################

#########################################

class Open3D_View:
    """
       open3d画图模组,
       design by DuWen
    """
    def __init__(self):
        self.total_num = 500                      # 画框数量
        self.num_num_all = self.total_num         # 画ID数量
        self.num_o3d = 0                          # 检测框计数
        self.num_num = 0                          # ID计数器
        self.create_o3d()                         # 图窗，点云，连线
        self.set_line_connect()                   # 8角点连接顺序
        # 扫描线信息
        self.detection_range = 100                # 扫描线最大半径
        self.theta = np.arange(0, 2 * np.pi, 0.1)
        self.scan_line_color = np.array([[0.6, 0.6, 0.6] for _ in range(len(self.theta))])
        self.scan_line_link = [[i, i + 1] for i in range(len(self.theta) - 1)]
        self.scan_line_link.append([0, len(self.theta) - 1])
        self.scan_line_link = np.array(self.scan_line_link)
        # 车道线
        try:
            xxx = np.load('lane_line_channel.npy')
            self.point_cloud.points = o3d.utility.Vector3dVector(xxx)
            point_color = np.array([[0.2, 0.2, 0.2] for _ in range(len(xxx))])
            self.point_cloud.colors = o3d.utility.Vector3dVector(point_color)
        except:
            self.point_cloud.points = o3d.utility.Vector3dVector(np.array([[0, 0, 0]]))

    def create_o3d(self):
        """创建一个open3d图框，及图内点线基本设定"""
        self.view_window = o3d.visualization.Visualizer()
        self.view_window.create_window(window_name='view',
                                  width=1080,
                                  height=720,
                                  left=300,
                                  top=150,
                                  visible=True)
        self.render_option = self.view_window.get_render_option()
        self.render_option.point_size = 0.5
        self.render_option.background_color = np.asarray([0, 0, 0])
        self.point_cloud = o3d.geometry.PointCloud()
        self.view_window.add_geometry(self.point_cloud)
        self.to_reser_view_point = True
        self.line_sets = [o3d.geometry.LineSet() for _ in range(self.total_num)]
        for i in self.line_sets:
            self.view_window.add_geometry(i)
        # 每隔xxm画地图网格
        self.plot_map_line(self.view_window)
        # 基站的扫描线
        self.scan_line = o3d.geometry.LineSet()
        self.view_window.add_geometry(self.scan_line)
        # ID字
        # self.id_box = [o3d.geometry.LineSet() for _ in range(self.num_num_all)]
        # for i in self.id_box:
        #     self.view_window.add_geometry(i)

        # 画区域
        # blind_zone1 = [25, 95, -60, 20]
        # blind_point1 = np.array([[blind_zone1[0], blind_zone1[2], 0], [blind_zone1[0], blind_zone1[3], 0],
        #                          [blind_zone1[1], blind_zone1[2], 0], [blind_zone1[1], blind_zone1[3], 0]])
        # blind_link1 = np.array([[0, 1], [2, 3]])
        # blind_color1 = np.array([[1, 0, 0] for _ in range(len(blind_link1))])
        # blind_line = o3d.geometry.LineSet()
        # self.view_window.add_geometry(blind_line)
        # blind_line.points = o3d.utility.Vector3dVector(blind_point1)
        # blind_line.lines = o3d.utility.Vector2iVector(blind_link1)
        # blind_line.colors = o3d.utility.Vector3dVector(blind_color1)

        test_zone = [-228, 165, -104, 50]
        test_point1 = np.array([[test_zone[0], test_zone[2], 0], [test_zone[0], test_zone[3], 0],
                                [test_zone[1], test_zone[2], 0], [test_zone[1], test_zone[3], 0]])
        test_link1 = np.array([[0, 1], [1, 3], [2, 3], [0, 2]])
        test_color1 = np.array([[0, 2, 0] for _ in range(len(test_link1))])
        test_line = o3d.geometry.LineSet()
        self.view_window.add_geometry(test_line)
        test_line.points = o3d.utility.Vector3dVector(test_point1)
        test_line.lines = o3d.utility.Vector2iVector(test_link1)
        test_line.colors = o3d.utility.Vector3dVector(test_color1)

        # 显示区域，用于调试
        region_list = [[[22.052, 16.91, -3], [22.2129, 2.78, -3], [97.45, 17.02, -3], [96.6, 2.39, -3]],  # 右边上
                       [[-48.99, 1.75, -3], [-51.2, -17.3, -3], [-21.99, 2.2, -3], [-19.45, -18.83, -3]],  # 左边下
                       [[-13.677, -31.6, -3], [-17.24, -76.63, -3], [11.2, -33.97, -3], [9.52, -79.39, -3]]]  # 下边
        line_set2 = o3d.geometry.LineSet()
        self.view_window.add_geometry(line_set2)
        triangle_points2 = np.array([region_list[0][1], region_list[0][0], region_list[0][3], region_list[0][2]],
                                    dtype=np.float32)
        lines = [[0, 1], [2, 3], [0, 2], [1, 3]]
        colors = [[1, 0, 0] for i in range(len(lines))]
        line_set2.points = o3d.utility.Vector3dVector(triangle_points2)
        line_set2.lines = o3d.utility.Vector2iVector(lines)
        line_set2.colors = o3d.utility.Vector3dVector(colors)

        line_set3 = o3d.geometry.LineSet()
        self.view_window.add_geometry(line_set3)
        triangle_points3 = np.array([region_list[1][1], region_list[1][0], region_list[1][3], region_list[1][2]],
                                    dtype=np.float32)
        lines = [[0, 1], [2, 3], [0, 2], [1, 3]]
        colors = [[1, 0, 1] for i in range(len(lines))]
        line_set3.points = o3d.utility.Vector3dVector(triangle_points3)
        line_set3.lines = o3d.utility.Vector2iVector(lines)
        line_set3.colors = o3d.utility.Vector3dVector(colors)

        line_set4 = o3d.geometry.LineSet()
        self.view_window.add_geometry(line_set4)
        triangle_points4 = np.array([region_list[2][1], region_list[2][0], region_list[2][3], region_list[2][2]],
                                    dtype=np.float32)
        lines = [[0, 1], [2, 3], [0, 2], [1, 3]]
        colors = [[0, 1, 0] for i in range(len(lines))]
        line_set4.points = o3d.utility.Vector3dVector(triangle_points4)
        line_set4.lines = o3d.utility.Vector2iVector(lines)
        line_set4.colors = o3d.utility.Vector3dVector(colors)

    def Reset(self):
        """重置计数器"""
        self.num_o3d = 0
        self.num_num = 0
        # 检测框定位
        # self.box_location()

    def plot_map_line(self, view_window):
        """-100~100，每隔10m，画地图方格线"""
        height = -6
        self.axis_limit = [-250, 200, -150, 100]
        step = 10
        self.map_line = o3d.geometry.LineSet()
        view_window.add_geometry(self.map_line)
        count = 0
        map_point = []
        map_link = []
        map_color = []
        for x in range(self.axis_limit[0], self.axis_limit[1] + step, step):
            points_box = [[x, self.axis_limit[2], height], [x, self.axis_limit[3], height]]
            map_point.extend(points_box)
            map_link.extend([[count, count + 1]])
            if x % (step * 5) == 0:
                color = [0.5, 0.5, 0.5]
            else:
                color = [0.25, 0.25, 0.25]
            map_color.append(color)
            count += 2
        for y in range(self.axis_limit[2], self.axis_limit[3] + step, step):
            points_box = [[self.axis_limit[0], y, height], [self.axis_limit[1], y, height]]
            map_point.extend(points_box)
            map_link.extend([[count, count + 1]])
            if y % (step * 5) == 0:
                color = [0.5, 0.5, 0.5]
            else:
                color = [0.25, 0.25, 0.25]
            map_color.append(color)
            count += 2
        self.map_line.points = o3d.utility.Vector3dVector(np.array(map_point))
        self.map_line.lines = o3d.utility.Vector2iVector(np.array(map_link))
        self.map_line.colors = o3d.utility.Vector3dVector(np.array(map_color))

    def set_line_connect(self):
        """检测框中连线方式"""
        self.lines_box = np.array([[0, 1], [1, 2], [0, 3], [2, 3], [4, 5], [4, 7], [5, 6],
                                   [6, 7], [0, 4], [1, 5], [2, 6], [3, 7], [0, 5], [1, 4]])
        # self.lines_box = np.array([[0, 1], [1, 2], [0, 3], [2, 3], [4, 5], [4, 7], [5, 6],
        #                            [6, 7], [0, 4], [1, 5], [2, 6], [3, 7]])

    def o3d_update(self):
        """多余框归零刷新,更新每帧open3d图窗显示"""
        # 多余框归零刷新
        for j_zero in range(self.num_o3d, self.total_num):
            points_box_zero = np.zeros((8, 3))
            color_zero = np.array([[0, 0, 0] for _ in range(len(self.lines_box))])
            self.line_sets[j_zero].points = o3d.utility.Vector3dVector(points_box_zero)
            self.line_sets[j_zero].lines = o3d.utility.Vector2iVector(self.lines_box)
            self.line_sets[j_zero].colors = o3d.utility.Vector3dVector(color_zero)
        # for j_zero in range(self.num_num, self.num_num_all):
        #     points_box_zero = np.zeros((8, 3))
        #     color_zero = np.array([[0, 0, 0] for _ in range(len(self.lines_box))])
        #     self.id_box[j_zero].points = o3d.utility.Vector3dVector(points_box_zero)
        #     self.id_box[j_zero].lines = o3d.utility.Vector2iVector(self.lines_box)
        #     self.id_box[j_zero].colors = o3d.utility.Vector3dVector(color_zero)
        # 更新每帧open3d图窗显示
        ###########################################
        # 0.12.0以下版本
        self.view_window.update_geometry()
        # 0.12.0版本
        # self.view_window.update_geometry(self.point_cloud)
        # for i in self.map_line:
        #     self.view_window.update_geometry(i)
        # for i in self.line_sets:
        #     self.view_window.update_geometry(i)
        # # for i in self.scan_line:
        # #     self.view_window.update_geometry(i)
        # for i in self.id_box:
        #     self.view_window.update_geometry(i)
        ############################################
        self.view_window.poll_events()
        self.view_window.update_renderer()
        if self.to_reser_view_point:
            self.view_window.reset_view_point(True)
            self.to_reser_view_point = False

    def Display_object(self, data):
        """open3d画目标"""
        car_head = [2, 0, 0]
        car_body = data.color
        color = np.array([car_head, car_body, car_body, car_body, car_head, car_body, car_body,
                          car_body, car_head, car_head, car_body, car_body, car_head, car_head])

        # color = np.array([[2, 0, 0] for _ in range(12)])
        # self.line_sets[self.num_o3d].points = o3d.utility.Vector3dVector(data.box)
        # self.line_sets[self.num_o3d].lines = o3d.utility.Vector2iVector(self.lines_box)
        # self.line_sets[self.num_o3d].colors = o3d.utility.Vector3dVector(color)
        # self.num_o3d += 1
        # # 画目标ID
        # try:
        #     self.Number_moudle(data.id_fs, [data.x - 2, data.y + 2, data.z + 1], size=1)
        # except:
        #     pass

        box_point = list(data.box)
        box_link = self.lines_box
        box_color = color
        # 画目标ID
        try:
            box_point, box_link, box_color = self.Number_moudle(box_point, box_link, box_color,
                                                                data.YC, [data.x - 2, data.y + 2, data.z + 1],
                                                                size=1)
        except:
            pass
        self.line_sets[self.num_o3d].points = o3d.utility.Vector3dVector(box_point)
        self.line_sets[self.num_o3d].lines = o3d.utility.Vector2iVector(box_link)
        self.line_sets[self.num_o3d].colors = o3d.utility.Vector3dVector(box_color)
        self.num_o3d += 1

    def Display_track(self, data, color):
        """画轨迹"""
        try:
            line_len = len(data)
            line_link = [[i, i + 1] for i in range(line_len - 1)]
            point = np.array(data)
            line_link = np.array(line_link)
            color = np.array([color for _ in range(len(line_link))])
            if len(line_link) != 0:
                self.line_sets[self.num_o3d].points = o3d.utility.Vector3dVector(point)
                self.line_sets[self.num_o3d].lines = o3d.utility.Vector2iVector(line_link)
                self.line_sets[self.num_o3d].colors = o3d.utility.Vector3dVector(color)
                self.num_o3d += 1
        except:
            pass

    def Scan_lines_lidar(self, frame):
        """基站的扫描线"""
        self.lidar_xyz = [[0, 0, 0]]
        radius = self.detection_range * ((frame % 20) / 20)
        data_len = len(self.theta)
        points = []
        z = 0
        for i in range(data_len):
            x = radius * np.cos(self.theta[i])
            y = radius * np.sin(self.theta[i])
            points.append([x, y, z])
        points = np.array(points)
        self.scan_line.points = o3d.utility.Vector3dVector(points)
        self.scan_line.lines = o3d.utility.Vector2iVector(self.scan_line_link)
        self.scan_line.colors = o3d.utility.Vector3dVector(self.scan_line_color)
        # # 写基站标号
        # self.Number_moudle(num + 1,
        #                    [self.lidar_xyz[num][0] - 10,
        #                     self.lidar_xyz[num][1] - 15,
        #                     self.lidar_xyz[num][2]],
        #                    size=5)

    def Number_moudle(self, box_point, box_link, box_color, number, xyz, size = 1):
        """0-9的字体, number:int(id), xyz:[x, y, z]"""
        number = str(number)
        wideth = size / 2    # 字与字间宽度
        org_color = [2, 2, 2]
        point_all, link_all, color_all = [], [], []
        for i in range(len(number)):
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
            elif num == '_' or num == '.':
                num_point = np.array([num_points[0], num_points[1]])
                num_link = np.array([[0, 1]])
                num_color = np.array([org_color for _ in range(len(num_link))])
            elif num == '-':
                num_point = np.array([num_points[2], num_points[3]])
                num_link = np.array([[0, 1]])
                num_color = np.array([org_color for _ in range(len(num_link))])
            elif num == '/':
                num_point = np.array([num_points[0], num_points[5]])
                num_link = np.array([[0, 1]])
                num_color = np.array([org_color for _ in range(len(num_link))])
            elif num == '[':
                num_point = np.array([num_points[0], num_points[1], num_points[4], num_points[5]])
                num_link = np.array([[0, 1], [2, 3], [0, 2]])
                num_color = np.array([org_color for _ in range(len(num_link))])
            elif num == ']':
                num_point = np.array([num_points[0], num_points[1], num_points[4], num_points[5]])
                num_link = np.array([[0, 1], [2, 3], [1, 3]])
                num_color = np.array([org_color for _ in range(len(num_link))])
            else:
                num_point = np.zeros((2, 3))
                num_link = np.array([0, 1])
                num_color = np.array([0, 0, 0])
            num_link = num_link + len(point_all)
            link_all.extend(list(num_link))
            point_all.extend(list(num_point))
        # ID数字汇总
        point_all = np.array(point_all)
        color_all = np.array([org_color for _ in range(len(link_all))])
        link_all = np.array(link_all) + len(box_point)
        # 画ID
        # self.id_box[self.num_num].points = o3d.utility.Vector3dVector(point_all)
        # self.id_box[self.num_num].lines = o3d.utility.Vector2iVector(link_all)
        # self.id_box[self.num_num].colors = o3d.utility.Vector3dVector(color_all)
        # self.num_num += 1
        box_point = np.concatenate((box_point, point_all))
        box_link = np.concatenate((box_link, link_all))
        box_color = np.concatenate((box_color, color_all))
        return box_point, box_link, box_color

    def box_location(self):
        """四角定位"""
        corner = [[self.axis_limit[0], self.axis_limit[2]],
                  [self.axis_limit[0], self.axis_limit[3]],
                  [self.axis_limit[1], self.axis_limit[2]],
                  [self.axis_limit[1], self.axis_limit[3]]]
        long = 2
        for i in range(4):
            box_center = corner[i]
            box = [[box_center[0] - long, box_center[1] - long, -long],
                   [box_center[0] - long, box_center[1] - long, +long],
                   [box_center[0] + long, box_center[1] - long, +long],
                   [box_center[0] + long, box_center[1] - long, -long],
                   [box_center[0] - long, box_center[1] + long, -long],
                   [box_center[0] - long, box_center[1] + long, +long],
                   [box_center[0] + long, box_center[1] + long, +long],
                   [box_center[0] + long, box_center[1] + long, -long]]
            color = np.array([[0, 0, 0] for _ in range(12)])
            self.line_sets[i].points = o3d.utility.Vector3dVector(box)
            self.line_sets[i].lines = o3d.utility.Vector2iVector(self.lines_box)
            self.line_sets[i].colors = o3d.utility.Vector3dVector(color)

    def Point_init(self, data):
        """点云设定"""
        data_point = np.array(data)
        if len(data_point) == 0:
            data_point = np.array([[0, 0, 0]])
        self.point_cloud.points = o3d.utility.Vector3dVector(data_point)

if __name__ == "__main__":
    Open3D_View()