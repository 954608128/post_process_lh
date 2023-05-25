import math
import numpy as np

def Display_track(data, color_i):
    """画轨迹"""
    line_len = len(data)
    line_link = [[i, i + 1] for i in range(line_len - 1)]
    point = np.array(data)
    line_link = np.array(line_link)
    color = np.array([color_i for _ in range(len(line_link))])
    return point, line_link, color

#画数字，用于open3d显示ID，角度，类别等，用于调试
def Number_moudle_old(number, xyz, size=0.5):
    """0-9的字体, number:int(id), xyz:[x, y, z]"""
    number = str(number)
    wideth = size / 2  # 字与字间宽度
    org_color = [2, 2, 2]
    id_points, id_link, id_color = [], [], []
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
        else:
            num_point = np.zeros((2, 3))
            num_link = np.array([0, 1])
            num_color = np.array([0, 0, 0])
        # 画ID
        # ID_sets[i].points = o3d.utility.Vector3dVector(num_point)
        # ID_sets[i].lines = o3d.utility.Vector2iVector(num_link)
        # ID_sets[i].colors = o3d.utility.Vector3dVector(num_color)
        id_points.append(num_point)
        id_link.append(num_link)
        id_color.append(num_color)
    return id_points, id_link, id_color

def Number_moudle(number, xyz, size=1):
    """0-9的字体, number:int(id), xyz:[x, y, z]"""
    number = str(number)
    wideth = size / 2  # 字与字间宽度
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
    link_all = np.array(link_all)
    return point_all, link_all, color_all