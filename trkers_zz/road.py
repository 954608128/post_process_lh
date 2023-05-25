import json
import utm
import numpy as np
import matplotlib.pyplot as plt
import cv2
from Lon_Lat_Meter_Transform import Lon_Lat_Meter_Transform

def Array_to_Image(road_xy, limit):
    """车道信息转换成图片"""
    image = np.ones((limit[0] * 10, limit[1] * 10, 3)) * 255
    image = np.array(image, dtype='uint8')
    radius = 2
    color = (50, 50, 50)
    for i in road_xy:
        try:
            x = round((i[0] + limit[0]) * 10)
            y = round((-i[1] + limit[0]) * 10)
            cv2.circle(image, (x, y), radius, color, thickness=-radius)
        except:
            continue
    # cv2.imwrite('map.jpg', image)
    # cv2.imshow('1', cv2.resize(image, (1000, 1000)))
    # cv2.waitKey(0)
    return image

def Json_Map(path):
    """读取矢量地图"""
    print('导入矢量地图')
    axis_limit = [-75, 75, -75, 75]    # x/y 限制
    limit = [max(abs(axis_limit[0]), abs(axis_limit[1])), max(abs(axis_limit[2]), abs(axis_limit[3]))]
    # 基站原始经纬度转UTM
    lat_0= 40.05169478
    lon_0= 116.28698024
    # angle_0 = 160  # 正北夹角
    angle_0 = 0  # 正北夹角

    lidar = Lon_Lat_Meter_Transform(lon_0, lat_0, angle_0)
    utm_0 = utm.from_latlon(lat_0, lon_0)
    x_0 = utm_0[0]
    y_0 = utm_0[1]
    try:
        with open(path, encoding='utf-8') as f_json:
            line = f_json.readline()
            d = json.loads(line)
            road_points = []
            for i in range(len(d['features'])):
                # temp_left, temp_right = [], []
                for ll in range(len(d['features'][i]['properties']['leftBoundaryLine']['geometry'])):
                    lat = d['features'][i]['properties']['leftBoundaryLine']['geometry'][ll]['lat']
                    lon = d['features'][i]['properties']['leftBoundaryLine']['geometry'][ll]['lng']
                    utm_ = utm.from_latlon(lat, lon)
                    x = utm_[0] - x_0
                    y = utm_[1] - y_0
                    if abs(x) <= limit[0] and abs(y) <= limit[1]:
                        road_points.append([lon, lat])
                for rr in range(len(d['features'][i]['properties']['rightBoundaryLine']['geometry'])):
                    lat = d['features'][i]['properties']['rightBoundaryLine']['geometry'][rr]['lat']
                    lon = d['features'][i]['properties']['rightBoundaryLine']['geometry'][rr]['lng']
                    utm_ = utm.from_latlon(lat, lon)
                    x = utm_[0] - x_0
                    y = utm_[1] - y_0
                    if abs(x) <= limit[0] and abs(y) <= limit[1]:
                        road_points.append([lon, lat])
                # left_points.append(temp_left)
                # right_points.append(temp_right)
            f_json.close()
            road_point = np.array(road_points)
            road_xy = lidar.lonlat_to_xy_of_draw(road_point)
            print('地图导入成功')
    except:
        print('导入车道矢量图失败')
        road_xy = np.zeros((3,2))

    # fig = plt.figure()
    # ax = plt.axes()
    # for j in road_xy:
    #     plt.plot(j[0] * 10, j[1] * 10, color=[0.5, 0.5, 0.5], marker='.', markersize='1')
    # plt.axis([-limit[0] * 10, limit[0] * 10, -limit[1] * 10, limit[1] * 10])
    # plt.axis('off')
    # ax.spines['top'].set_visible(False)
    # ax.spines['right'].set_visible(False)
    # ax.spines['bottom'].set_visible(False)
    # ax.spines['left'].set_visible(False)
    # fig.tight_layout()
    # plt.savefig('map.jpg', dpi=500)

    map = Array_to_Image(road_xy, limit)
    road = road_xy.tolist()
    road2 = []
    for i in range(len(road)):
        road1 = []
        road1.append(road[i][0])
        road1.append(road[i][1])
        road1.append(-5)
        road2.append(road1)
    np.savetxt("./trkers/data_save/road.csv", road2, delimiter=",")

if __name__=='__main__':

    Json_Map(r'/home/wanji/Documents/wjv4/trkers/data_save/the_8.json')

