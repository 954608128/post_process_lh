import copyreg
import math
import scipy.linalg as linalg
import numpy as np

PI=3.1415926535897932
ARC=6371004
# // 椭球体长半轴, 米
__A = 6378137
# // 椭球体短半轴, 米
# __B = 6356752.3142
__B = 6356725

# // 标准纬度, 弧度（取激光雷达所在纬度）
__B0=0.0
# // 原点经度, 弧度(0°)
__L0=0.0
# //反向转换程序中的迭代初始值
__IterativeValue=10

#旋转矩阵 欧拉角
def rotate_mat(axis, radian):
    rot_matrix = linalg.expm(np.cross(np.eye(3), axis / linalg.norm(axis) * radian))
    return rot_matrix

#角度转换成弧度
def FG_degree2rad(degree):
    return degree*PI/180.0

#弧度转角度
def FG_rad2degree(rad):
    return rad*180.0/PI

#经纬度转换成墨卡托坐标
def LonLat2Mercator(B, L):
    # f / * 扁率 * /, e / * 第一偏心率 * /, e_ / * 第二偏心率 * /, NB0 / * 卯酉圈曲率半径 * /, K, dtemp;
    # f, e, e_, NB0, K, dtemp=0
    # print('lonlat2mercator')
    f = 0.0
    e = 0.0
    e_ = 0.0
    NB0 = 0.0
    E = 0.0
    dtemp = 0.0
    E = float(math.exp(1))
    # E = math.exp(1)
    # print(f'B:{B},L:{L}')

    # print('1')
    __B0 = B
    __L0 = 0
    if  L < -PI or L > PI or B < -PI / 2 or B > PI / 2:
        # print('2')
        return False
    if __A <= 0 or __B <= 0:
        # print('3')
        return False
    f = (__A - __B) / __A

    dtemp = 1 - (__B / __A) * (__B / __A)
    if dtemp < 0:
        # print('4')
        return False
    # print(f'dtemp1-->{dtemp}')
    e = math.sqrt(dtemp)

    dtemp = (__A / __B) * (__A / __B) - 1
    if dtemp < 0:
        # print('5')
        return False
    # print('6')
    # print(f'dtemp2-->{dtemp}')
    e_ = math.sqrt(dtemp)
    NB0 = ((__A * __A) / __B) / math.sqrt(1 + e_ * e_ * math.cos(__B0) * math.cos(__B0))
    K = NB0 * math.cos(__B0)
    x = K * (L - __L0)
    y = K * math.log(math.tan(PI / 4 + B / 2) * math.pow((1 - e * math.sin(B)) / (1 + e * math.sin(B)), e / 2))
    # print(f'x:{x},y:{y}')
    # return 0
    # print(f'__B0:{__B0}')
    # print(f'L2M:__B0:{__B0},__L0{__L0}')
    return x,y
#墨卡托坐标转经纬度
def Mercator2LonLat(B,L,X, Y):
    # double f/*扁率*/, e/*第一偏心率*/, e_/*第二偏心率*/, NB0/*卯酉圈曲率半径*/, K, dtemp;
    # double E = exp(1);
    f = 0.0
    e = 0.0
    e_ = 0.0
    NB0 = 0.0
    E = 0.0
    dtemp = 0.0
    E = float(math.exp(1))
    __B0 = B
    __L0 = 0


    if __A <= 0 or __B <= 0:
        return False

    f = (__A - __B) / __A
    dtemp = 1 - (__B / __A) * (__B / __A)
    if dtemp < 0:
        return False
    e = math.sqrt(dtemp)

    dtemp = (__A / __B) * (__A / __B) - 1
    if dtemp < 0:
        return False
    e_ = math.sqrt(dtemp)
    NB0 = ((__A * __A) / __B) / math.sqrt(1 + e_ * e_ * math.cos(__B0) * math.cos(__B0))
    K = NB0 * math.cos(__B0)
    Object_Long = FG_rad2degree(Y / K + __L0)
    # print(f'__B0:{__B0}')
    B = 0.0
    for i in range(__IterativeValue):
        B = PI / 2 - 2 * math.atan(math.pow(E, (-X/K)) * math.pow(E, (e/2) * math.log((1-e * math.sin(B)) / (1 + e * math.sin(B)))))
    Object_Lat= FG_rad2degree(B)
    return Object_Long,Object_Lat

# /*
#   XYZ_To_BLH()：特征物体XY坐标转经纬度
#   输入：
#   WayPoint BLH_Origin: 激光雷达原始经纬度，角度
#   WayPoint XYZ_Move: 特征物体XY坐标，米
#   double rotaionangle： 激光雷达坐标系Y轴相对于正北方向夹角(0°～360°)顺时针
#   输出：
#   WayPoint *BLH_Move: 特征物体经纬度，角度
#  */
def XYZ_To_BLH(original_long,original_lat, move_x, move_y, rotaionangle):
    RadAngle =float(FG_degree2rad(rotaionangle))
    # Lat,Lng=0
    # Mercator_X, Mercator_Y=0
    # //激光器经纬度转墨卡托XY
    # WayPoint LiDAR_XYZ;
    # mer_x=0.0
    # mer_y=0.0
    mer_x, mer_y = LonLat2Mercator(FG_degree2rad(original_lat), FG_degree2rad(original_long))

    # //坐标轴旋转到正北方向,计算Move点墨卡托XY坐标
    # WayPoint Move;
    mer_move_x = move_x * math.cos(RadAngle) + move_y * math.sin(RadAngle) + mer_x
    mer_move_y = move_y * math.cos(RadAngle) - move_x * math.sin(RadAngle) + mer_y
    Object_Long,Object_Lat=Mercator2LonLat(FG_degree2rad(original_lat), FG_degree2rad(original_long), mer_move_y, mer_move_x)
    return Object_Long,Object_Lat