import math

import matplotlib.pyplot as plt
import numpy as np
import time
import os
# plt.figure()
# ID_x = np.loadtxt("/home/wanji/Documents/wjv4/second.pytorch/save_result/lines/lines137.0x.csv")
# y_plot = np.loadtxt("/home/wanji/Documents/wjv4/second.pytorch/save_result/lines/lines137.0y_.csv")
# y1 = np.loadtxt("/home/wanji/Documents/wjv4/second.pytorch/save_result/lines/lines137.0y.csv")
#
# # plt.plot(ID_x, y_plot, color='teal', linewidth=2, label="degree %d" % 5)
# plt.scatter(ID_x, y1, marker = 'x', color = 'red', s = 40, label = 'First')
# plt.legend(loc='lower left')
# plt.grid(True)
#
# plt.figure()

# #单帧修正
# def Curve_fiting(fram_list, angle_list, number, i):
#     z1 = np.polyfit(fram_list, angle_list, 20)  # 用4次多项式拟合
#     p1 = np.poly1d(z1)
#     frame_x = p1(fram_list)
#     plt.scatter(fram_list, angle_list,  marker = '*', color = 'teal', s = 40, label = 'First')
#     plt.plot(fram_list, frame_x, 'r', label='polyfit values', linewidth=2)
#     plt.xlabel('x axis')
#     plt.ylabel('y axis')
#     plt.legend(loc=4) # 指定legend的位置,读者可以自己help它的用法
#     plt.title('1')
#     save_path = "/home/wanji/Documents/wjv4/second.pytorch/save_result/lines_pictures/angle/" + str(number) + "/"
#     if os.path.exists(save_path):
#         path = save_path + str(i) + "fram_angle_c.jpg"
#         # save_path = "/home/wanji/Documents/wjv4/second.pytorch/save_result/lines_pictures/angle/" + str(number) + "_" + str(i) + "fram_angle_c.jpg"
#         plt.savefig(path)
#         plt.clf()
#     else:
#         os.makedirs(save_path)
#         path = save_path + str(i) + "fram_angle_c.jpg"
#         # save_path = "/home/wanji/Documents/wjv4/second.pytorch/save_result/lines_pictures/angle/" + str(number) + "_" + str(i) + "fram_angle_c.jpg"
#         plt.savefig(path)
#         plt.clf()
#
#     for i in range(fram_list.size):
#         if abs(angle_list[i] - p1(fram_list[i])) > 0.2:
#             angle_list[i] = p1(fram_list[i])
#     return fram_list, angle_list
#
#
#
# number = 1
# while True:
#     plt.figure()
#     path1 = "/home/wanji/Documents/wjv4/second.pytorch/save_result/lines_pictures/" + str(number) +".0angle.csv"
#     path2 = "/home/wanji/Documents/wjv4/second.pytorch/save_result/lines_pictures/" + str(number) +".0frame_plot.csv"
#
#     path3 = "/home/wanji/Documents/wjv4/second.pytorch/save_result/lines_pictures/" + str(number) + ".0x.csv"
#     path4 = "/home/wanji/Documents/wjv4/second.pytorch/save_result/lines_pictures/" + str(number) +".0y.csv"
#
#     if os.path.exists(path1):
#         x_pose = np.loadtxt(path3)
#         y_pose = np.loadtxt(path4)
#         if x_pose.size < 10:
#             number += 1
#             continue
#         distance = math.sqrt((x_pose[0] - x_pose[len(x_pose) - 1]) * (x_pose[0] - x_pose[len(x_pose) - 1]) + (y_pose[0] - y_pose[len(x_pose) - 1]) * (y_pose[0] - y_pose[len(y_pose) - 1]))
#         if distance < 10:
#             number += 1
#             continue
#
#         frame = np.loadtxt(path2)
#         y = np.loadtxt(path1)
#
#         # z1 = np.polyfit(frame, y, 20) # 用4次多项式拟合
#         # p1 = np.poly1d(z1)
#         # yvals = p1(frame) # 也可以使用yvals=np.polyval(z1,x)
#         #
#         # plt.plot(frame, y, '*', label='original values')
#         # plt.plot(frame, yvals, 'r', label='polyfit values')
#         # plt.xlabel('x axis')
#         # plt.ylabel('y axis')
#         # plt.legend(loc=4) # 指定legend的位置,读者可以自己help它的用法
#         # plt.title('1')
#         # save_path = "/home/wanji/Documents/wjv4/second.pytorch/save_result/lines_pictures/angle/" + str(number) + "fram_angle.jpg"
#         # plt.savefig(save_path)
#         for i in range(3):
#             frame, y = Curve_fiting(frame, y, number, i)
#         number += 1
#
#     else:
#         number += 1


#轨迹修正测试
def Curve_fiting(fram_list, x_list, y_list, angle_list, number, number_):
    z1 = np.polyfit(fram_list, x_list, 20)  # 用4次多项式拟合
    p1 = np.poly1d(z1)
    frame_x = p1(fram_list)
    plt.plot(fram_list, x_list, '*', label='original values')
    plt.plot(fram_list, frame_x, 'r',label='polyfit values')
    plt.xlabel('x axis')
    plt.ylabel('y axis')
    plt.legend(loc=4) # 指定legend的位置,读者可以自己help它的用法
    plt.title('1')
    save_path = "/home/wanji/Documents/wjv4/test/lines_pictures/test/x/"
    if os.path.exists(save_path):
        save_path = save_path + str(number_) + "_" + str(number) + "fram_x.jpg"
        plt.savefig(save_path)
        plt.clf()
    else:
        os.makedirs(save_path)
        save_path = save_path + str(number_) + "_" + str(number) + "fram_x.jpg"
        plt.savefig(save_path)
        plt.clf()

    z2 = np.polyfit(fram_list, y_list, 20)  # 用4次多项式拟合
    p2 = np.poly1d(z2)
    frame_y = p2(fram_list)
    plt.plot(fram_list, y_list, '*', label='original values')
    plt.plot(fram_list, frame_y, 'r', label='polyfit values')
    plt.xlabel('x axis')
    plt.ylabel('y axis')
    plt.legend(loc=4)  # 指定legend的位置,读者可以自己help它的用法
    plt.title('2')
    save_path = "/home/wanji/Documents/wjv4/test/lines_pictures/test/y/"
    if os.path.exists(save_path):
        save_path = save_path + str(number_) + "_" + str(number) + "fram_y.jpg"
        plt.savefig(save_path)
        plt.clf()
    else:
        os.makedirs(save_path)
        save_path = save_path + str(number_) + "_" + str(number) + "fram_y.jpg"
        plt.savefig(save_path)
        plt.clf()

    z3 = np.polyfit(x_list, y_list, 20)  # 用4次多项式拟合
    p3 = np.poly1d(z3)
    frame_y_ = p3(x_list)
    plt.plot(x_list, y_list, '*', label='original values')
    plt.plot(x_list, frame_y_, 'r', label='polyfit values')
    plt.xlabel('x axis')
    plt.ylabel('y axis')
    plt.legend(loc=4)  # 指定legend的位置,读者可以自己help它的用法
    plt.title('2')
    save_path = "/home/wanji/Documents/wjv4/test/lines_pictures/test/xy/"
    if os.path.exists(save_path):
        save_path = save_path + str(number_) + "_" + str(number) + "x_y.jpg"
        plt.savefig(save_path)
        plt.clf()
    else:
        os.makedirs(save_path)
        save_path = save_path + str(number_) + "_" + str(number) + "x_y.jpg"
        plt.savefig(save_path)
        plt.clf()


    for i in range(fram_list.size):
        if abs(x_list[i] - p1(fram_list[i])) > 0.5:
            x_list[i] = p1(fram_list[i])
        if abs(y_list[i] - p2(fram_list[i])) > 0.5:
            y_list[i] = p2(fram_list[i])
        if abs(angle_list[i] - p3(fram_list[i])) > 0.5:
            angle_list[i] = p3(fram_list[i])
    return x_list, y_list, angle_list


if __name__=='__main__':
    number = 0
    while True:
        path2 = r"/home/wanji/Documents/wjv4/test/lines_pictures/" + str(number) + "x.csv"
        path3 = r"/home/wanji/Documents/wjv4/test/lines_pictures/" + str(number) +"y.csv"
        path1 = r"/home/wanji/Documents/wjv4/test/lines_pictures/" + str(number) +"frame_plot.csv"
        path4 = r"/home/wanji/Documents/wjv4/test/lines_pictures/" + str(number) + ".0angle.csv"
        number += 1
        key = os.path.exists(path1)
        if os.path.exists(path1):
            frame = np.loadtxt(path1)
            y = np.loadtxt(path2)
            x = np.loadtxt(path3)
            angle = np.loadtxt(path4)
            frame_ = frame
            x_ = x
            y_ = y
            angle_ = angle
            for i in range(2):
                x_, y_, angle_ = Curve_fiting(frame_, x_, y_, angle_, i, number)
            print("done!")


# number = 1
# while True:
#     plt.figure()
#     path1 = "/home/wanji/Documents/wjv4/second.pytorch/save_result/lines_pictures/" + str(number) +".0x.csv"
#     path2 = "/home/wanji/Documents/wjv4/second.pytorch/save_result/lines_pictures/" + str(number) +".0plot.csv"
#     path3 = "/home/wanji/Documents/wjv4/second.pytorch/save_result/lines_pictures/" + str(number) +".0y.csv"
#     path4 = "/home/wanji/Documents/wjv4/second.pytorch/save_result/lines_pictures/" + str(number) +".0frame_plot.csv"
#     if os.path.exists(path1):
#         frame = np.loadtxt(path4)
#         y = np.loadtxt(path3)
#         if y.size < 10:
#             number += 1
#             continue
#         number += 1
#         z1 = np.polyfit(frame, y, 20) # 用4次多项式拟合
#         p1 = np.poly1d(z1)
#         yvals = p1(frame) # 也可以使用yvals=np.polyval(z1,x)
#
#         plt.plot(frame, y, '*',label='original values')
#         plt.plot(frame, yvals, 'r',label='polyfit values')
#         plt.xlabel('x axis')
#         plt.ylabel('y axis')
#         plt.legend(loc=4) # 指定legend的位置,读者可以自己help它的用法
#         plt.title('1')
#         save_path = "/home/wanji/Documents/wjv4/second.pytorch/save_result/lines_pictures/frame/" + str(number) + "fram_y.jpg"
#         plt.savefig(save_path)
#         # plt.grid(True)
#
#
#         plt.figure()
#         x_f = np.loadtxt(path1)
#         z2 = np.polyfit(frame, x_f, 20) # 用4次多项式拟合
#         p2 = np.poly1d(z2)
#         yvals_f = p2(frame)
#
#         plt.plot(frame, x_f, '*', label='original values')
#         plt.plot(frame, yvals_f, 'r',label='polyfit values')
#         plt.xlabel('x axis')
#         plt.ylabel('y axis')
#         plt.legend(loc=4) # 指定legend的位置,读者可以自己help它的用法
#         plt.title('2')
#         save_path = "/home/wanji/Documents/wjv4/second.pytorch/save_result/lines_pictures/frame/" + str(
#             number) + "fram_x.jpg"
#         plt.savefig(save_path)
#         # plt.grid(True)
#         # plt.draw()
#         # plt.show()
#     number += 1




# i = 1
# while True:
#     time.sleep(0.1)
#     path1 = "/home/wanji/Documents/wjv4/second.pytorch/save_result/lines_pictures/" + str(i) + ".0" + "x.csv"
#     path2 = "/home/wanji/Documents/wjv4/second.pytorch/save_result/lines_pictures/" + str(i) + ".0" + "plot.csv"
#     path3 = "/home/wanji/Documents/wjv4/second.pytorch/save_result/lines_pictures/" + str(i) + ".0" + "y.csv"
#     if os.path.exists(path1):
#         x = np.loadtxt(path1)
#         y = np.loadtxt(path2)
#
#         if x.size > 30:
#             distance = math.sqrt((x[0] - x[x.size - 1]) * (x[0] - x[x.size - 1]) + (y[0] - y[y.size - 1]) * (y[0] - y[y.size - 1]))
#             if distance >= 10:
#                 z1 = np.polyfit(x, y, 20) # 用4次多项式拟合
#                 p1 = np.poly1d(z1)
#                 yvals=p1(x) # 也可以使用yvals=np.polyval(z1,x)
#
#                 z2 = np.polyfit(y, x, 20)  # 用4次多项式拟合
#                 p1 = np.poly1d(z2)
#                 yvals2 = p1(y)  # 也可以使用yvals=np.polyval(z1,x)
#
#                 plot1=plt.plot(x, y, '*',label='original values')
#                 plot2=plt.plot(x, yvals, 'r',label='polyfit values')
#                 plt.xlabel('x axis')
#                 plt.ylabel('y axis')
#                 plt.legend(loc=4) # 指定legend的位置,读者可以自己help它的用法
#                 plt.title('polyfitting')
#                 path = "/home/wanji/Documents/wjv4/second.pytorch/save_result/lines_pictures/1/" + str(i) + "test.jpg"
#                 plt.savefig(path)
#                 plt.clf()
#
#                 plt.plot(y, x, '*', label='original values')
#                 plt.plot(y, yvals2, 'r', label='polyfit values')
#                 plt.xlabel('x axis')
#                 plt.ylabel('y axis')
#                 plt.legend(loc=4)  # 指定legend的位置,读者可以自己help它的用法
#                 plt.title('polyfitting')
#                 path = "/home/wanji/Documents/wjv4/second.pytorch/save_result/lines_pictures/1/" + str(i) + "test1.jpg"
#                 plt.savefig(path)
#                 plt.clf()
#                 i += 1
#                 # plt.grid(True)
#                 # plt.draw()
#                 # plt.show()
#                 print("done!")
#     i += 1



# x = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24]
# y = np.array([5760, 3600, 1620, 1260, 1080, 900, 1080, 1800, 3060, 4680, 2880, 5040, 4140, 5580, 5040, 4860, 3780,
#    3420, 4860, 3780, 4860, 5220, 4860, 3600])
# x = np.loadtxt("/home/wanji/Documents/wjv4/second.pytorch/save_result/lines/x.csv")
# y = np.loadtxt("/home/wanji/Documents/wjv4/second.pytorch/save_result/lines/y.csv")
# z1 = np.polyfit(x, y, 50) # 用4次多项式拟合
# p1 = np.poly1d(z1)
# print(p1) # 在屏幕上打印拟合多项式
# yvals=p1(x) # 也可以使用yvals=np.polyval(z1,x)
#
# plot1=plt.plot(x, y, '*',label='original values')
# plot2=plt.plot(x, yvals, 'r',label='polyfit values')
# plt.xlabel('x axis')
# plt.ylabel('y axis')
# plt.legend(loc=4) # 指定legend的位置,读者可以自己help它的用法
# plt.title('polyfitting')
# plt.show()
# #
# # #
# # #
