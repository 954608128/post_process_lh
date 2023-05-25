import os
import pickle
import time
import math
import argparse
import glob
from pathlib import Path
import pdb
import numpy as np
import torch
import open3d as o3d
# from multiprocessing import Process
# from multiprocessing import Queue
from queue import Queue
from core import visualize_utils as V
from numba import jit
import dpkt


#@jit(nopython=True)
@jit()
def init_angle():
    deg = 0
    ver_hori_angle = np.zeros((32, 3600, 3))
    for m in range(32):
        if m == 0:
            deg = 0
        elif m <= 15:
            deg -= 0.6
        elif m <= 23:
            deg -= 1
        elif m <= 27:
            deg -= 2
        else:
            deg -= 3

        vert_cos = math.cos(math.pi / 180 * deg)
        vert_sin = math.sin(math.pi / 180 * deg)
        for n in range(3600):
            ver_hori_angle[m, n, 0] = vert_cos * math.sin(math.pi / 180 * n / 10)  ##x
            ver_hori_angle[m, n, 1] = vert_cos * math.cos(math.pi / 180 * n / 10)  ##y
            ver_hori_angle[m, n, 2] = vert_sin  ##z
    return ver_hori_angle
# 32线包数据解析
#@jit(nopython=True)
@jit()
def get_bag_data(packet_data, ver_hori_angle, now_angle_np, head_data):
    """
    :param packet_data:
        一包的数据
    :param ver_hori_angle:
    :param now_angle_np:
        当前的角度
    :param head_data:
        包头
    :return:
        当前包的数据
    """
    ###读一包数据
    bagdata = np.zeros((320, 8))  # X, Y, Z, I, lineID, D, H_angle, V_angle
    h_temp = 0
    for i in range(0, 1320, 132):
        h_temp = int(
            (packet_data[2 + i] + packet_data[3 + i] * 256) / 10
        )  ###+352（+1630）,-270(y为负)

        for j in range(32):
            index = 32 * (i // 132) + j
            intesty = packet_data[6 + i + j * 4] / 255
            dis_temp = (
                packet_data[4 + i + j * 4] + packet_data[5 + i + j * 4] * 256
            ) * 0.004
            bagdata[index, 3] = intesty
            h_temp_index = h_temp % 3600
            if j % 2 == 0 and j < 23:
                h_temp_index = (40 + h_temp) % 3600
            bagdata[index, 4] = j + 1
            bagdata[index, 5] = dis_temp
            bagdata[index, 6] = (h_temp / 10) % 360
            bagdata[index, 0] = dis_temp * ver_hori_angle[j, h_temp_index - 1, 0]
            bagdata[index, 1] = -dis_temp * ver_hori_angle[j, h_temp_index - 1, 1]
            bagdata[index, 2] = dis_temp * ver_hori_angle[j, h_temp_index - 1, 2]
            if dis_temp == 0:
                bagdata[index, 7] = 0
            else:
                bagdata[index, 7] = -1 * np.arccos(bagdata[index, 2] / dis_temp)
    now_angle = (h_temp / 10) % 360
    now_angle_np[0] = now_angle
    # bagdata[0, 4] = now_angle

    # head_data = np.ones(30)
    # print(head_data)
    for j in range(1320, len(packet_data)):
        head_data[j - 1320] = packet_data[j]

    # return now_angle  # f8
    # return head_data  # f8[:]
    # return bagdata  # f8[:,:]
    return bagdata  # f8[:,:]

def custom_draw_geometry(pcd,linesets):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd)
    for i in linesets:
        vis.add_geometry(i)
    render_option = vis.get_render_option()
    render_option.point_size = 4
    render_option.background_color = np.asarray([0, 0, 0])
    vis.run()
    vis.destroy_window()


def read_wjlidar(Point_data, pcap_path):
    last_angle = -1
    frame_id = 0
    now_angle_np = np.ones(1)
    head_data = np.ones(30)

    k = 0
    ver_hori_angle = init_angle()
    with open(pcap_path, "rb") as f:
        pcap = dpkt.pcap.Reader(f)
        for timestamp, raw_buf in pcap:
            eth = dpkt.ethernet.Ethernet(raw_buf)
            if not isinstance(eth.data, dpkt.ip.IP):
                continue
            if not isinstance(
                    eth.data.data, dpkt.udp.UDP
            ):
                continue
            packet = eth.data
            udp_data = packet.data
            pl_data = udp_data.data
            packet_data = np.array([int(i) for i in pl_data])

            if len(packet_data) == 1350:
                bag_data = get_bag_data(
                    packet_data, ver_hori_angle, now_angle_np, head_data
                )

                now_angle = now_angle_np[0]
                if now_angle > last_angle:
                    if last_angle == -1:
                        PC_data = bag_data
                        last_angle = now_angle
                    else:
                        PC_data = np.concatenate((PC_data, bag_data), axis=0)
                        last_angle = now_angle
                else:
                    last_angle = 0
                    frame_id += 1

                    Point_data.put(PC_data[:, :4])

                    PC_data = bag_data

def pcdet_wj(Point_data, view, save_path):
    if view:
        # ==================================== open3d =================================
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name='pvrcnn alg show', width=1080, height=720, left=300, top=150, visible=True)
        point_cloud = o3d.geometry.PointCloud()
        to_reset = True
        render_option = vis.get_render_option()
        render_option.point_size = 1
        render_option.background_color = np.asarray([0, 0, 0])
        lineset_update = [o3d.geometry.LineSet() for _ in range(250)]
        temp_p = np.array([[0, 0, 0], [0.01, 0, 0], [0, 0.01, 0], [0.01, 0.01, 0], [0, 0, 0.01], [0.01, 0, 0.01],
                           [0, 0.01, 0.01], [0.01, 0.01, 0.01]], dtype=np.float32)
        for i in lineset_update:
            vis.add_geometry(i)
        vis.add_geometry(point_cloud)
        # ===================================================================================

    with torch.no_grad():
        k = 0
        while True:
            #points = Point_data.get()
            #t1 = time.time()
            #data_dict = demo_dataset.collate_batch([demo_dataset.prepare(points)])
            #load_data_to_gpu(data_dict)

            #pred_dicts, _  = model.forward(data_dict)

            path_list = os.listdir(save_path)
            path_list.sort(key=lambda x: int(x.split('.')[0]))
            for i in range(len(path_list)):
                load_path = save_path + path_list[i]
                de_result = np.load(load_path, allow_pickle=True)

                boxes_corners = V.boxes_to_corners_3d(torch.from_numpy(de_result[i][0]))

                # boxes_numpy = boxes_corners.cpu().numpy()
                # #print(boxes_numpy)
                # # ==========================================================================
                # if save_path is not None:
                #     #res = {}
                #     #res["box3d_lidar"] = pred_dicts[0]["pred_boxes"]
                #     #res["scores"] = pred_dicts[0]["pred_scores"]
                #     #res["label_preds"] = pred_dicts[0]["pred_labels"]
                #
                #     if not os.path.exists(save_path):
                #         os.makedirs(save_path)
                #     with open(save_path + str(k).zfill(4) + ".pkl", 'wb') as fo:
                #         pickle.dump(boxes_numpy, fo)
                #     # ==========================================================================

                # print("Frame {} : {} targets were detected".format(k, len(pred_dicts[0]['pred_boxes'])))
                # print("* "* 20 + "cost time : {}".format(time.time() - t1))
                k += 1
                if view:
                    # point_cloud.points = o3d.utility.Vector3dVector(points[:, :3])
                    # boxes_corners = V.boxes_to_corners_3d(pred_dicts[0]['pred_boxes'])
                    for i in range(boxes_corners.shape[0]):
                        points_box = boxes_corners[i]
                        # pdb.set_trace()
                        # print(points_box)
                        center = torch.mean(points_box, 0).reshape((-1, 3)) # 中心点
                        points_box = torch.cat((points_box, center), 0)
                        points_cen = torch.mean(points_box[[0, 1, 4, 5], :], 0).reshape((-1, 3))  # 车头中心
                        points_box = torch.cat((points_box, points_cen), 0)
                        lines_box = np.array([[0, 1], [1, 2], [0, 3], [2, 3], [4, 5], [4, 7], [5, 6], [6, 7],
                                              [0, 4], [1, 5], [2, 6], [3, 7], [8, 9], [0, 5], [1, 4]])
                        colors = np.array([[1, 0, 0], [0, 1, 0], [0, 1, 0], [0, 1, 0], [1, 0, 0], [0, 1, 0],
                                           [0, 1, 0], [0, 1, 0], [1, 0, 0], [1, 0, 0], [0, 1, 0], [0, 1, 0],
                                           [1, 0, 0], [1, 0, 0], [1, 0, 0]])
                        # line_set = o3d.geometry.LineSet()
                        lineset_update[i].points = o3d.utility.Vector3dVector(np.array(points_box.cpu()))
                        lineset_update[i].lines = o3d.utility.Vector2iVector(lines_box)
                        lineset_update[i].colors = o3d.utility.Vector3dVector(colors)
                        # linesets.append(line_set)
                    for j in range(boxes_corners.shape[0], 250):
                        lines_box = np.array([[0, 1], [1, 2], [0, 3], [2, 3], [4, 5], [4, 7], [5, 6], [6, 7],
                                              [0, 4], [1, 5], [2, 6], [3, 7]])
                        colors = np.array([[0, 0, 0] for j in range(len(lines_box))])
                        # line_set = o3d.geometry.LineSet()
                        lineset_update[j].points = o3d.utility.Vector3dVector(temp_p)
                        lineset_update[j].lines = o3d.utility.Vector2iVector(lines_box)
                        lineset_update[j].colors = o3d.utility.Vector3dVector(colors)

                    vis.update_geometry()
                    # time.sleep(0.1)
                    if to_reset:
                        vis.reset_view_point(True)
                        to_reset = False
                    vis.poll_events()
                    vis.update_renderer()

if __name__ == "__main__":
    #view = True
    view = False
    pcap_path = r'/home/wanji/Documents/wjv4/data/detection_result/20200821100429.pcap'
    save_path = r'/home/wanji/Documents/wjv4/data/detection_result/20200821100429_result/'

    Point_data = Queue()
    Dete_res = Queue()
    read = Process(target=read_wjlidar, args=(Point_data, pcap_path,))
    detection = Process(target=pcdet_wj, args=(Point_data, view, save_path))


    read.start()
    detection.start()

    read.join()
    detection.terminate()
