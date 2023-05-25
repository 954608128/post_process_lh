from distutils import core
import os
import math
import numpy as np
import torch
import open3d as o3d
import open3d.visualization.gui as gui
#import visualize_utils as V
import time
import pcl
import pickle
import mayavi.mlab as mlab
import numpy as np
import torch
import torch as nn
import matplotlib.pyplot as plt
import cv2


def check_numpy_to_torch(x):
    if isinstance(x, np.ndarray):
        return torch.from_numpy(x).float(), True
    return x, False


def rotate_points_along_z(points, angle):
    """
    Args:
        points: (B, N, 3 + C)
        angle: (B), angle along z-axis, angle increases x ==> y
    Returns:

    """
    points, is_numpy = check_numpy_to_torch(points)
    angle, _ = check_numpy_to_torch(angle)

    cosa = torch.cos(angle)
    sina = torch.sin(angle)
    zeros = angle.new_zeros(points.shape[0])
    ones = angle.new_ones(points.shape[0])
    rot_matrix = torch.stack((
        cosa,  sina, zeros,
        -sina, cosa, zeros,
        zeros, zeros, ones
    ), dim=1).view(-1, 3, 3).float()
    points_rot = torch.matmul(points[:, :, 0:3], rot_matrix)
    points_rot = torch.cat((points_rot, points[:, :, 3:]), dim=-1)
    return points_rot.numpy() if is_numpy else points_rot


def boxes_to_corners_3d(boxes3d):
    """
        7 -------- 4
       /|         /|
      6 -------- 5 .
      | |        | |
      . 3 -------- 0
      |/         |/
      2 -------- 1
    Args:
        boxes3d:  (N, 7) [x, y, z, dx, dy, dz, heading], (x, y, z) is the box center

    Returns:
    """
    boxes3d, is_numpy = check_numpy_to_torch(boxes3d)
    
    template = boxes3d.new_tensor((
        [1, 1, -1], [1, -1, -1], [-1, -1, -1], [-1, 1, -1],
        [1, 1, 1], [1, -1, 1], [-1, -1, 1], [-1, 1, 1],
    )) / 2
    
    corners3d = boxes3d[:, None, 3:6].repeat(1, 8, 1) * template[None, :, :]
    corners3d = rotate_points_along_z(corners3d.view(-1, 8, 3), boxes3d[:, 6]).view(-1, 8, 3)
    corners3d += boxes3d[:, None, 0:3]

    # corners3d
    return corners3d.numpy() if is_numpy else corners3d ,boxes3d[:,-1],


def visualize_pts(pts, fig=None, bgcolor=(0, 0, 0), fgcolor=(1.0, 1.0, 1.0),
                  show_intensity=False, size=(600, 600), draw_origin=True):
    if not isinstance(pts, np.ndarray):
        pts = pts.cpu().numpy()
    if fig is None:
        fig = mlab.figure(figure=None, bgcolor=bgcolor, fgcolor=fgcolor, engine=None, size=size)

    if show_intensity:
        G = mlab.points3d(pts[:, 0], pts[:, 1], pts[:, 2], pts[:, 3], mode='point',
                          colormap='gnuplot', scale_factor=1, figure=fig)
    else:
        G = mlab.points3d(pts[:, 0], pts[:, 1], pts[:, 2], mode='point',
                          colormap='gnuplot', scale_factor=1, figure=fig)
    if draw_origin:
        mlab.points3d(0, 0, 0, color=(1, 1, 1), mode='cube', scale_factor=0.2)
        mlab.plot3d([0, 3], [0, 0], [0, 0], color=(0, 0, 1), tube_radius=0.1)
        mlab.plot3d([0, 0], [0, 3], [0, 0], color=(0, 1, 0), tube_radius=0.1)
        mlab.plot3d([0, 0], [0, 0], [0, 3], color=(1, 0, 0), tube_radius=0.1)

    return fig


def draw_sphere_pts(pts, color=(0, 1, 0), fig=None, bgcolor=(0, 0, 0), scale_factor=0.2):
    if not isinstance(pts, np.ndarray):
        pts = pts.cpu().numpy()

    if fig is None:
        fig = mlab.figure(figure=None, bgcolor=bgcolor, fgcolor=None, engine=None, size=(600, 600))

    if isinstance(color, np.ndarray) and color.shape[0] == 1:
        color = color[0]
        color = (color[0] / 255.0, color[1] / 255.0, color[2] / 255.0)

    if isinstance(color, np.ndarray):
        pts_color = np.zeros((pts.__len__(), 4), dtype=np.uint8)
        pts_color[:, 0:3] = color
        pts_color[:, 3] = 255
        G = mlab.points3d(pts[:, 0], pts[:, 1], pts[:, 2], np.arange(0, pts_color.__len__()), mode='sphere',
                          scale_factor=scale_factor, figure=fig)
        G.glyph.color_mode = 'color_by_scalar'
        G.glyph.scale_mode = 'scale_by_vector'
        G.module_manager.scalar_lut_manager.lut.table = pts_color
    else:
        mlab.points3d(pts[:, 0], pts[:, 1], pts[:, 2], mode='sphere', color=color,
                      colormap='gnuplot', scale_factor=scale_factor, figure=fig)

    mlab.points3d(0, 0, 0, color=(1, 1, 1), mode='cube', scale_factor=0.2)
    mlab.plot3d([0, 3], [0, 0], [0, 0], color=(0, 0, 1), line_width=3, tube_radius=None, figure=fig)
    mlab.plot3d([0, 0], [0, 3], [0, 0], color=(0, 1, 0), line_width=3, tube_radius=None, figure=fig)
    mlab.plot3d([0, 0], [0, 0], [0, 3], color=(1, 0, 0), line_width=3, tube_radius=None, figure=fig)

    return fig


def draw_grid(x1, y1, x2, y2, fig, tube_radius=None, color=(0.5, 0.5, 0.5)):
    mlab.plot3d([x1, x1], [y1, y2], [0, 0], color=color, tube_radius=tube_radius, line_width=1, figure=fig)
    mlab.plot3d([x2, x2], [y1, y2], [0, 0], color=color, tube_radius=tube_radius, line_width=1, figure=fig)
    mlab.plot3d([x1, x2], [y1, y1], [0, 0], color=color, tube_radius=tube_radius, line_width=1, figure=fig)
    mlab.plot3d([x1, x2], [y2, y2], [0, 0], color=color, tube_radius=tube_radius, line_width=1, figure=fig)
    return fig


def draw_multi_grid_range(fig, grid_size=20, bv_range=(-60, -60, 60, 60)):
    for x in range(bv_range[0], bv_range[2], grid_size):
        for y in range(bv_range[1], bv_range[3], grid_size):
            fig = draw_grid(x, y, x + grid_size, y + grid_size, fig)

    return fig


def draw_scenes(points, gt_boxes=None, ref_boxes=None, ref_scores=None, ref_labels=None):
    if not isinstance(points, np.ndarray):
        points = points.cpu().numpy()
    if ref_boxes is not None and not isinstance(ref_boxes, np.ndarray):
        ref_boxes = ref_boxes.cpu().numpy()
    if gt_boxes is not None and not isinstance(gt_boxes, np.ndarray):
        gt_boxes = gt_boxes.cpu().numpy()
    if ref_scores is not None and not isinstance(ref_scores, np.ndarray):
        ref_scores = ref_scores.cpu().numpy()
    if ref_labels is not None and not isinstance(ref_labels, np.ndarray):
        ref_labels = ref_labels.cpu().numpy()

    fig = visualize_pts(points)
    fig = draw_multi_grid_range(fig, bv_range=(0, -40, 80, 40))
    if gt_boxes is not None:
        corners3d = boxes_to_corners_3d(gt_boxes)
        fig = draw_corners3d(corners3d, fig=fig, color=(0, 0, 1), max_num=100)

    if ref_boxes is not None and len(ref_boxes) > 0:
        ref_corners3d = boxes_to_corners_3d(ref_boxes)
        if ref_labels is None:
            fig = draw_corners3d(ref_corners3d, fig=fig, color=(0, 1, 0), cls=ref_scores, max_num=100)
        else:
            for k in range(ref_labels.min(), ref_labels.max() + 1):
                cur_color = tuple(box_colormap[k % len(box_colormap)])
                mask = (ref_labels == k)
                fig = draw_corners3d(ref_corners3d[mask], fig=fig, color=cur_color, cls=ref_scores[mask], max_num=100)
    mlab.view(azimuth=-179, elevation=54.0, distance=104.0, roll=90.0)
    return fig
    
def save_view_point(pcd, filename):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user changes the view and press "q" to terminate
    param = vis.get_view_control().convert_to_pinhole_camera_parameters()
    o3d.io.write_pinhole_camera_parameters(filename, param)
    vis.destroy_window()
    import pdb;pdb.set_trace()


def draw_corners3d(corners3d, fig, color=(1, 1, 1), line_width=2, cls=None, tag='', max_num=500, tube_radius=None):
    """
    :param corners3d: (N, 8, 3)
    :param fig:
    :param color:
    :param line_width:
    :param cls:
    :param tag:
    :param max_num:
    :return:
    """
    import mayavi.mlab as mlab
    num = min(max_num, len(corners3d))
    for n in range(num):
        b = corners3d[n]  # (8, 3)

        if cls is not None:
            if isinstance(cls, np.ndarray):
                mlab.text3d(b[6, 0], b[6, 1], b[6, 2], '%.2f' % cls[n], scale=(0.3, 0.3, 0.3), color=color, figure=fig)
            else:
                mlab.text3d(b[6, 0], b[6, 1], b[6, 2], '%s' % cls[n], scale=(0.3, 0.3, 0.3), color=color, figure=fig)

        for k in range(0, 4):
            i, j = k, (k + 1) % 4
            mlab.plot3d([b[i, 0], b[j, 0]], [b[i, 1], b[j, 1]], [b[i, 2], b[j, 2]], color=color, tube_radius=tube_radius,
                        line_width=line_width, figure=fig)

            i, j = k + 4, (k + 1) % 4 + 4
            mlab.plot3d([b[i, 0], b[j, 0]], [b[i, 1], b[j, 1]], [b[i, 2], b[j, 2]], color=color, tube_radius=tube_radius,
                        line_width=line_width, figure=fig)

            i, j = k, k + 4
            mlab.plot3d([b[i, 0], b[j, 0]], [b[i, 1], b[j, 1]], [b[i, 2], b[j, 2]], color=color, tube_radius=tube_radius,
                        line_width=line_width, figure=fig)

        i, j = 0, 5
        mlab.plot3d([b[i, 0], b[j, 0]], [b[i, 1], b[j, 1]], [b[i, 2], b[j, 2]], color=color, tube_radius=tube_radius,
                    line_width=line_width, figure=fig)
        i, j = 1, 4
        mlab.plot3d([b[i, 0], b[j, 0]], [b[i, 1], b[j, 1]], [b[i, 2], b[j, 2]], color=color, tube_radius=tube_radius,
                    line_width=line_width, figure=fig)

    return fig

def border():
    #绘制顶点
    polygon_points = np.array([[-100, 100, -4], [100, 100, -4], [100, -100, -4],[-100,-100, -4]])
    lines = [[0, 1], [1, 2], [2, 3],[3, 0]] #连接的顺序，封闭链接
    color = [[1, 1, 0] for _ in range(len(lines))] 
    #添加顶点，点云
    points_pcd = o3d.geometry.PointCloud()
    points_pcd.points = o3d.utility.Vector3dVector(polygon_points)
    points_pcd.paint_uniform_color([1, 1, 0]) #点云颜色
 
    #绘制线条
    lines_pcd = o3d.geometry.LineSet()
    lines_pcd.lines = o3d.utility.Vector2iVector(lines)
    lines_pcd.colors = o3d.utility.Vector3dVector(color) #线条颜色
    lines_pcd.points = o3d.utility.Vector3dVector(polygon_points)
 
    return lines_pcd, points_pcd

def paint_circle(center, radius):
    points = []
    for theta in np.arange(0, 2*np.pi, 0.0006):
        coor = (center[0]+radius*np.sin(theta), center[1]+radius*np.cos(theta), 0)
        points.append(coor)
    points = np.array(points)
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    return point_cloud

def custom_draw_geometry(vis, pcd,linesets_gt,linesets_pred,id_list):
    vis.add_geometry(pcd)#, reset_bounding_box=True
    # for i in linesets_gt:
    #     vis.add_geometry(i)
    for i in linesets_pred:
        vis.add_geometry(i)
    # import pdb;pdb.set_trace()
    for i in id_list:
        vis.add_geometry(i)
    # for radius in [30, 50, 80]:
    #     circle = paint_circle(center=(5, -22), radius=radius)
    #     circle.paint_uniform_color([1, 1, 0])
    #     vis.add_geometry(circle)
    lines, points = border()
    vis.add_geometry(lines)
    vis.add_geometry(points)
    render_option = vis.get_render_option()
    render_option.point_size = 1
    # render_option.point_color_option(1,0,9,2,3,4)
    #import pdb;pdb.set_trace()
    # render_option.background_color = np.asarray([0, 0, 0])
    '''这行读视角'''
    view_json_filename = './viewpoint.json'
    load_view_point(vis, view_json_filename)
    # 一定要q才开始跑
    # vis.update_renderer
    # vis.reset_view_point=False
    
    # vis.reset_view_point(False)
    vis.run()
    # time.sleep(0.1)

    # if cv2.waitKey(1) & 0xFF == ord(' '):
    #     # vis.run()
    #     # time.sleep(0.1)
    #     cv2.waitKey(0)
    # else:
    #     # vis.run()
    #     # time.sleep(0.1)
    vis.clear_geometries()

def read_point_cloud_bin(bin_path):
    """
    Read point cloud in bin format

    Parameters
    ----------
    bin_path: str
        Input path of Oxford point cloud bin

    Returns
    ----------

    """
    data = np.fromfile(bin_path, dtype=np.float64)
    #import pdb;pdb.set_trace()
    # format:
    N, D = data.shape[0]// 6, 6
    point_cloud_with_normal = np.reshape(data, (N, D))

    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(point_cloud_with_normal[:, 0:3])
    point_cloud.normals = o3d.utility.Vector3dVector(point_cloud_with_normal[:, 3:6])

    return point_cloud

def text_3d(text, pos, direction=None, degree=-90.0, font='calibril.ttf', font_size=15,density=2):
    """
    Generate a 3D text point cloud used for visualization.
    :param text: content of the text
    :param pos: 3D xyz position of the text upper left corner
    :param direction: 3D normalized direction of where the text faces
    :param degree: in plane rotation of text
    :param font: Name of the font - change it according to your system
    :param font_size: size of the font
    :return: o3d.geoemtry.PointCloud object
    """
    if direction is None:
        direction = (0., 0., 1.)
    from PIL import Image, ImageFont, ImageDraw
    from pyquaternion import Quaternion

    font_obj = ImageFont.truetype(font, font_size)
    font_dim = font_obj.getsize(text)

    img = Image.new('RGB', font_dim, color=(255, 255, 255))
    draw = ImageDraw.Draw(img)
    draw.text((0, 0), text, font=font_obj, fill=(0, 0, 0))
    img = np.asarray(img)
    img_mask = img[:, :, 0] < 128
    indices = np.indices([*img.shape[0:2], 1])[:, img_mask, 0].reshape(3, -1).T
    # import pdb;pdb.set_trace()
    pcd = o3d.geometry.PointCloud()
    pcd.colors = o3d.utility.Vector3dVector(img[img_mask, :].astype(float)/ 255 )
    pcd.points = o3d.utility.Vector3dVector(indices/5)

    raxis = np.cross([0.0, 0.0, 1.0], direction)
    if np.linalg.norm(raxis) < 1e-6:
        raxis = (0.0, 0.0, 1.0)
    trans = (Quaternion(axis=raxis, radians=np.arccos(direction[2])) *
             Quaternion(axis=direction, degrees=degree)).transformation_matrix
    trans[0:3, 3] = np.asarray(pos)
    pcd.transform(trans)
    return pcd

def read_bin_velodyne(path):
    import struct
    pc_list=[]
    with open(path,'rb') as f:
        content=f.read()
        pc_iter=struct.iter_unpack('ffff',content)
        for idx,point in enumerate(pc_iter):
            pc_list.append([point[0],point[1],point[2]])
    return np.asarray(pc_list,dtype=np.float32)

def load_view_point(vis, view_json_filename):
    ctr = vis.get_view_control()
    param = o3d.io.read_pinhole_camera_parameters(view_json_filename)
    ctr.convert_from_pinhole_camera_parameters(param)

def read_npy_velodyne(path):
    pc_list = []
    np.set_printoptions(suppress=True) 
    # 作用是取消numpy默认的科学计数法，测试表明open3d点云读取函数没法读取科学计数法的表示
    data = np.load(path)[:,0:3]

    return data

def post_process_vis(detection_result_path,post_process_result_path):
    detection_result_list = os.listdir(detection_result_path)
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    for detection_result in detection_result_list:
        detection = detection_result_path + detection_result
        post_process = post_process_result_path + detection_result
        points = np.array([[0,0,0]])
        points=read_bin_velodyne('D:/post_process/trkers_zz/bin/SZ-3D-dou-64line-20230417/20230414072511_000901.bin')
        # points = points.astype(np.float32)
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(points)
        
        linesets_gt = []
        linesets_pred = []
        # draw label
        gt_result = []
        pred_result =[]
        ''' 用这行存视角'''
        # save_view_point(point_cloud, "viewpoint.json")
        # points = o3d.io.read_point_cloud(bin_file)
        # points = np.array(points.points)
    ####################################################
    #设定预测模型
    ####################################################
        with open(post_process, 'r') as f:
            print(post_process)
            lines = f.readlines()
            for idx in range(len(lines)):
                label = lines[idx].strip().split(',')
                id = int(float(label[0]))
                x = float(label[2]) / 100
                y = float(label[3]) / 100
                z = float(label[4]) / 100
                l = float(label[7]) / 100
                w = float(label[8]) / 100
                h = float(label[9]) / 100
                alp = float(label[6]) / 180 * math.pi
                cls = float(label[1])
                pred_result.append([x, y, z, l, w, h, alp,cls,id])
        
        with open(detection, 'r') as f:
            lines = f.readlines()
            # print(label_file)
            for idx in range(len(lines)):
                label = lines[idx].strip().split(',')
                x = float(label[2]) / 100
                y = float(label[3]) / 100
                z = float(label[4]) / 100
                l = float(label[7]) / 100
                w = float(label[8]) / 100
                h = float(label[9]) / 100
                alp = float(label[6]) / 180 * math.pi
                cls = float(label[1])
                # print('max z: ', z+(h/2))
                # if cls == float(201):
                gt_result.append([x, y, z, l, w, h, alp,cls])

        gt_boxes_corners ,gt_cls = boxes_to_corners_3d(torch.Tensor(gt_result))
        pred_boxes_corners ,pred_id= boxes_to_corners_3d(torch.Tensor(pred_result))
        id_list = []
        for j in range(pred_boxes_corners.shape[0]):
            pred_points_box = pred_boxes_corners[j]
            center = torch.mean(pred_points_box, 0).reshape((-1, 3))
            pred_points_box = torch.cat((pred_points_box, center), 0)
            points_cen = torch.mean(pred_points_box[[0, 1, 4, 5], :], 0).reshape((-1, 3))  # 车头中心
            pred_points_box = torch.cat((pred_points_box, points_cen), 0)
            lines_box = np.array([[0, 1], [1, 2], [0, 3], [2, 3], [4, 5], [4, 7], [5, 6], [6, 7],
                                  [0, 4], [1, 5], [2, 6], [3, 7], [8, 9], [0, 5], [1, 4]])

            colors = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0],
            [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0],
            [0, 0, 0], [0, 0, 0], [0, 0, 0]])
            pred_line_set = o3d.geometry.LineSet()
            pred_line_set.points = o3d.utility.Vector3dVector(np.array(pred_points_box.cpu()))
            pred_line_set.lines = o3d.utility.Vector2iVector(lines_box)
            pred_line_set.colors = o3d.utility.Vector3dVector(colors)

            linesets_pred.append(pred_line_set)
            
            id_pcd = text_3d(str(int(np.array(pred_id[j]))),points_cen)
            # import pdb;pdb.set_trace()
            id_list.append(id_pcd)
            
        custom_draw_geometry(vis, point_cloud,linesets_gt,linesets_pred,id_list)


if __name__ == "__main__":
    detection_result_path = 'D:/post_process/trkers_zz/data/detect_csv/'
    post_process_result_path = 'D:/post_process/trkers_zz/data_save/save/'
    post_process_vis(detection_result_path,post_process_result_path)