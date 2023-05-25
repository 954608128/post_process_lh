import pcl
import numpy as np

from real_time_function import save_bmp

if __name__=='__main__':
    load_path = r'./20230414072511_000901.pcd'
    bmp_path = r'./20230414072511_000901.bmp'
    import pdb;pdb.set_trace()
    in_points = pcl.load_XYZI(load_path)
    in_points_ = np.array(in_points)
    save_bmp(in_points_, 2000, bmp_path)