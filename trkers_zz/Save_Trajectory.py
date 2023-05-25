import numpy as np

def Save_Trajectory_(all_trk_ID_det):
    # 遍历保存置信度值较低的跟踪链，阈值为0.3
    number_reliable = 0
    for i in all_trk_ID_det.keys():
        save_er_x = []
        save_er_y = []
        frames = []
        for j in range(len(all_trk_ID_det[i])):
            if all_trk_ID_det[i][j][10] < 0.3:
                save_er_x.append(all_trk_ID_det[i][j][0])
                save_er_y.append(all_trk_ID_det[i][j][1])
                frames.append(all_trk_ID_det[i][j][13])
        if len(save_er_x) > 10:
            xuhao = int(all_trk_ID_det[i][0][9])
            path1 = "./data_save/lines_pictures/" + str(xuhao) + "x.csv"
            path2 = "./data_save/lines_pictures/" + str(xuhao) + "y.csv"
            path3 = "./data_save/lines_pictures/" + str(xuhao) + "frame_plot.csv"
            np.savetxt(path3, frames)
            np.savetxt(path1, save_er_x)
            np.savetxt(path2, save_er_y)
            save_er_x.clear()
            save_er_y.clear()
            frames.clear()

        if all_trk_ID_det[i][0][10] >= 0.3:
            number_reliable += 1
    return number_reliable