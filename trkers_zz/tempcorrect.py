def SaveframelossFun(frameloss):
    filename = "./data_save/" + "frameloss.csv"
    with open(filename,'w') as file:
        file_csv = csv.writer(file)
        for obj_ in frameloss.keys():
            file_csv.writerow([obj_,frameloss[obj_]])


def SavecutoutBoxFun(matchcut,frameid):
    filename = "./data_save/" + "cutout.csv"
    with open(filename, 'w') as file:
        file_csv = csv.writer(file)
        for obj_ in matchcut.keys():
            file_csv.writerow([obj_, matchcut[obj_],frameid[obj_]])  #



def SaveFillBoxFun(framematch,fillforeobj):
    for objframe in framematch.keys():
        filename = "./data_save/forefill/" + str(int(objframe)) + ".csv"
        with open(filename, 'w') as file:
            file_csv = csv.writer(file)
            for obj_ in framematch[objframe].keys():
                file_csv.writerow([objframe,obj_,fillforeobj[obj_]]) # 补充的目标 相邻的目标 补充的帧号

def Cutout_correct(all_trk_fram,all_trk_ID_det):
    Rads_cov = 180.0 / math.pi
    all_trk_ID_det_ = copy.deepcopy(all_trk_ID_det)
    matchcut = {}
    frameid = {}
    for i in all_trk_ID_det_.keys():
        if all_trk_ID_det_[i][0][7] in [0, 1]:  # [1, 2]
            disorg = math.sqrt(all_trk_ID_det_[i][-1][0] ** 2 + all_trk_ID_det_[i][-1][1] ** 2)
            if disorg < 25:  # 距离原点小于25m
                print("发现有断链的目标，断链的ID是{}，断链的距离是{},断链的帧号是{}".format(all_trk_ID_det_[i][-1][9], disorg,
                                                                   int(all_trk_ID_det_[i][-1][13])))
                det1 = math.sqrt((all_trk_ID_det_[i][-4][0] - all_trk_ID_det_[i][-5][0]) ** 2 + (
                            all_trk_ID_det_[i][-4][1] - all_trk_ID_det_[i][-5][1]) ** 2) / (
                                   (all_trk_ID_det_[i][-4][13] - all_trk_ID_det_[i][-5][13]) * 0.1)
                det2 = math.sqrt((all_trk_ID_det_[i][-3][0] - all_trk_ID_det_[i][-4][0]) ** 2 + (
                        all_trk_ID_det_[i][-3][1] - all_trk_ID_det_[i][-4][1]) ** 2) / (
                                   (all_trk_ID_det_[i][-3][13] - all_trk_ID_det_[i][-4][13]) * 0.1)
                det3 = math.sqrt((all_trk_ID_det_[i][-2][0] - all_trk_ID_det_[i][-3][0]) ** 2 + (
                        all_trk_ID_det_[i][-2][1] - all_trk_ID_det_[i][-3][1]) ** 2) / (
                                   (all_trk_ID_det_[i][-2][13] - all_trk_ID_det_[i][-3][13]) * 0.1)
                det4 = math.sqrt((all_trk_ID_det_[i][-1][0] - all_trk_ID_det_[i][-2][0]) ** 2 + (
                        all_trk_ID_det_[i][-1][1] - all_trk_ID_det_[i][-2][1]) ** 2) / (
                                   (all_trk_ID_det_[i][-1][13] - all_trk_ID_det_[i][-2][13]) * 0.1)
                detangle1 = all_trk_ID_det_[i][-4][4] - all_trk_ID_det_[i][-5][4]
                detw1 = detangle1 / ((all_trk_ID_det_[i][-4][13] - all_trk_ID_det_[i][-5][13]) * 0.1)
                detangle2 = all_trk_ID_det_[i][-3][4] - all_trk_ID_det_[i][-4][4]
                detw2 = detangle2 / ((all_trk_ID_det_[i][-3][13] - all_trk_ID_det_[i][-4][13]) * 0.1)
                detangle3 = all_trk_ID_det_[i][-2][4] - all_trk_ID_det_[i][-1][4]
                detw3 = detangle3 / ((all_trk_ID_det_[i][-2][13] - all_trk_ID_det_[i][-3][13]) * 0.1)
                detangle4 = all_trk_ID_det_[i][-1][4] - all_trk_ID_det_[i][-2][4]
                detw4 = detangle4 / ((all_trk_ID_det_[i][-1][13] - all_trk_ID_det_[i][-2][13]) * 0.1)
                averagew = np.mean(np.array([detw1, detw2, detw3, detw4]))
                averagew = abs(averagew)
                averdis = np.mean(np.array([det1, det2, det3, det4]))
                averspeed = averdis
                disthreshold = averspeed * 5
                for obj_ in all_trk_ID_det_.keys():
                    if obj_ == i:
                        continue
                    if all_trk_ID_det_[obj_][0][13] > all_trk_ID_det_[i][-1][13]:
                        detspace = math.sqrt((all_trk_ID_det_[obj_][0][0] - all_trk_ID_det_[i][-1][0]) ** 2 + (
                                    all_trk_ID_det_[obj_][0][1] - all_trk_ID_det_[i][-1][1]) ** 2)
                        if detspace <= disthreshold:
                            if abs(all_trk_ID_det_[obj_][0][4] - all_trk_ID_det_[i][-1][4]) % 360 < averagew * (
                                    all_trk_ID_det_[obj_][0][13] - all_trk_ID_det_[i][0][13]) * 0.1:
                                lineangle = math.asin(
                                    (all_trk_ID_det_[obj_][0][1] - all_trk_ID_det_[i][-1][1]) / math.sqrt(
                                        (all_trk_ID_det_[obj_][0][0] - all_trk_ID_det_[i][-1][0]) ** 2 + (
                                                    all_trk_ID_det_[obj_][0][1] - all_trk_ID_det_[i][-1][1]) ** 2))
                                lineangle = lineangle * Rads_cov
                                lineangle = 270 - lineangle
                                if abs(lineangle - all_trk_ID_det_[obj_][0][4]) < 8 and abs(
                                        lineangle - all_trk_ID_det_[i][-1][4]) < 8:
                                    if all_trk_ID_det_[obj_][0][7] == all_trk_ID_det_[i][-1][7]:
                                        detnew1 = math.sqrt(
                                            (all_trk_ID_det_[obj_][1][0] - all_trk_ID_det_[obj_][0][0]) ** 2 + (
                                                        all_trk_ID_det_[obj_][1][1] - all_trk_ID_det_[obj_][0][
                                                    1]) ** 2) / ((all_trk_ID_det_[obj_][1][13] -
                                                                  all_trk_ID_det_[obj_][0][13]) * 0.1)
                                        detnew2 = math.sqrt(
                                            (all_trk_ID_det_[obj_][2][0] - all_trk_ID_det_[obj_][1][0]) ** 2 + (
                                                    all_trk_ID_det_[obj_][2][1] - all_trk_ID_det_[obj_][1][1]) ** 2) / (
                                                              (all_trk_ID_det_[obj_][2][13] - all_trk_ID_det_[obj_][1][
                                                                  13]) * 0.1)
                                        detnew3 = math.sqrt(
                                            (all_trk_ID_det_[obj_][3][0] - all_trk_ID_det_[obj_][2][0]) ** 2 + (
                                                        all_trk_ID_det_[obj_][3][1] - all_trk_ID_det_[obj_][2][
                                                    1]) ** 2) / ((all_trk_ID_det_[obj_][3][13] -
                                                                  all_trk_ID_det_[obj_][2][13]) * 0.1)
                                        newaverdis = np.mean(np.array([detnew1, detnew2, detnew3]))
                                        if abs((newaverdis - averspeed) * (
                                                all_trk_ID_det_[obj_][0][13] - all_trk_ID_det_[i][-1][13]) * 0.1) < 1:
                                            rundist = averspeed * (
                                                        all_trk_ID_det_[obj_][0][13] - all_trk_ID_det_[i][-1][
                                                    13]) * 0.1  # 位移进行X Y 分解
                                            rundistX = rundist * math.cos(270 - all_trk_ID_det_[i][-1][4])
                                            rundistY = rundist * math.sin(270 - all_trk_ID_det_[i][-1][4])
                                            trk = copy.deepcopy(all_trk_ID_det_[i][-1])
                                            trk[0] = all_trk_ID_det_[i][-1][0] + rundistX
                                            trk[1] = all_trk_ID_det_[i][-1][1] + rundistY
                                            cal_box = np.array([trk])
                                            fit_box = np.array([all_trk_ID_det_[obj_][0]])
                                            cal_iou = rotate_nms_cc(cal_box, fit_box)
                                            print("IOU IS {}".format(cal_iou))
                                            if cal_iou > 0.5:  # 匹配 计算的和实际的匹配可认为即是断链
                                                frameinterval = all_trk_ID_det_[obj_][0][13] - all_trk_ID_det_[i][-1][
                                                    13]
                                                startfillpos = int(all_trk_ID_det_[i][-1][13] + 1)
                                                endfillpos = int(all_trk_ID_det_[obj_][0][13])
                                                if frameinterval > 0:
                                                    frameid[i] = (startfillpos, endfillpos)
                                                    matchcut[i] = obj_
                                                    if len(all_trk_ID_det_[i]) >= 5 and len(all_trk_ID_det_[obj_]) >= 3:
                                                        Simufill_X = []
                                                        Simufill_Y = []
                                                        Simufill_frame_plot = []
                                                        Simufill_Kuang_size_w = []
                                                        Simufill_Kuang_size_l = []
                                                        Simufill_Kuang_size_h = []
                                                        Simufill_Angle = []
                                                        for loopin in range(5):
                                                            Simufill_X.append(
                                                                all_trk_ID_det_[i][loopin - 5][0])  # 存入x坐标
                                                            Simufill_Y.append(
                                                                all_trk_ID_det_[i][loopin - 5][1])  # 存入y坐标
                                                            Simufill_frame_plot.append(
                                                                all_trk_ID_det_[i][loopin - 5][13])  # 存入帧号
                                                            Simufill_Kuang_size_w.append(
                                                                all_trk_ID_det_[i][loopin - 5][2])
                                                            Simufill_Kuang_size_l.append(
                                                                all_trk_ID_det_[i][loopin - 5][3])
                                                            Simufill_Kuang_size_h.append(
                                                                all_trk_ID_det_[i][loopin - 5][6])
                                                            Simufill_Angle.append(
                                                                all_trk_ID_det_[i][loopin - 5][4])  # 存入角度
                                                        for loopin in range(3):
                                                            Simufill_X.append(all_trk_ID_det_[obj_][loopin][0])  # 存入x坐标
                                                            Simufill_Y.append(all_trk_ID_det_[obj_][loopin][1])  # 存入y坐标
                                                            Simufill_frame_plot.append(
                                                                all_trk_ID_det_[obj_][loopin][13])  # 存入帧号
                                                            Simufill_Kuang_size_w.append(
                                                                all_trk_ID_det_[obj_][loopin][2])
                                                            Simufill_Kuang_size_l.append(
                                                                all_trk_ID_det_[obj_][loopin][3])
                                                            Simufill_Kuang_size_h.append(
                                                                all_trk_ID_det_[obj_][loopin][6])
                                                            Simufill_Angle.append(
                                                                all_trk_ID_det_[obj_][loopin][4])  # 存入角度

                                                        TrackLinefillX = np.polyfit(Simufill_frame_plot, Simufill_X,
                                                                                    20)  # 20 最高阶数
                                                        TrackLinefillSimuX = np.poly1d(TrackLinefillX)

                                                        TrackLinefillY = np.polyfit(Simufill_frame_plot, Simufill_Y,
                                                                                    20)  # 20 最高阶数
                                                        TrackLinefillSimuY = np.poly1d(TrackLinefillY)

                                                        TrackLinefillhead = np.polyfit(Simufill_frame_plot,
                                                                                       Simufill_Angle,
                                                                                       20)  # 20 最高阶数
                                                        TrackLinefillSimuY = np.poly1d(TrackLinefillhead)

                                                        for fillnum_ in range(startfillpos, endfillpos):
                                                            trk = copy.deepcopy(all_trk_ID_det_[i][-1])
                                                            trk[0] = TrackLinefillX(fillnum_)  # x点模拟坐标，其他类似
                                                            trk[1] = TrackLinefillY(fillnum_)  # 拟合Y坐标
                                                            trk[4] = TrackLinefillhead(fillnum_)  # 拟合航向角
                                                            trk[13] = fillnum_
                                                            all_trk_ID_det[i].append(trk)
                                                            all_trk_fram[trk[13]].append(trk[9])
                                                    else:
                                                        head = math.acos((all_trk_ID_det_[obj_][0][0] -
                                                                          all_trk_ID_det_[i][-1][0]) / math.sqrt((
                                                                                                                             all_trk_ID_det_[
                                                                                                                                 obj_][
                                                                                                                                 0][
                                                                                                                                 0] -
                                                                                                                             all_trk_ID_det_[
                                                                                                                                 i][
                                                                                                                                 -1][
                                                                                                                                 0]) ** 2 + (
                                                                                                                             all_trk_ID_det_[
                                                                                                                                 obj_][
                                                                                                                                 0][
                                                                                                                                 1] -
                                                                                                                             all_trk_ID_det_[
                                                                                                                                 i][
                                                                                                                                 -1][
                                                                                                                                 1]) ** 2))
                                                        for fillnum_ in range(startfillpos, endfillpos):
                                                            trk = copy.deepcopy(all_trk_ID_det_[i][-1])
                                                            trk[0] = all_trk_ID_det_[i][-1][
                                                                         0] + averspeed * 0.1 * math.cos(head)
                                                            trk[1] = all_trk_ID_det_[i][-1][
                                                                         0] + averspeed * 0.1 * math.sin(head)
                                                            trk[4] = (270 - head * Rads_cov) % 360  # 拟合航向角
                                                            trk[13] = fillnum_
                                                            all_trk_ID_det[i].append(trk)
                                                            all_trk_fram[trk[13]].append(trk[9])

                                                for trkbox_ in range(len(all_trk_ID_det_[obj_])):
                                                    all_trk_ID_det_[obj_][trkbox_][9] = all_trk_ID_det_[i][-1][9]
                                                    all_trk_ID_det_[obj_][trkbox_][8] = 2
                                                    all_trk_ID_det[i].append(all_trk_ID_det_[obj_][trkbox_])
                                                del all_trk_ID_det[obj_]
    #
    if matchcut:
        SavecutoutBoxFun(matchcut, frameid)

def Foreloss_correct(all_trk_ID_det,all_trk_fram):
    all_trk_ID_det_ = copy.deepcopy(all_trk_ID_det)
    # 首端填补
    # ①根据策略查找符合的目标
    framematch = {}
    referobjs = {}
    foreobj = {}  # 查找在设定范围内认为是首端丢失的目标 格式 目标： 初始帧
    fillforeobj = {}
    adjoinpos = {}
    forefillblank = {}
    front_adjoin = {}
    rear_adjoin = {}
    for i in all_trk_ID_det_.keys():
        if all_trk_ID_det_[i][0][7] in [0, 1]:  # [1, 2]
            if all_trk_ID_det_[i][0][13] > 0:
                detrefer1 = math.sqrt(all_trk_ID_det_[i][0][0] ** 2 + all_trk_ID_det_[i][0][1] ** 2)
                detrefer2 = math.sqrt(all_trk_ID_det_[i][1][0] ** 2 + all_trk_ID_det_[i][1][1] ** 2)
                if (detrefer1 < 50) or (detrefer2 < 50):
                    print("发现有首端丢失的目标{},距离0：{}，距离1：{}".format(i, detrefer1, detrefer2))
                    foreobj[i] = all_trk_ID_det_[i][0][13]
                    det1 = math.sqrt((all_trk_ID_det_[i][1][0] - all_trk_ID_det_[i][0][0]) ** 2 + (
                                all_trk_ID_det_[i][1][1] - all_trk_ID_det_[i][0][1]) ** 2)
                    det2 = math.sqrt((all_trk_ID_det_[i][2][0] - all_trk_ID_det_[i][0][0]) ** 2 + (
                                all_trk_ID_det_[i][2][1] - all_trk_ID_det_[i][0][1]) ** 2)
                    framepos = all_trk_ID_det_[i][0][13]
                    fillboxcount = []
                    if det1 < 0.1 or det2 < 0.2:
                        framelist = []
                        boxindex = []
                        for frameobj in all_trk_ID_det_.keys():
                            if frameobj == i:
                                continue
                            for objnum_ in range(len(all_trk_ID_det_[frameobj])):
                                if all_trk_ID_det_[frameobj][objnum_][13] == framepos:
                                    if all_trk_ID_det_[frameobj][objnum_][7] not in [0, 1]:
                                        break
                                    if abs(all_trk_ID_det_[frameobj][objnum_][4] - all_trk_ID_det_[i][0][
                                        4]) % 360 < 90 or abs(
                                            all_trk_ID_det_[frameobj][objnum_][4] - all_trk_ID_det_[i][1][
                                                4]) % 360 > 270:
                                        refer_dis = math.sqrt(
                                            (all_trk_ID_det_[frameobj][objnum_][0] - all_trk_ID_det_[i][0][0]) ** 2 + (
                                                        all_trk_ID_det_[frameobj][objnum_][1] - all_trk_ID_det_[i][0][
                                                    1]) ** 2)
                                        if refer_dis < 12:
                                            refer_angle = math.asin(abs(
                                                all_trk_ID_det_[frameobj][objnum_][1] - all_trk_ID_det_[i][0][
                                                    1]) / math.sqrt((all_trk_ID_det_[frameobj][objnum_][0] -
                                                                     all_trk_ID_det_[i][0][0]) ** 2 + (
                                                                                all_trk_ID_det_[frameobj][objnum_][1] -
                                                                                all_trk_ID_det_[i][0][
                                                                                    1]) ** 2)) * Rads_cov
                                            refer_angle = (270 - refer_angle) % 360
                                            # if abs(refer_angle - all_trk_ID_det_[i][0][4] ) % 360 < 30 or (refer_angle - all_trk_ID_det_[i][0][4]  % 360) > 315 or (refer_angle - all_trk_ID_det_[i][0][4] % 360):
                                            #     framelist.append(frameobj)
                                            #     index = 0
                                            '''
                                            筛选同车道前后相邻的目标防止碾压碰撞2021111224
                                            '''
                                            if abs(refer_angle - all_trk_ID_det_[i][0][4]) % 360 < 10 or abs(
                                                    refer_angle - all_trk_ID_det_[i][0][
                                                        4]) % 360 > 350:  # 以10°为阈值作为前后相邻目标阈值
                                                front_adjoin.update({frameobj: objnum_})  # 找到前相邻的目标
                                            if abs(refer_angle - all_trk_ID_det_[i][0][4]) % 360 > 170 and abs(
                                                    refer_angle - all_trk_ID_det_[i][0][4]) % 360 < 190:
                                                rear_adjoin.update({frameobj: objnum_})  ##找到后相邻的目标
                                                '''
                                                =====================================================================2021112241116
                                                '''
                                            for count_ in range(0, objnum_):
                                                det = math.sqrt((all_trk_ID_det[frameobj][count_][0] -
                                                                 all_trk_ID_det[frameobj][objnum_][0]) ** 2 + (
                                                                        all_trk_ID_det[frameobj][count_][1] -
                                                                        all_trk_ID_det[frameobj][objnum_][
                                                                            1]) ** 2)
                                                if det < 10:
                                                    index = count_
                                                    break
                                            boxcount = all_trk_ID_det_[frameobj][objnum_][13] - \
                                                       all_trk_ID_det_[frameobj][index][13]
                                            fillboxcount.append(boxcount)
                                            boxindex.append(objnum_)
                                            break
                        if len(framelist) > 0:
                            referobjs[i] = framelist.copy()
                            framelist.clear()
                            fillforeobj[i] = max(fillboxcount) + 1  # 对应的目标需要补充的帧数
                            adjoinpos[i] = boxindex  #
                    #
                    referobjs_ = copy.deepcopy(referobjs)
                    if framepos in framematch.keys():
                        framematch[framepos].update(referobjs_)
                    else:
                        framematch[framepos] = referobjs_
                    referobjs.clear()
            # ②根据策略进行修补 V1.0
            # detrefer1 = []
            # #利用相邻的目标为参考找到补充的帧数
            #
            # for correct_ in framematch.keys():
            #     for correctframe_ in range(int(correct_)):
            #         correctframe = correct_ - 1 - correctframe_
            #         for obj_ in framematch[correct_].keys():
            #             objfeaturs = 0
            #             for adjacent_ in range(len(framematch[correct_][obj_])):  #相邻的目标
            #                 for framein_ in range(int(correct_)):
            #                     tempobj = framematch[correct_][obj_][adjacent_]
            #                     if framein_ < len(all_trk_ID_det_[tempobj]):
            #                         if all_trk_ID_det_[tempobj][framein_][13] == correctframe:
            #                             tempdata = all_trk_ID_det_[tempobj][framein_ + 1][0]
            #                             dettemp = math.sqrt((all_trk_ID_det_[tempobj][framein_+1][0] - all_trk_ID_det_[tempobj][framein_][0])**2 + (all_trk_ID_det_[tempobj][framein_+ 1][1] - all_trk_ID_det_[tempobj][framein_][1])**2)
            #                             detrefer1.append(dettemp)
            #             for objnum in range(len(detrefer1)):
            #                 if detrefer1[objnum] > 0.1:
            #                     objfeaturs = objfeaturs + 1 #周围目标在移动不好确定位置，只有相邻目标在静止的情况下才好确定位置
            #             if objfeaturs == 0 :
            #                 trk = copy.deepcopy(all_trk_ID_det_[obj_][0])
            #                 trk[13] = correctframe
            #                 all_trk_ID_det[obj_].append(trk)
            #                 all_trk_fram[trk[13]].append(trk[9])

            # ②根据策略进行修补 V1.1
    # 根据策略修补
    for corre_ in framematch.keys():
        for correobj_ in framematch[corre_].keys():
            forefillblank[correobj_] = []
            adjoindis = []  # 相邻目标移动的距离
            if fillforeobj[correobj_] < 60:  # 最多6秒
                index = 0
                for adjoinobj in framematch[corre_][correobj_]:
                    if index < len(adjoinpos[correobj_]):
                        framepos = adjoinpos[correobj_][index]
                        index = index + 1
                        for calcount in range(framepos):
                            det = math.sqrt((all_trk_ID_det_[adjoinobj][framepos - calcount][0] -
                                             all_trk_ID_det_[adjoinobj][framepos - calcount - 1][0]) ** 2 + (
                                                    all_trk_ID_det_[adjoinobj][framepos - calcount][1] -
                                                    all_trk_ID_det_[adjoinobj][framepos - calcount - 1][1]) ** 2)
                            adjoindis.append(det)
                if len(adjoindis) > 0:
                    thresholdvalue = [v for k, v in enumerate(adjoindis) if v < 1]
                    percentration = len(thresholdvalue) / len(adjoindis)
                    if percentration > 0.5:  # 1、整体保持近似静止，则车辆都在静止 一种隐患就是后面的会碾压这个还得修复 2、每次往前推断一帧，判断前后车道的车辆的运动状态再进行修补
                        framelist_ = []
                        for framepos in range(int(fillforeobj[correobj_])):
                            if all_trk_ID_det_[correobj_][0][13] - framepos > 0:
                                print("填充目标")
                                """
                                增加动态修补，每退后一一步，就以退后的帧所在的位置为基准，检测前后车的位置，根据前后车的动态进行修补202112241116
                                """
                                '''
                                for fronobj_ in front_adjoin.keys():
                                    trk = copy.deepcopy(all_trk_ID_det_[correobj_][0])
                                    cal_box = np.array([trk])
                                    backpos_ = front_adjoin[fronobj_] - framepos - 1
                                    if all_trk_ID_det_[fronobj_][backpos_]:
                                        if all_trk_ID_det_[fronobj_][backpos_][13] == all_trk_ID_det_[correobj_][0][13] - framepos -1:
                                            fit_box = np.array([all_trk_ID_det_[fronobj_][backpos_]])
                                            cal_iou = rotate_nms_cc(cal_box, fit_box)
                                            if cal_iou > 0.2:
                                                pass


                                '''

                                # ==================================================================================================
                                # ==================================================================================================202112241116增加动态修正
                                trk = copy.deepcopy(all_trk_ID_det_[correobj_][0])
                                trk[13] = all_trk_ID_det_[correobj_][0][13] - framepos - 1
                                trk[8] = 1
                                # all_trk_ID_det[correobj_].append(trk)
                                all_trk_ID_det[correobj_].insert(0, trk)

                                all_trk_fram[trk[13]].append(trk[9])
                                framelist_.append(trk[13])
                            else:
                                break
                        matchresult = copy.deepcopy(framelist_)
                        forefillblank.update({correobj_: matchresult})

    if framematch:
        SaveFillBoxFun(framematch, forefillblank)


def Lossframe_correcr(all_trk_ID_det,all_trk_fram):
    # #丢帧修补
    all_trk_ID_det_ = copy.deepcopy(all_trk_ID_det)
    frameloss = {}
    for i in all_trk_ID_det_.keys():
        if all_trk_ID_det_[i][0][7] in [0, 1]:  # [1, 2]
            for j in range(len(all_trk_ID_det_[i]) - 1):
                frame_dev = all_trk_ID_det_[i][j + 1][13] - all_trk_ID_det_[i][j][13]
                if frame_dev > 1:
                    diu_frame += 1
                    if i in frameloss.keys():
                        frameloss[i].append((all_trk_ID_det_[i][j + 1][13], all_trk_ID_det_[i][j][13]))
                    else:
                        frameloss.update({i: [(all_trk_ID_det_[i][j + 1][13], all_trk_ID_det_[i][j][13])]})

                    print("该轨迹存在丢帧！", all_trk_ID_det_[i][0][9], frame_dev)
                    '''
                    如果两帧之间距离小于一定的阈值或者前后帧距离很小，说明在缓慢行驶，或者等红灯zhaowei

                    '''
                    findframe = 0
                    Simu_X = []
                    Simu_Y = []
                    Simu_frame_plot = []
                    Simu_Kuang_size_w = []
                    Simu_Kuang_size_l = []
                    Simu_Kuang_size_h = []
                    Simu_Angle = []

                    if j > 10:  # 保证点数大于一定的个数，拟合出的轨迹有一定的规律
                        maxindex = j + 10
                        if (j + 10) >= (len(all_trk_ID_det_[i]) - 1):
                            maxindex = len(all_trk_ID_det_[i]) - 1

                        for loopin in range(j - 10, maxindex):
                            Simu_X.append(all_trk_ID_det_[i][loopin][0])  # 存入x坐标
                            Simu_Y.append(all_trk_ID_det_[i][loopin][1])  # 存入y坐标
                            Simu_frame_plot.append(all_trk_ID_det_[i][loopin][13])  # 存入帧号
                            Simu_Kuang_size_w.append(all_trk_ID_det_[i][loopin][2])
                            Simu_Kuang_size_l.append(all_trk_ID_det_[i][loopin][3])
                            Simu_Kuang_size_h.append(all_trk_ID_det_[i][loopin][6])
                            Simu_Angle.append(all_trk_ID_det_[i][loopin][4])  # 存入角度

                        # 拟合轨迹
                        TrackLineX = np.polyfit(Simu_frame_plot, Simu_X, 20)  # 20 最高阶数
                        TrackLineSimuX = np.poly1d(TrackLineX)

                        TrackLineY = np.polyfit(Simu_frame_plot, Simu_Y, 20)
                        TrackLineSimuY = np.poly1d(TrackLineY)

                        TrackLineAngel = np.polyfit(Simu_frame_plot, Simu_Angle, 20)
                        TrackLineSimuAngel = np.poly1d(TrackLineAngel)

                        for breakpointloop in range(int(all_trk_ID_det_[i][j][13]) + 1,
                                                    int(all_trk_ID_det_[i][j + 1][13])):
                            trk = copy.deepcopy(all_trk_ID_det_[i][j])
                            trk[0] = TrackLineSimuX(breakpointloop)  # x点模拟坐标，其他类似
                            trk[1] = TrackLineSimuY(breakpointloop)  # 拟合Y坐标
                            trk[4] = TrackLineSimuAngel(breakpointloop)  # 拟合航向角
                            trk[8] = 3
                            trk[13] = breakpointloop
                            all_trk_ID_det[i].append(trk)
                            if trk[13] in all_trk_fram:
                                all_trk_fram[trk[13]].append(trk[9])
                            else:
                                tempdata = []
                                tempdata.append(trk[9])
                                all_trk_fram[trk[13]] = tempdata

                        # Simu_frame_plot = range(int(all_trk_ID_det_[i][j][13]) + 1,
                        #                             int(all_trk_ID_det_[i][j + 1][13]))

                        # simuxdis = np.polyval(TrackLineX,Simu_frame_plot)
                        # simuydis = np.polyval(TrackLineY,Simu_frame_plot)
                        # simuangdis = np.polyval(TrackLineAngel,Simu_frame_plot)
                        # plt.plot(Simu_frame_plot,simuxdis,'s',label='SimuX')
                        # plt.plot(Simu_frame_plot,simuydis,'r',label='SimuY')
                        # plt.plot(Simu_frame_plot,simuangdis,'b',label=Simu_Angle)
                        # plt.show()
                    else:  # j < len(all_trk_ID_det_[i])-1
                        SpaceFrame = all_trk_ID_det_[i][j + 1][13] - all_trk_ID_det_[i][j][13]
                        SpaceLenth = math.sqrt((all_trk_ID_det_[i][j + 1][0] - all_trk_ID_det_[i][j][0]) ** 2 + (
                                    all_trk_ID_det_[i][j + 1][1] - all_trk_ID_det_[i][j][1]) ** 2)
                        EstimateSpeed_1 = math.sqrt(
                            (all_trk_ID_det_[i][j + 2][0] - all_trk_ID_det_[i][j + 1][0]) ** 2 + (
                                        all_trk_ID_det_[i][j + 2][1] - all_trk_ID_det_[i][j + 1][1]) ** 2) / (
                                                      all_trk_ID_det_[i][j + 2][13] - all_trk_ID_det_[i][j + 1][13])
                        EstimateSpeed_2 = math.sqrt(
                            (all_trk_ID_det_[i][j + 1][0] - all_trk_ID_det_[i][j][0]) ** 2 + (
                                    all_trk_ID_det_[i][j + 1][1] - all_trk_ID_det_[i][j][1]) ** 2) / (
                                                  all_trk_ID_det_[i][j + 1][13] - all_trk_ID_det_[i][j][13])
                        EstimateSpeed = (EstimateSpeed_2 + EstimateSpeed_1) / 2
                        SpaceAngle = abs(all_trk_ID_det_[i][j + 1][4] - all_trk_ID_det_[i][j][4])
                        Estimate_AngleSpeed = ((all_trk_ID_det_[i][j + 2][4] - all_trk_ID_det_[i][j + 1][4]) / (
                                    all_trk_ID_det_[i][j + 2][13] - all_trk_ID_det_[i][j + 1][13]) + (
                                                           all_trk_ID_det_[i][j + 1][4] - all_trk_ID_det_[i][j][4]) / (
                                                           all_trk_ID_det_[i][j + 1][13] - all_trk_ID_det_[i][j][
                                                       13])) / 2

                        for spacepos in range(int(all_trk_ID_det_[i][j][13]) + 1, int(all_trk_ID_det_[i][j + 1][13])):
                            trk = copy.deepcopy(all_trk_ID_det_[i][j])
                            trk[4] = trk[4] + Estimate_AngleSpeed * (spacepos - (all_trk_ID_det_[i][j][13] + 1))
                            if abs(trk[4] - all_trk_ID_det_[i][j][4]) >= SpaceAngle:
                                spaceaver = (abs(trk[4] - all_trk_ID_det_[i][j][4]) - SpaceAngle) / (
                                            all_trk_ID_det_[i][j + 1][13] - spacepos)
                                trk[4] = trk[4] - spaceaver

                            trk[0] = trk[0] + EstimateSpeed * math.cos((270 - trk[4]) * PI_rads) * (
                                    spacepos - (all_trk_ID_det_[i][j][13] + 1))
                            trk[1] = trk[1] + EstimateSpeed * math.sin((270 - trk[4]) * PI_rads) * (
                                    spacepos - (all_trk_ID_det_[i][j][13] + 1))
                            trk[8] = 3
                            trk[13] = spacepos
                            if math.sqrt((trk[0] - all_trk_ID_det_[i][j][0]) ** 2 + (
                                    trk[1] - all_trk_ID_det_[i][j][1]) ** 2) >= SpaceLenth:
                                lenthever = (math.sqrt((trk[0] - all_trk_ID_det_[i][j][0]) ** 2 + (
                                            trk[1] - all_trk_ID_det_[i][j][1]) ** 2) - SpaceLenth) / (
                                                        all_trk_ID_det_[i][j + 1][13] - spacepos)
                                trk[0] = trk[0] - abs(lenthever * math.cos((270 - trk[4]) * PI_rads))
                                trk[1] = trk[1] - abs(lenthever * math.cos((270 - trk[4]) * PI_rads))
                            all_trk_ID_det[i].append(trk)

                    # =====================================================================

                    # x_dev1 = (all_trk_ID_det_[i][j][0] - all_trk_ID_det_[i][j - 1][0])
                    # y_dev1 = (all_trk_ID_det_[i][j][1] - all_trk_ID_det_[i][j - 1][1])
                    #
                    # x_dev2 = (all_trk_ID_det_[i][j + 1][0] - all_trk_ID_det_[i][j + 2][0])
                    # y_dev2 = (all_trk_ID_det_[i][j + 1][1] - all_trk_ID_det_[i][j + 2][1])
                    # D2 = math.sqrt(x_dev2 * x_dev2 + y_dev2 * y_dev2)#后
                    # D1 = math.sqrt(x_dev1 * x_dev1 + y_dev1 * y_dev1)#前
                    #
                    # x_dev = (all_trk_ID_det_[i][j + 1][0] - all_trk_ID_det_[i][j][0]) / frame_dev
                    # y_dev = (all_trk_ID_det_[i][j + 1][1] - all_trk_ID_det_[i][j][1]) / frame_dev
                    #
                    # if D1 < 0.1 and D2 < 0.1:#动静判断
                    #     for k in range(int(frame_dev)):
                    #         trk = copy.deepcopy(all_trk_ID_det_[i][j])
                    #         if k == 0:
                    #             continue
                    #         trk[0] = trk[0]
                    #         trk[1] = trk[1]
                    #         trk[11] = 300
                    #         trk[13] = all_trk_ID_det_[i][j][13] + k
                    #         all_trk_ID_det[i].append(trk)
                    #         all_trk_fram[trk[13]].append(i)
                    # if D1 > 0.1 and D2 > 0.1:#动静判断
                    #     for k in range(int(frame_dev)):
                    #         trk = copy.deepcopy(all_trk_ID_det_[i][j])
                    #         if k == 0:
                    #             continue
                    #         trk[0] = trk[0] + k * x_dev
                    #         trk[1] = trk[1] + k * y_dev
                    #         trk[11] = 300
                    #         trk[13] = all_trk_ID_det_[i][j][13] + k
                    #         all_trk_ID_det[i].append(trk)
                    #         all_trk_fram[trk[13]].append(i)
                    # if D1 > 0.1 and D2 < 0.1:  # 动静判断
                    #     for k in range(int(frame_dev)):
                    #         trk = copy.deepcopy(all_trk_ID_det_[i][j + 1])
                    #         if k == 0:
                    #             continue
                    #         trk[0] = trk[0]
                    #         trk[1] = trk[1]
                    #         trk[11] = 300
                    #         trk[13] = all_trk_ID_det_[i][j][13] + k
                    #         all_trk_ID_det[i].append(trk)
                    #         all_trk_fram[trk[13]].append(i)
                    # if D1 < 0.1 and D2 > 0.1:  # 动静判断
                    #     for k in range(int(frame_dev)):
                    #         trk = copy.deepcopy(all_trk_ID_det_[i][j])
                    #         if k == 0:
                    #             continue
                    #         trk[0] = trk[0]
                    #         trk[1] = trk[1]
                    #         trk[11] = 300
                    #         trk[13] = all_trk_ID_det_[i][j][13] + k
                    #         all_trk_ID_det[i].append(trk)
                    #         all_trk_fram[trk[13]].append(i)
    if frameloss:
        SaveframelossFun(frameloss)