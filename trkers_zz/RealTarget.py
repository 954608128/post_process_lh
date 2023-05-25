
class RealTargetInfo:
    def __init__(self):
        self.target_id = 0                  # 真值目标ID
        self.dict_realtestmatch = {}        # 真值与待测ID匹配关系
        self.matchNum = 0                   # 总匹配数
        self.type = 0                       # 目标类型
        self.track_type = ''                # 跟踪情况
        self.b_track_success = False        # 跟踪成功标志
        self.real_frame_num = 0             # 真值目标帧数
        self.list_test_id = []              # 匹配的待测目标ID
        self.list_test_type = []            # 匹配的待测目标类型
