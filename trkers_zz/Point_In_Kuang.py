import math

# 计算一个点是否属于矩形框内 1----3
                    #   2----4
def Point_In_kuang(x, y, p1, p2, p3, p4):
    #第一部分
    p_x_1 = p1[0] - x
    p_y_1 = p1[1] - y
    P_2_x_1 = p1[0] - p2[0]
    P_2_y_1 = p1[1] - p2[1]

    p_x_3 = p3[0] - x
    p_y_3 = p3[1] - y
    p_4_x_3 = p3[0] - p4[0]
    p_4_y_3 = p3[1] - p4[1]

    #第二部分
    P_3_x_1 = p1[0] - p3[0]
    P_3_y_1 = p1[1] - p3[1]

    p_x_2 = p2[0] - x
    p_y_2 = p2[1] - y

    p_4_x_2 = p2[0] - p4[0]
    p_4_y_2 = p2[1] - p4[1]

    q1 = p_x_1 * P_2_y_1 - p_y_1 * P_2_x_1
    q2 = p_x_3 * p_4_y_3 - p_y_3 * p_4_x_3

    q3 = p_x_1 * P_3_y_1 - p_y_1 * P_3_x_1
    q4 = p_x_2 * p_4_y_2 - p_y_2 * p_4_x_2

    q5 = q1 * q2
    q6 = q3 * q4
    if q5 < 0 and q6 < 0:
        return True
    else:
        return False