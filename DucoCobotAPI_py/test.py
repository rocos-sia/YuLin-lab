import logging
import time
import numpy as np

import Translate as T

def get_T(pose1, pose2):
    """
    获得过渡矩阵
    pose1:视觉定位点位姿
    pose2:目标点位姿
    返回 过渡矩阵T
    """
    pose1_T = T.rpy2T(pose1)  # 视觉定位点位姿
    pose2_T = T.rpy2T(pose2)  # 目标点位姿
    offset = np.dot(np.linalg.inv(pose1_T), pose2_T)
    # transition=T.T2rot(transition_T)
    return offset


# aruco = [-0.5987771153450012, -0.7512031197547913, 0.21273213624954224, 1.5639138221740723, 0.0018499377183616161,
#          0.005866547580808401]
# A4 = [-0.6633021831512451, -0.8880190253257751, 0.17243138360977173, 1.572665810585022, 0.0003740479296538979,
#       -1.252971887588501]
# A4 = get_T(aruco, A4)
# print("A4", A4)
logging.basicConfig(filename='robot_Log',level=logging.NOTSET,
                    format='%(asctime)s %(filename)s %(levelname)s %(message)s',
                    datefmt='%a %d %b %Y %H:%M:%S',
                    filemode='a'
                    )
print('2')
time.sleep(3)
print('22')
