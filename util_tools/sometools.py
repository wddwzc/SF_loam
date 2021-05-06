#!env python
# -*- coding: utf-8 -*-

import numpy as np
import math

def rotationMatrixToEulerAngles(RM):
    # sy = math.sqrt(RM[0,0] * RM[0,0] + RM[1,0] * RM[1,0])
    sy = math.sqrt(RM[2,1] * RM[2,1] + RM[2,2] * RM[2,2])
    singular = sy < 1e-6
    if not singular:
        r = math.atan2(RM[2,1], RM[2,2])
        p = math.atan2(-RM[2,0], sy)
        y = math.atan2(RM[1,0], RM[0,0])
    else:
        r = math.atan2(RM[1,2], RM[1,1])
        p = math.atan2(-RM[2,0], sy)
        y = 0
        # r = math.atan2(RM[2,1], RM[2,2])
        # p = math.atan2(-RM[2,0], sy)
        # y = math.atan2(RM[1,0], RM[0,0])

    return np.array([r, p, y])


def eulerAnglesToRotationMatrix(theta):
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(theta[0]), -math.sin(theta[0])],
                    [0, math.sin(theta[0]), math.cos(theta[0])]
                  ])
    R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                    [0, 1, 0],
                    [-math.sin(theta[1]), 0, math.cos(theta[1])]
                  ])
    R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                    [math.sin(theta[2]), math.cos(theta[2]), 0],
                    [0, 0, 1]
                  ])
    R_m = np.dot(R_z, np.dot(R_y, R_x))
    return R_m


