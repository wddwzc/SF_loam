#!env python
# -*- coding: utf-8 -*-

import sys

import tf
import os
import cv2
import rospy
import rosbag
import progressbar
import math
import matplotlib as mpl
import matplotlib.pyplot as plt
import glob
from tf2_msgs.msg import TFMessage
from datetime import datetime
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo, Imu, PointField, NavSatFix
import sensor_msgs.point_cloud2 as pcl2
from geometry_msgs.msg import TransformStamped, TwistStamped, Transform, Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import numpy as np
import argparse



def plotIntensity(x, y):
    # x = np.array(data_x)
    # y = np.array(data_y)
    print(len(x))
    # print(y)
    plt.figure(figsize=(7,5))
    plt.scatter(x, y, marker='o')
    plt.grid(True)
    plt.show()

def record(velo_files, output_file):
    print("Exporting velodyne data")
    print(len(velo_files))
    intensity_list = []
    drive_id = 0
    counts = 0
    for filename in velo_files:
        print(drive_id)
        scan = (np.fromfile(filename, dtype=np.float32)).reshape(-1, 4)
        for p in scan:
            if counts == 1000:
                r = math.sqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2])
                R_ref = 1.0
                intensity_list.append([r, p[3] * r * r / (R_ref * R_ref)])
                # print(p)
                counts = 0
            counts += 1
        drive_id += 1

    return intensity_list



def main():
    # data_dir = "/home/wzc/Data/kitti/data_odometry_laser/sequences"
    data_dir = "/media/wzc/27f38a77-e29e-4c11-b3fd-0d7d02745eb2/home/wzc/catkin_ws/sequences"
    sequence = "31"
    velo_dir = os.path.join(data_dir, sequence, 'velodyne')
    print(velo_dir)
    output_dir = data_dir
    output_file = os.path.join(output_dir, 'intensity.bin')

    if not os.path.exists(velo_dir):
        print('Path {} does not exists. Exiting.'.format(velo_dir))
        sys.exit(1)

    velo_files = sorted(glob.glob(os.path.join(velo_dir, '*.bin')))
    data_ri = record(velo_files, output_file)
    plotIntensity([a[0] for a in data_ri], [b[1] for b in data_ri])
    print("plot over")


if __name__ == '__main__':
    main()
