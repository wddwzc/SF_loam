#!env python
# -*- coding: utf-8 -*-

import sys
import os
import math
import time
import rosbag
import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pcl2
try:
    from cStringIO import StringIO
except ImportError:
    from io import StringIO


bag_name = "/media/wzc/27f38a77-e29e-4c11-b3fd-0d7d02745eb2/home/wzc/catkin_ws/43.bag"
lidar_id = 0

topics = [
    # "/gps_navi",
    # "/imu/data",
    "/lslidar_point_cloud"
]
bag = rosbag.Bag(bag_name)

range_list = []
x_list = []
y_list = []
z_list = []
remission_list = []
range_std_list = []
x_std_list = []
y_std_list = []
z_std_list = []
remission_std_list = []

for topic, msg, t in bag.read_messages(topics = topics):

    if (topic == str('/lslidar_point_cloud')):
        point_list = pcl2.read_points_list(msg, skip_nans=True)
        point_array = np.array(point_list, dtype=np.float32)
        
        range_temp = []
        x_temp = []
        y_temp = []
        z_temp = []
        remission_temp = []

        for line in point_array:
            range_temp.append(math.sqrt(line[0]*line[0]+line[1]*line[1]+line[2]*line[2]))
            x_temp.append(line[0])
            y_temp.append(line[1])
            z_temp.append(line[2])
            remission_temp.append(line[3])

        print('scans: {}'.format(lidar_id))

        lidar_id += 1

        range_array_temp = np.array(range_temp)
        x_array_temp = np.array(x_temp)
        y_array_temp = np.array(y_temp)
        z_array_temp = np.array(z_temp)
        remission_array_temp = np.array(remission_temp)

        range_list.append(range_array_temp.mean())
        x_list.append(x_array_temp.mean())
        y_list.append(y_array_temp.mean())
        z_list.append(z_array_temp.mean())
        remission_list.append(remission_array_temp.mean())

        range_std_list.append(range_array_temp.std())
        x_std_list.append(x_array_temp.std())
        y_std_list.append(y_array_temp.std())
        z_std_list.append(z_array_temp.std())
        remission_std_list.append(remission_array_temp.std())

bag.close()

print("read finished")
range_array = np.array(range_list)
x_array = np.array(x_list)
y_array = np.array(y_list)
z_array = np.array(z_list)
remission_array = np.array(remission_list)

range_std_array = np.array(range_std_list)
x_std_array = np.array(x_std_list)
y_std_array = np.array(y_std_list)
z_std_array = np.array(z_std_list)
remission_std_array = np.array(remission_std_list)

print("means")
print(range_array.mean())
print(x_array.mean())
print(y_array.mean())
print(z_array.mean())
print(remission_array.mean())
print("stds")
print(range_std_array.mean())
print(x_std_array.mean())
print(y_std_array.mean())
print(z_std_array.mean())
print(remission_std_array.mean())