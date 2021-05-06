#!env python
# -*- coding: utf-8 -*-

import sys
import os
import time
import rosbag
import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pcl2
try:
    from cStringIO import StringIO
except ImportError:
    from io import StringIO

bag_dir = "/media/wzc/27f38a77-e29e-4c11-b3fd-0d7d02745eb2/home/wzc/catkin_ws"
bag_id = "45"
bag_name = os.path.join(bag_dir, bag_id + ".bag")

output_base = os.path.join(bag_dir, "trans_data")
output_dir = os.path.join(output_base, bag_id)
lidar_dir = os.path.join(output_dir, "velodyne")
lidar_id = 0
lidar_file = os.path.join(lidar_dir, "{0:06d}.bin".format(lidar_id))
times_file = os.path.join(output_dir, "times.txt")

if not os.path.exists(output_dir):
    os.makedirs(output_dir)
if not os.path.exists(lidar_dir):
    os.makedirs(lidar_dir)

topics = [
    # "/gps_navi",
    # "/imu/data",
    "/lslidar_point_cloud"
]
bag = rosbag.Bag(bag_name)


time_begin = None
timestamps = []

for topic, msg, t in bag.read_messages(topics = topics):

    header = msg.header
    header_seq = header.seq
    stamp_sec = header.stamp.secs
    stamp_nsec = header.stamp.nsecs
    frame_id = header.frame_id
    
    if (topic == str('/lslidar_point_cloud')):
        lidar_file = os.path.join(lidar_dir, "{0:06d}.bin".format(lidar_id))

        point_list = pcl2.read_points_list(msg, skip_nans=True)
        point_array = np.array(point_list, dtype=np.float32)
        point_array.tofile(lidar_file)

        # read
        # point_array_new = np.fromfile(lidar_file, dtype=np.float32).reshape(-1, 4)
        # print(point_array_new.shape)

        
        cur_time = header.stamp.to_sec()
        if time_begin == None:
            time_begin = cur_time
        cur_time = cur_time - time_begin
        timestamps.append(cur_time)

        # print('scans: {}--{}--{}'.format(lidar_id, point_array.shape, cur_time))
        print('scans: {}--{}'.format(lidar_id, cur_time))

        lidar_id += 1

bag.close()



with open(times_file, 'w') as ft:
    for time in timestamps:
        ft.writelines("{}\n".format(time))
        # ft.write(str(time))











# if sys.getdefaultencoding() != 'utf-8':
#     reload(sys)
#     sys.setdefaultencoding('utf-8')

# dst_dir = "/media/wzc/27f38a77-e29e-4c11-b3fd-0d7d02745eb2/home/wzc/catkin_ws"
# bag_name = "2021-01-16-DianQiLou.bag"
 
# if dst_dir[-1] == '/':
#     img_name = dst_dir + "{:0>10d}.jpg"
#     imu_info_name = dst_dir + "IMUInformation.txt"
#     info_name = dst_dir + "Information.txt"

# else :
#     img_name = dst_dir + "/{:0>10d}.jpg"
#     imu_info_name = dst_dir + "/IMUInformation.txt"
#     info_name = dst_dir + "/Information.txt"
 
# topics = ["/inspva_info",
#     "/vehicle_speed",
#     "/imu_data",
#     "/camera/compressed"
# ]
# bag = rosbag.Bag(bag_name)

# picture_cnt = 0
# imu_infos = []
# infos = []
# front_radar_infos = []
# left_radar_infos = []
# right_radar_infos = []
 
# front_x_data = [None] * 64
# front_y_data = [None] * 64
 
# left_x_data = [None] * 64
# left_y_data = [None] * 64
 
# right_x_data = [None] * 64
# right_y_data = [None] * 64
 
 
# for topic, msg, t in bag.read_messages(topics = topics): #print(topic)# print(msg.header)
  
#     header = msg.header
#     header_seq = header.seq
#     stamp_sec = header.stamp.secs
#     stamp_nsec = header.stamp.nsecs
#     frame_id = header.frame_id
 
#     #parse topic inspva_infos
#     if (topic == str('/inspva_info')): #inspva info 
#         time_stamp = msg.time_stamp
#         latitude = msg.latitude
#         longitude = msg.longitude
#         height = msg.height
#         vehicle_x = msg.vehicle_x
#         vehicle_y = msg.vehicle_y
#         roll = msg.roll
#         pitch = msg.pitch
#         azimuth = msg.azimuth
#         acc_x = msg.acc_x
#         acc_y = msg.acc_y
#         acc_z = msg.acc_z
#         valid = msg.valid# Bool#
#     #parse topic vehicle_speed
#     if (topic == str('/vehicle_speed')): #vehicle speed# sys_time_us = msg.sys_time_us
#         vehicle_speed_isvalid = msg.vehicle_speed_isvalid# bool
#         vehicle_speed = msg.vehicle_speed#
#     #parse topic imu_data
#     if (topic == str('/imu_data')): #imu_data# sys_time_us = msg.sys_time_us
#         measurement_time = msg.measurement_time
#         status = msg.status
#         measurement_span = msg.measurement_span
#         linear_acceleration_x = msg.linear_acceleration_x
#         linear_acceleration_y = msg.linear_acceleration_y
#         linear_acceleration_z = msg.linear_acceleration_z
#         linear_acceleration_covariance = msg.linear_acceleration_covariance# tuple
#         angular_velocity_x = msg.angular_velocity_x
#         angular_velocity_y = msg.angular_velocity_y
#         angular_velocity_z = msg.angular_velocity_z
#         angular_velocity_covariance = msg.angular_velocity_covariance# tuple
#         orientation_pitch = msg.orientation_pitch
#         orientation_roll = msg.orientation_roll
#         orientation_yaw = msg.orientation_yaw
#         orientation_quternion_1 = msg.orientation_quternion_1
#         orientation_quternion_2 = msg.orientation_quternion_2
#         orientation_quternion_3 = msg.orientation_quternion_3
#         orientation_quternion_4 = msg.orientation_quternion_4
#         orientation_covariance = msg.orientation_covariance# tuple
#     #parse topic camara
#     if (topic == '/camera/compressed'): 
#         im_format = msg.format
#         data = msg.data
#         def imgmsg_to_pil(img_msg, rgba = True):
#             pil_img = Image.open(StringIO(img_msg))
#             if pil_img.mode != 'L':
#                 pil_img.convert("RGB")
#             return pil_img
#         image = imgmsg_to_pil(msg.data)
#         image.save(img_name.format(picture_cnt))
 
#     ## dump info when image
#     if (topic == '/camera/compressed'): 
#         dum_str = "{:0>10d} {} {} {} {} {} {} {} {} {} {} {}".format(picture_cnt,
#                                      header_seq,
#                                      stamp_sec,
#                                      stamp_nsec,
#                                      latitude,
#                                      longitude,
#                                     # vehicle_speed,
#                                      roll,
#                                      pitch,
#                                      azimuth,
#                                      acc_z,
#                                      acc_y,
#                                      acc_x)# print(picture_cnt)
           
#         print (dum_str)
#         infos.append(dum_str)
#         #except:
#         #    pass
#     picture_cnt += 1
    
# with open(info_name, "w") as fp:
#     for it in infos:
#         fp.write(it + "\n")
# bag.close()
