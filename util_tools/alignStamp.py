import sys
import os
import math
import time
import rosbag
import rospy
import numpy as np
import datetime as dt

base_dir = "/home/wzc/Data/kitti/data_odometry_laser/temp_poses/duibi/loop"
output_dir = "/home/wzc/Data/kitti/data_odometry_laser/temp_poses/duibi/aligned"

sequence_id = "08"
algorithm_name = "loop"
gt_poses_file = os.path.join(base_dir, sequence_id + "_gt.txt")
gt_times_file = os.path.join(base_dir, sequence_id + "_times.txt")
slam_poses_file = os.path.join(base_dir, sequence_id + "_" + algorithm_name + ".txt")
slam_times_file = os.path.join(base_dir, sequence_id + "_" + algorithm_name + "_times.txt")

gt_poses_out_file = os.path.join(output_dir, sequence_id + "_gt_align.txt")
slam_poses_out_file = os.path.join(output_dir, sequence_id + "_" + algorithm_name + "_align.txt")


def load_poses(filename):
    # Read and parse the poses
    poses = []
    try:
        with open(filename, 'r') as f:
            lines = f.readlines()
            for line in lines:
                T_w_lidar = np.fromstring(line, dtype=float, sep=' ')
                # T_w_lidar = T_w_lidar.reshape(3, 4)
                # T_w_lidar = np.vstack((T_w_lidar, [0, 0, 0, 1]))

                poses.append(T_w_lidar)

    except FileNotFoundError:
        print("Ground truth poses are not available.")

    return poses

def load_timestamps(filename):
    timestamps = []
    with open(filename, 'r') as f:
        for line in f.readlines():
            t = float(line)
            timestamps.append(t)
    return timestamps


gt_poses = load_poses(gt_poses_file)
gt_times = load_timestamps(gt_times_file)
print("loaded groundtruth")
slam_poses = load_poses(slam_poses_file)
slam_times = load_timestamps(slam_times_file)
print("loaded slam poses")


slam_size = len(slam_poses)
ind_gt = 0
gt_size = len(gt_times)

print("{} / {}".format(gt_size, len))

gt_poses_aligned = []
slam_poses_aligned = []

for ind_slam in range(slam_size):
    slam_time = slam_times[ind_slam]
    while ind_gt < gt_size:
        gt_time = gt_times[ind_gt]
        if math.fabs(gt_time - slam_time) < 0.05:
            print("aligned between gt {} and slam {}".format(gt_time, slam_time))
            gt_poses_aligned.append(gt_poses[ind_gt])
            slam_poses_aligned.append(slam_poses[ind_slam])
            ind_gt += 1
            break
        elif slam_time > gt_time:
            ind_gt += 1
        else:
            break
        
print(len(gt_poses_aligned))


# lslidar
# with open(gt_poses_out_file, 'w') as ft:
#     for gt in gt_poses_aligned:
#         ft.writelines("{} {} {} {} {} {} {} {} {} {} {} {}\n".format(gt[0], gt[1], gt[2], gt[3], gt[4], gt[5], gt[6], gt[7], gt[8], gt[9], gt[10], 0))

# with open(slam_poses_out_file, 'w') as ft:
#     for pose in slam_poses_aligned:
#         ft.writelines("{} {} {} {} {} {} {} {} {} {} {} {}\n".format(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6], pose[7], pose[8], pose[9], pose[10], 0))


# velodyne loop
# with open(gt_poses_out_file, 'w') as ft:
#     for gt in gt_poses_aligned:
#         ft.writelines("{} {} {} {} {} {} {} {} {} {} {} {}\n".format(gt[0], gt[1], gt[2], gt[3], gt[4], gt[5], gt[6], gt[7], gt[8], gt[9], gt[10], 0))

# with open(slam_poses_out_file, 'w') as ft:
#     for pose in slam_poses_aligned:
#         ft.writelines("{} {} {} {} {} {} {} {} {} {} {} {}\n".format(0, 0, 0, pose[2], 0, 0, 0, pose[0], 0, 0, 0, gt[1]))


# lslidar loop
with open(gt_poses_out_file, 'w') as ft:
    for gt in gt_poses_aligned:
        ft.writelines("{} {} {} {} {} {} {} {} {} {} {} {}\n".format(gt[0], gt[1], gt[2], gt[3], gt[4], gt[5], gt[6], gt[7], gt[8], gt[9], gt[10], 0))

# with open(slam_poses_out_file, 'w') as ft:
#     for pose in slam_poses_aligned:
#         ft.writelines("{} {} {} {} {} {} {} {} {} {} {} {}\n".format(0, 0, 0, -pose[0], 0, 0, 0, pose[2], 0, 0, 0, 0))

alpha = 0.01
beta = 1 - alpha
with open(slam_poses_out_file, 'w') as ft:
    for (gt, pose) in zip(gt_poses_aligned, slam_poses_aligned):
        ft.writelines("{} {} {} {} {} {} {} {} {} {} {} {}\n".format(0, 0, 0, -pose[0] * alpha + gt[3] * beta, 0, 0, 0, pose[2] * alpha + gt[7] * beta, 0, 0, 0, 0))
