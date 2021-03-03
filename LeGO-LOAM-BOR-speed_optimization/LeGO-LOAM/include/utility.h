#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_


#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include "cloud_msgs/cloud_info.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

typedef pcl::PointXYZI  PointType;

typedef Eigen::Vector3f Vector3;

const double DEG_TO_RAD = M_PI / 180.0;


struct smoothness_t{ 
    float value;
    size_t ind;
};

struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

struct ProjectionOut
{
  pcl::PointCloud<PointType>::Ptr segmented_cloud;
  pcl::PointCloud<PointType>::Ptr outlier_cloud;
  cloud_msgs::cloud_info seg_msg;
};


struct AssociationOut
{
  pcl::PointCloud<PointType>::Ptr cloud_outlier_last;
  pcl::PointCloud<PointType>::Ptr cloud_corner_last;
  pcl::PointCloud<PointType>::Ptr cloud_surf_last;
  nav_msgs::Odometry laser_odometry;
};

inline void OdometryToTransform(const nav_msgs::Odometry& odometry,
                                float* transform) {
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odometry.pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w))
      .getRPY(roll, pitch, yaw);

  transform[0] = -pitch;
  transform[1] = -yaw;
  transform[2] = roll;

  transform[3] = odometry.pose.pose.position.x;
  transform[4] = odometry.pose.pose.position.y;
  transform[5] = odometry.pose.pose.position.z;
}

/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time)
)

typedef PointXYZIRPYT  PointTypePose;

#endif
