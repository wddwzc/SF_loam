#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#define PCL_NO_PRECOMPILE

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "cloud_msgs/cloud_info.h"

#include <opencv/cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
// #include <pcl/common/impl/transforms.hpp>

#include <pcl/common/transforms.h>

#include <pcl/registration/icp.h>
#include <pcl/segmentation/extract_clusters.h>

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

// use the KITTI Semantic data, XYZL
// #define SEMANTIC_KITTI

#define PI_M 3.14159265

using namespace std;


// -24.9
// 2
// 64

    // if (angle > -8.7) {
    //   scanID = int(63-fabs(angle-2)*3+0.5);
    // } else {
    //   scanID = int(0+(fabs(angle+24.87)*2+0.5));//velodyne 64
    // }
    // //cout<<scanID<<",";
    //  /*if (angle < 0)
    //  {
    //    scanID--;
    //  }*/

    //  if (scanID>63||scanID<0)
    //  {

    //    continue;
    //  }

// extern const int systemDelay = 0;

// extern const int edgeFeatureNum = 2;
// extern const int surfFeatureNum = 4;
// extern const int sectionsTotal = 6;

struct smoothness_t{ 
    float value;
    size_t ind;
};

struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

/*
    * A point cloud type that has "ring" channel
    */
struct PointXYZIR
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIR,  
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (uint16_t, ring, ring)
)

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

typedef PointXYZIRPYT PointTypePose;



/*
    * A point cloud type that has intensity and class label.
	*/
struct PointXYZIL
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
	uint32_t label;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIL,
                                   (float, x, x)
								   (float, y, y)
                                   (float, z, z)
								   (float, intensity, intensity)
                                   (uint32_t, label, label)
)

typedef PointXYZIL PointSemantic;
typedef PointXYZIL PointType;
// typedef pcl::PointXYZI PointType;


#endif
