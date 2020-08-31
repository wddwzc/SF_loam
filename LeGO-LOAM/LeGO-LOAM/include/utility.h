#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

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
#include <unordered_map>

#include "dbscan.h"

// use the KITTI Semantic data, XYZL
#define SEMANTIC_KITTI
// #define DEBUG_CLUSTER
#define DEBUG_PUBLISH
#define DEBUG_LOOPCLOSURE
#define TIME_COST

#define PI 3.14159265

using namespace std;

typedef pcl::PointXYZI  PointType;

#ifdef SEMANTIC_KITTI
extern const string pointCloudTopic = "/kitti/velo/label";
#else
extern const string pointCloudTopic = "/velodyne_points";
#endif // SEMANTIC_KITTI
extern const string imuTopic = "/imu/data";

// Save pcd
extern const string fileDirectory = "/tmp/";

#ifdef SEMANTIC_KITTI
extern const bool useCloudRing = false;
#else
// Using velodyne cloud "ring" channel for image projection (other lidar may have different name for this channel, change "PointXYZIR" below)
extern const bool useCloudRing = true; // if true, ang_res_y and ang_bottom are not used
#endif

// VLP-16
/*
extern const int N_SCAN = 16;
extern const int Horizon_SCAN = 1800;
extern const float ang_res_x = 0.2;
extern const float ang_res_y = 2.0;
extern const float ang_bottom = 15.0+0.1;
extern const int groundScanInd = 7;
*/

// VLP-64
extern const int N_SCAN = 64;
extern const int Horizon_SCAN = 2250;
extern const float ang_res_x = 0.16;
extern const float ang_res_y = 0.472;
extern const float ang_bottom = 24.9;
extern const int groundScanInd = 50;

// HDL-32E
// extern const int N_SCAN = 32;
// extern const int Horizon_SCAN = 1800;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 41.33/float(N_SCAN-1);
// extern const float ang_bottom = 30.67;
// extern const int groundScanInd = 20;

// VLS-128
// extern const int N_SCAN = 128;
// extern const int Horizon_SCAN = 1800;
// extern const float ang_res_x = 0.2;
// extern const float ang_res_y = 0.3;
// extern const float ang_bottom = 25.0;
// extern const int groundScanInd = 10;

// Ouster users may need to uncomment line 159 in imageProjection.cpp
// Usage of Ouster imu data is not fully supported yet (LeGO-LOAM needs 9-DOF IMU), please just publish point cloud data
// Ouster OS1-16
// extern const int N_SCAN = 16;
// extern const int Horizon_SCAN = 1024;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 33.2/float(N_SCAN-1);
// extern const float ang_bottom = 16.6+0.1;
// extern const int groundScanInd = 7;

// Ouster OS1-64
// extern const int N_SCAN = 64;
// extern const int Horizon_SCAN = 1024;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 33.2/float(N_SCAN-1);
// extern const float ang_bottom = 16.6+0.1;
// extern const int groundScanInd = 15;

extern const bool loopClosureEnableFlag = true;
extern const double mappingProcessInterval = 0.3;

extern const float scanPeriod = 0.1;
extern const int systemDelay = 0;
extern const int imuQueLength = 200;

extern const float sensorMinimumRange = 1.0;
extern const float sensorMountAngle = 0.0;
extern const float segmentTheta = 60.0/180.0*M_PI; // decrese this value may improve accuracy
extern const int segmentValidPointNum = 5;
extern const int segmentValidLineNum = 3;
extern const float segmentAlphaX = ang_res_x / 180.0 * M_PI;
extern const float segmentAlphaY = ang_res_y / 180.0 * M_PI;


extern const int edgeFeatureNum = 2;
extern const int surfFeatureNum = 4;
extern const int sectionsTotal = 6;
extern const float edgeThreshold = 0.1;
extern const float surfThreshold = 0.1;
extern const float nearestFeatureSearchSqDist = 25;


// Mapping Params
extern const float surroundingKeyframeSearchRadius = 50.0; // key frame that is within n meters from current pose will be considerd for scan-to-map optimization (when loop closure disabled)
extern const int   surroundingKeyframeSearchNum = 50; // submap size (when loop closure enabled)
// history key frames (history submap for loop closure)
extern const float historyKeyframeSearchRadius = 7.0; // key frame that is within n meters from current pose will be considerd for loop closure
extern const int   historyKeyframeSearchNum = 25; // 2n+1 number of hostory key frames will be fused into a submap for loop closure
extern const float historyKeyframeFitnessScore = 0.3; // the smaller the better alignment

extern const float globalMapVisualizationSearchRadius = 500.0; // key frames with in n meters will be visualized


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

typedef PointXYZIRPYT  PointTypePose;



#ifdef SEMANTIC_KITTI
// 34 classes, the map for class ID to RGB
// 0 is unclassified
// raw class to rgb
std::unordered_map<int, std::vector<int> > colors_map = {
	{0, {0, 0, 0}},
    {1, {0, 0, 255}},
    {10, {245, 150, 100}},
    {11, {245, 230, 100}},
    {13, {250, 80, 100}},
    {15, {150, 60, 30}},
    {16, {255, 0, 0}},
    {18, {180, 30, 80}},
    {20, {255, 0, 0}},
    {30, {30, 30, 255}},
    {31, {200, 40, 255}},
    {32, {90, 30, 150}},
    {40, {255, 0, 255}},
    {44, {255, 150, 255}},
    {48, {75, 0, 75}},
    {49, {75, 0, 175}},
    {50, {0, 200, 255}},
    {51, {50, 120, 255}},
    {52, {0, 150, 255}},
    {60, {170, 255, 150}},
    {70, {0, 175, 0}},
    {71, {0, 60, 135}},
    {72, {80, 240, 150}},
    {80, {150, 240, 255}},
    {81, {0, 0, 255}},
    {99, {255, 255, 50}},
    {252, {245, 150, 100}},
    {256, {255, 0, 0}},
    {253, {200, 40, 255}},
    {254, {30, 30, 255}},
    {255, {90, 30, 150}},
    {257, {250, 80, 100}},
    {258, {180, 30, 80}},
    {259, {255, 0, 0}}
};

// trans classes to 
std::unordered_map<int, std::vector<int> > colors_map_tran = {
	// {0, {0, 0, 0}},
    {0, {40, 40, 40}},
    {1, {245, 150, 100}},
    {2, {245, 230, 100}},
    {3, {150, 60, 30}},
    {4, {180, 30, 80}},
    {5, {250, 80, 100}},
    {6, {30, 30, 255}},
    {7, {200, 40, 255}},
    {8, {90, 30, 150}},
    {9, {255, 0, 255}},
    {10, {255, 150, 255}},
    {11, {75, 0, 75}},
    {12, {75, 0, 175}},
    {13, {0, 200, 255}},
    {14, {50, 120, 255}},
    {15, {0, 175, 0}},
    {16, {0, 60, 135}},
    {17, {80, 240, 150}},
    {18, {150, 240, 255}},
    {19, {0, 0, 255}},
    {20, {245, 150, 100}},
    {21, {200, 40, 255}},
    {22, {30, 30, 255}},
    {23, {90, 30, 150}},
    {24, {255, 0, 0}},
    {25, {180, 30, 80}},
};

// 26 valid classes, the map for original id to training id
// 0 is unclassified (total 26 classes with 0)
std::unordered_map<int, int> classes_map = {
	{0 , 0},
	{1 , 0},
	{10, 1},
	{11, 2},
	{13, 5},
	{15, 3},
	{16, 5},
	{18, 4},
	{20, 5},
	{30, 6},
	{31, 7},
	{32, 8},
	{40, 9},
	{44, 10},
	{48, 11},
	{49, 12},
	{50, 13},
	{51, 14},
	{52, 0},
	{60, 9},
	{70, 15},
	{71, 16},
	{72, 17},
	{80, 18},
	{81, 19},
	{99, 0},
	{252, 20},
	{253, 21},
	{254, 22},
	{255, 23},
	{256, 24},
	{257, 24},
	{258, 25},
	{259, 24}
};

/*
0:  0,1,52,99
1:  10
2:  11
3:  15
4:  18
5:  13,16,20
6:  30
7:  31
8:  32
9:  40,60
10: 44
11: 48
12: 49
13: 50
14: 51
15: 70
16: 71
17: 72
18: 80
19: 81
20: 252
21: 253
22: 254
23: 255
24: 256,257,259
25: 258
*/

/*
  0 : 0     # "unlabeled"
  1 : 0     # "outlier" mapped to "unlabeled" --------------------------mapped
  10: 1     # "car"
  11: 2     # "bicycle"
  13: 5     # "bus" mapped to "other-vehicle" --------------------------mapped
  15: 3     # "motorcycle"
  16: 5     # "on-rails" mapped to "other-vehicle" ---------------------mapped
  18: 4     # "truck"
  20: 5     # "other-vehicle"
  30: 6     # "person"
  31: 7     # "bicyclist"
  32: 8     # "motorcyclist"
  40: 9     # "road"
  44: 10    # "parking"
  48: 11    # "sidewalk"
  49: 12    # "other-ground"
  50: 13    # "building"
  51: 14    # "fence"
  52: 0     # "other-structure" mapped to "unlabeled" ------------------mapped
  60: 9     # "lane-marking" to "road" ---------------------------------mapped
  70: 15    # "vegetation"
  71: 16    # "trunk"
  72: 17    # "terrain"
  80: 18    # "pole"
  81: 19    # "traffic-sign"
  99: 0     # "other-object" to "unlabeled" ----------------------------mapped
  252: 20    # "moving-car"
  253: 21    # "moving-bicyclist"
  254: 22    # "moving-person"
  255: 23    # "moving-motorcyclist"
  256: 24    # "moving-on-rails" mapped to "moving-other-vehicle" ------mapped
  257: 24    # "moving-bus" mapped to "moving-other-vehicle" -----------mapped
  258: 25    # "moving-truck"
  259: 24    # "moving-other-vehicle"
*/

#endif // SEMANTIC_KITTI


#endif
