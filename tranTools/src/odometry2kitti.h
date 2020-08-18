#include <string>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <unordered_map>
#include <vector>

#define PI 3.14159265
#define QUE_MAX 10

// 34 classes, the map for class ID to RGB
// 0 is unclassified
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

// 25 valid classes, the map for original id to training id
// 0 is unclassified
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
  0 : [0, 0, 0]
  1 : [0, 0, 255]
  10: [245, 150, 100]
  11: [245, 230, 100]
  13: [250, 80, 100]
  15: [150, 60, 30]
  16: [255, 0, 0]
  18: [180, 30, 80]
  20: [255, 0, 0]
  30: [30, 30, 255]
  31: [200, 40, 255]
  32: [90, 30, 150]
  40: [255, 0, 255]
  44: [255, 150, 255]
  48: [75, 0, 75]
  49: [75, 0, 175]
  50: [0, 200, 255]
  51: [50, 120, 255]
  52: [0, 150, 255]
  60: [170, 255, 150]
  70: [0, 175, 0]
  71: [0, 60, 135]
  72: [80, 240, 150]
  80: [150, 240, 255]
  81: [0, 0, 255]
  99: [255, 255, 50]
  252: [245, 150, 100]
  256: [255, 0, 0]
  253: [200, 40, 255]
  254: [30, 30, 255]
  255: [90, 30, 150]
  257: [250, 80, 100]
  258: [180, 30, 80]
  259: [255, 0, 0]
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