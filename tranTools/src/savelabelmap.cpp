#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

std::string output_dir = "/home/wzc";
std::string output_map = "/featureMap05.txt";
std::string output_mapRGBA = "/featureMapRGBA05.txt";
std::string input_mapXYZL = "/labelMap05.txt";

std::ofstream opf_map;
std::ifstream ipf_mapXYZL;
std::ofstream opf_mapRGBA;

pcl::PointCloud<pcl::PointXYZI>::Ptr SurroundPoints(new pcl::PointCloud<pcl::PointXYZI>());

const int col_map[][3] = { 153, 0, 0, 		// building
	                       160, 160, 160,  // road
	                       0, 102, 0,      // vegetation
	                       255, 200, 50,   // vehicle
	                       193, 120, 87,   // pole
	                       255, 153, 255 };  // pedestrian

int mapnum = 0;

void laserCloudSurroundHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudSurround)
{
    SurroundPoints->clear();
    pcl::fromROSMsg(*laserCloudSurround, *SurroundPoints);

    int label;
    for (auto point : SurroundPoints->points) {
        label = int(point.intensity) / 100;
        std::cout << label << " ";
        // opf_map << point.x << " " << point.y << " " << point.z << " " << label << std::endl;
    }
    mapnum += SurroundPoints->points.size();
    std::cout << "Map Size: " << mapnum << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "savelabelmap");
    ros::NodeHandle n;

    opf_map.open((output_dir + output_map).c_str());
    ipf_mapXYZL.open((output_dir + input_mapXYZL).c_str());
    opf_mapRGBA.open((output_dir + output_mapRGBA).c_str());

    // ros::Subscriber subLaserCloudSurround = n.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surround", 2, laserCloudSurroundHandler);
    //
    // ros::spin();

    int pointNum = 0;
    pcl::PointXYZL pointIn;
    pcl::PointXYZRGBA pointOut;
    bool status = ros::ok();
    while (!ipf_mapXYZL.eof() && status) {
        ipf_mapXYZL >> pointIn.x >> pointIn.y >> pointIn.z >> pointIn.label;
        pointOut.x = pointIn.x;
        pointOut.y = pointIn.y;
        pointOut.z = pointIn.z;
        pointOut.r = col_map[pointIn.label][0];
        pointOut.g = col_map[pointIn.label][1];
        pointOut.b = col_map[pointIn.label][2];
        pointOut.a = 160;
        // std::cout << pointOut.x << " " << pointOut.y << " " << pointOut.z << " " << pointOut.rgba << std::endl;
        opf_mapRGBA << pointOut.x << " " << pointOut.y << " " << pointOut.z << " " << pointOut.rgba << std::endl;
	// opf_mapRGBA << pointOut.z << " " << pointOut.x << " " << pointOut.y << " " << int(pointIn.label) << std::endl;
        ++pointNum;
        std::cout << pointNum << std::endl;
        // opf_mapRGBA
        status = ros::ok();
    }

    opf_map.close();
    ipf_mapXYZL.close();
    opf_mapRGBA.close();

    return 0;
}
