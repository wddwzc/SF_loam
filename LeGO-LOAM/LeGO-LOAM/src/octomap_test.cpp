#include "utility.h"

#include <Eigen/Core>
#include <Eigen/Geometry> 

#include <octomap/octomap.h>

ros::Publisher pubLocalMap;
pcl::PointCloud<pcl::PointXYZL>::Ptr testMapCloud;
octomap::OcTree *tree;

void addPoint(float x, float y, float z) {
    tree->updateNode(octomap::point3d(x, y, z), true);
}

void addRay(float x, float y, float z) {
    tree->insertRay(octomap::point3d(0.0, 0.0, 0.0), octomap::point3d(x, y, z));
}

void test() {
    static int ray_pose = 20;
    static int point_pose = 1;

    addPoint(point_pose++ % 100, 0, 0);
    addRay(0, ray_pose-- % 100, 0);
    tree->updateInnerOccupancy();

    sensor_msgs::PointCloud2 tempLaserCloud;
    testMapCloud.reset(new pcl::PointCloud<pcl::PointXYZL>());
    for(octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs();it != end; ++it){
        pcl::PointXYZL cube_center;
        cube_center.x = it.getX();
        cube_center.y = it.getY();
        cube_center.z = it.getZ();
        // cube_center.label = it.getDepth();
    
        if(tree->isNodeOccupied(*it)){
            testMapCloud->points.push_back(cube_center);
        }
    }
    pcl::toROSMsg(*testMapCloud, tempLaserCloud);
    tempLaserCloud.header.stamp = ros::Time::now();
    tempLaserCloud.header.frame_id = "/octomap";
    pubLocalMap.publish(tempLaserCloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Octomap_Test");

    ROS_INFO("\033[1;32m---->\033[0m Octomap Test Started.");

    ros::NodeHandle nh;
    pubLocalMap = nh.advertise<sensor_msgs::PointCloud2> ("/octomap/test_mapcloud", 2);
    tree = new octomap::OcTree(0.1);


    ros::Rate rate(1);
    while (ros::ok()) {
        test();
        rate.sleep();
    }
    return 0;
}

