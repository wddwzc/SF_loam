#include "utility.h"

#include <Eigen/Core>
#include <Eigen/Geometry> 

#include <octomap/octomap.h>

class LocalMap {

private:

    ros::NodeHandle nh;

    ros::Publisher pubLocalMap;
    ros::Subscriber subLaserCloud;
    ros::Subscriber subLaserOdom;

    pcl::PointCloud<pcl::PointXYZL>::Ptr localMapCloud;
    pcl::PointCloud<pcl::PointXYZL>::Ptr laserCloudLast;
    geometry_msgs::Point geoPose;
    geometry_msgs::Quaternion geoQuat;
    octomap::OcTree *tree;

    double timeLaserCloudLast;
    double timeLaserOdomLast;
    bool newLaserCloudLast;
    bool newLaserOdomLast;

    std_msgs::Header currentHeader;

public:

    LocalMap(){

        pubLocalMap = nh.advertise<sensor_msgs::PointCloud2> ("/octomap/localmap", 2);
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/kitti/velo/pointxyzl", 5, &LocalMap::laserCloudHandler, this);
        subLaserOdom = nh.subscribe<nav_msgs::Odometry>("/kitti/velodyne_poses", 5, &LocalMap::laserOdometryHandler, this);

        localMapCloud.reset(new pcl::PointCloud<pcl::PointXYZL>());
        laserCloudLast.reset(new pcl::PointCloud<pcl::PointXYZL>());

        tree = new octomap::OcTree(0.1);
    }

    void laserCloudHandler(const sensor_msgs::PointCloud2::ConstPtr& laserCloud)
    {
        currentHeader = laserCloud->header;

        timeLaserCloudLast = currentHeader.stamp.toSec();
        laserCloudLast->clear();
        pcl::fromROSMsg(*laserCloud, *laserCloudLast);
        newLaserCloudLast = true;
    }

    void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry)
    {
        currentHeader = laserOdometry->header;

        timeLaserOdomLast = currentHeader.stamp.toSec();
        geoPose = laserOdometry->pose.pose.position;
        geoQuat = laserOdometry->pose.pose.orientation;
        newLaserOdomLast = true;
    }






    // save ground points into groundCloud
    // groundMat: -1 no valid info; 1 ground; 0 init_value
    void groundRemoval(){
        size_t lowerInd, upperInd;
        float diffX, diffY, diffZ, angle;
        // groundMat
        // -1, no valid info to check if ground of not
        //  0, initial value, after validation, means not ground
        //  1, ground
        for (size_t j = 0; j < Horizon_SCAN; ++j){
            for (size_t i = 0; i < groundScanInd; ++i){

                lowerInd = j + ( i )*Horizon_SCAN;
                upperInd = j + (i+1)*Horizon_SCAN;

                // have no valid info, intensity = -1
                if (fullCloud->points[lowerInd].intensity == -1 ||
                    fullCloud->points[upperInd].intensity == -1){
                    // no info to check, invalid points
                    groundMat.at<int8_t>(i,j) = -1;
                    continue;
                }

                diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
                diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
                diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

                angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;

                if (abs(angle - sensorMountAngle) <= 10){
                //if (abs(angle - sensorMountAngle) <= 5){
                // if (abs(angle - sensorMountAngle) <= 8){
                    groundMat.at<int8_t>(i,j) = 1;
                    groundMat.at<int8_t>(i+1,j) = 1;
                }
            }
        }
        // extract ground cloud (groundMat == 1)
        // mark entry that doesn't need to label (ground and invalid point) for segmentation
        // note that ground remove is from 0~N_SCAN-1, need rangeMat for mark label matrix for the 16th scan
        for (size_t i = 0; i < N_SCAN; ++i){
            for (size_t j = 0; j < Horizon_SCAN; ++j){
                if (groundMat.at<int8_t>(i,j) == 1 || rangeMat.at<float>(i,j) == FLT_MAX){
                    labelMat.at<int>(i,j) = -1;
                }
            }
        }

        if (pubGroundCloud.getNumSubscribers() != 0){
            for (size_t i = 0; i <= groundScanInd; ++i){
                for (size_t j = 0; j < Horizon_SCAN; ++j){
                    if (groundMat.at<int8_t>(i,j) == 1)
                        groundCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                }
            }
        }

    }


    void generateLocalMap() {
        if (newLaserCloudLast && newLaserOdomLast && std::abs(timeLaserCloudLast - timeLaserOdomLast) < 0.005)
        {
            Eigen::Quaterniond q(geoQuat.w, geoQuat.x, geoQuat.y, geoQuat.z);
            Eigen::Matrix4d T_matrix = Eigen::Matrix4d::Identity();
            T_matrix.block<3, 3>(0, 0) = q.matrix();
            T_matrix(0, 3) = geoPose.x;
            T_matrix(1, 3) = geoPose.y;
            T_matrix(2, 3) = geoPose.z;

            // cout << T_matrix << endl;

            pcl::PointCloud<pcl::PointXYZL>::Ptr tempLocalMap1(new pcl::PointCloud<pcl::PointXYZL>());
            pcl::PointCloud<pcl::PointXYZL>::Ptr filterLocalMap(new pcl::PointCloud<pcl::PointXYZL>());
            pcl::PointCloud<pcl::PointXYZL>::Ptr tempLocalMap2(new pcl::PointCloud<pcl::PointXYZL>());
            sensor_msgs::PointCloud2 tempLaserCloud;

            pcl::transformPointCloud(*laserCloudLast, *tempLocalMap1, T_matrix);

            ros::Time begin = ros::Time::now();

            // pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZL>);
            // pcl::VoxelGrid<pcl::PointXYZL> local_filter;
            // local_filter.setInputCloud(tempLocalMap1);
            // local_filter.setLeafSize(0.2f, 0.2f, 0.2f);
            // local_filter.filter(*filterLocalMap);

            // *localMapCloud += *filterLocalMap;

            // local_filter.setInputCloud(localMapCloud);
            // local_filter.setLeafSize(0.2f, 0.2f, 0.2f);
            // local_filter.filter(*tempLocalMap2);

            // 创建八叉树对象，参数为分辨率，这里设成了0.05
            // for (auto &p : tempLocalMap1->points) {
            //     // 将点云里的点插入到octomap中
            //     tree->updateNode(octomap::point3d(p.x, p.y, p.z), true);
            // }

            // tree->insertPointCloudRays();

            // tree->insertPointCloud(tempLocalMap1, octomap::point3d(0.0, 0.0, 0.0));



            // 更新octomap
            tree->updateInnerOccupancy();
            for(octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs();it != end; ++it){
                pcl::PointXYZL cube_center;
                cube_center.x = it.getX();
                cube_center.y = it.getY();
                cube_center.z = it.getZ();
                // cube_center.label = it.getDepth();
            
                if(tree->isNodeOccupied(*it)){
                    tempLocalMap2->points.push_back(cube_center);
                }
            }

            
            ros::Time end = ros::Time::now();
            double du = (end - begin).toSec();
            cout << "Time Cost: " << du << "s" << endl;

            pcl::toROSMsg(*tempLocalMap2, tempLaserCloud);
            tempLaserCloud.header.stamp = currentHeader.stamp;
            tempLaserCloud.header.frame_id = "/local_map";
            pubLocalMap.publish(tempLaserCloud);

            newLaserCloudLast = false;
            newLaserOdomLast = false;
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_map");
    
    LocalMap localmap;

    ROS_INFO("\033[1;32m---->\033[0m Local Map Started.");

    ros::Rate rate(50);
    while (ros::ok()) {
        ros::spinOnce();

        localmap.generateLocalMap();

        rate.sleep();
    }
    return 0;
}
