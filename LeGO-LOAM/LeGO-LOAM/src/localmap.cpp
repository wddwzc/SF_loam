#include "utility.h"

#include <Eigen/Core>
#include <Eigen/Geometry> 

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

            pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZL>);
            pcl::VoxelGrid<pcl::PointXYZL> local_filter;
            local_filter.setInputCloud(tempLocalMap1);
            local_filter.setLeafSize(0.2f, 0.2f, 0.2f);
            local_filter.filter(*filterLocalMap);

            *localMapCloud += *filterLocalMap;

            local_filter.setInputCloud(localMapCloud);
            local_filter.setLeafSize(0.2f, 0.2f, 0.2f);
            local_filter.filter(*tempLocalMap2);

            pcl::toROSMsg(*tempLocalMap2, tempLaserCloud);
            tempLaserCloud.header.stamp = currentHeader.stamp;
            tempLaserCloud.header.frame_id = "/local_map";
            pubLocalMap.publish(tempLaserCloud);

            octomap::Octree tree(0.02);
            for (auto &p : tempLaserCloud->points) {
                tree.updateNode(octomap::point3d(p.x, p.y, p.z), true);
            }
            tree.updateInnerOccupancy();

            // pcl::PointCloud<pcl::PointXYZRGBA> cloud;
            // pcl::io::loadPCDFile<pcl::PointXYZRGBA> ( input_file, cloud );

            // cout<<"point cloud loaded, piont size = "<<cloud.points.size()<<endl;

            // //声明octomap变量
            // cout<<"copy data into octomap..."<<endl;
            // // 创建八叉树对象，参数为分辨率，这里设成了0.05
            // octomap::OcTree tree( 0.05 );

            // for (auto p:cloud.points)
            // {
            //     // 将点云里的点插入到octomap中
            //     tree.updateNode( octomap::point3d(p.x, p.y, p.z), true );
            // }

            // // 更新octomap
            // tree.updateInnerOccupancy();
            // // 存储octomap
            // tree.writeBinary( output_file );
            // cout<<"done."<<endl;
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
