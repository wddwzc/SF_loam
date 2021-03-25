#include "utility.h"
#include "parameter.h"
#include "extra_tools/debug_utility.h"
#include "cluster_method/segment.h"
#include "cluster_method/cluster.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry> 

#include <octomap/octomap.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

class LocalMap {

private:

    ros::NodeHandle nh;

    ros::Publisher pubLocalMap;
    ros::Publisher pubGroundCloud;
    ros::Publisher pubNoGroundCloud;
    ros::Publisher pubBoundingBox;
    ros::Subscriber subPointCloudAll;
    ros::Subscriber subGroundCloud;
    ros::Subscriber subSegmentedCloud;
    ros::Subscriber subLaserOdom;

    pcl::PointCloud<PointType>::Ptr pointCloudAll;
    pcl::PointCloud<PointType>::Ptr groundCloudLast;
    pcl::PointCloud<PointType>::Ptr segmentedCloudLast;
    geometry_msgs::Point geoPose;
    geometry_msgs::Quaternion geoQuat;
    std_msgs::Header currentHeader;
    double timePointCloudAll;
    double timeGroundCloudLast;
    double timeSegmentedCloudLast;
    double timeLaserOdomLast;
    bool newPointCloudAll;
    bool newGroundCloudLast;
    bool newSegmentedCloudLast;
    bool newLaserOdomLast;

    Eigen::Matrix4d global_pose;
    octomap::OcTree *tree;
    vector<Segment> mapSegments;                          // 局部地图分割
    pcl::PointCloud<PointType>::Ptr mapSegmentsCenters;   // boudning box 中心
    pcl::PointCloud<PointType>::Ptr localMapCloud;        // 局部地图点云

    ParamServer param;

public:

    LocalMap(){

        pubLocalMap = nh.advertise<sensor_msgs::PointCloud2>("/octomap/localmap", 2);
        pubGroundCloud = nh.advertise<sensor_msgs::PointCloud2>("/octomap/ground_cloud", 2);
        pubNoGroundCloud = nh.advertise<sensor_msgs::PointCloud2>("/octomap/no_ground_cloud", 2);
        pubBoundingBox = nh.advertise<visualization_msgs::MarkerArray>("/octomap/bounding_box", 2);
        subPointCloudAll = nh.subscribe<sensor_msgs::PointCloud2>("/kitti/velo/pointall", 2, &LocalMap::pointCloudAllHandler, this);
        subGroundCloud = nh.subscribe<sensor_msgs::PointCloud2>("/ground_cloud", 2, &LocalMap::groundCloudHandler, this);
        subSegmentedCloud = nh.subscribe<sensor_msgs::PointCloud2>("/segmented_cloud_pure", 2, &LocalMap::segmentedCloudHandler, this);
        subLaserOdom = nh.subscribe<nav_msgs::Odometry>(param.odometryTopic, 5, &LocalMap::laserOdometryHandler, this);

        pointCloudAll.reset(new pcl::PointCloud<PointType>());
        groundCloudLast.reset(new pcl::PointCloud<PointType>());
        segmentedCloudLast.reset(new pcl::PointCloud<PointType>());

        mapSegmentsCenters.reset(new pcl::PointCloud<PointType>());
        localMapCloud.reset(new pcl::PointCloud<PointType>());

        newPointCloudAll = false;
        newGroundCloudLast = false;
        newSegmentedCloudLast = false;
        newLaserOdomLast = false;

        tree = new octomap::OcTree(param.octomapLeafSize);  // default: 0.1
        global_pose = Eigen::Matrix4d::Identity();
    }

    void resetMemory(){
        pointCloudAll->clear();
        groundCloudLast->clear();
        segmentedCloudLast->clear();
    }

    void pointCloudAllHandler(const sensor_msgs::PointCloud2::ConstPtr& laserCloud) {
        currentHeader = laserCloud->header;

        timePointCloudAll = currentHeader.stamp.toSec();
        pointCloudAll->clear();
        pcl::fromROSMsg(*laserCloud, *pointCloudAll);
        newPointCloudAll = true;
    }

    void groundCloudHandler(const sensor_msgs::PointCloud2::ConstPtr& laserCloud)
    {
        currentHeader = laserCloud->header;

        timeGroundCloudLast = currentHeader.stamp.toSec();
        groundCloudLast->clear();
        pcl::fromROSMsg(*laserCloud, *groundCloudLast);
        newGroundCloudLast = true;
    }

    void segmentedCloudHandler(const sensor_msgs::PointCloud2::ConstPtr& laserCloud)
    {
        currentHeader = laserCloud->header;

        timeSegmentedCloudLast = currentHeader.stamp.toSec();
        segmentedCloudLast->clear();
        pcl::fromROSMsg(*laserCloud, *segmentedCloudLast);
        newSegmentedCloudLast = true;
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
        // if (newGroundCloudLast && newSegmentedCloudLast && newLaserOdomLast 
        //     && std::abs(timeGroundCloudLast - timeSegmentedCloudLast) < 0.005
        //     && std::abs(timeGroundCloudLast - timeLaserOdomLast) < 0.005
        //     && std::abs(timePointCloudAll - timeGroundCloudLast) < 0.005)

        pcl::PointCloud<PointType>::Ptr groundCloud(new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr noGroundCloud(new pcl::PointCloud<PointType>);

        if ( (param.usePclSegmentation && newPointCloudAll && newLaserOdomLast && std::abs(timePointCloudAll - timeLaserOdomLast) < 0.005) || 
            (!param.usePclSegmentation && newGroundCloudLast && newSegmentedCloudLast && newLaserOdomLast &&
            (timeGroundCloudLast - timeSegmentedCloudLast) < 0.005 && (timeGroundCloudLast - timeLaserOdomLast) < 0.005) )
        {
            if (param.debugOctomapGenerator) cout << "# generate local map ######";
            TimeRecorder generatorTime;
            generatorTime.recordStart();

            // 将 local map 转到以当前位置为中心的坐标系下
            transformLocalMapCloud();

            
            if (param.usePclSegmentation) {  // 用pcl的提取地面
                if (param.debugOctomapGenerator)  cout << "### pcl extract plane ###" << endl;
                // 对单帧点云滤波
                if (param.debugOctomapGenerator)  cout << "voxel filter (all): " << pointCloudAll->points.size();
                pcl::VoxelGrid<PointType> local_filter;
                local_filter.setInputCloud(pointCloudAll);
                local_filter.setLeafSize(param.voxelLeafSize, param.voxelLeafSize, param.voxelLeafSize);  // default: 0.1
                local_filter.filter(*pointCloudAll);
                if (param.debugOctomapGenerator)  cout << " ---> " << pointCloudAll->points.size() << endl;
                // 地面分离
                groundRemoval(pointCloudAll, groundCloud, noGroundCloud);
                if (param.debugOctomapGenerator) {
                    cout << "Ground Cloud: " << groundCloud->points.size() << endl;
                    cout << "No Ground Cloud: " << noGroundCloud->points.size() << endl;
                }
            }
            else {  // 用imageProjection输出的地面和分割
                if (param.debugOctomapGenerator)  cout << "### fast extract plane ###" << endl;
                // 对单帧点云滤波
                pcl::PointCloud<PointType>::Ptr lasercloud_filtered(new pcl::PointCloud<PointType>);
                pcl::VoxelGrid<PointType> local_filter;
                local_filter.setLeafSize(param.voxelLeafSize, param.voxelLeafSize, param.voxelLeafSize);  // default: 0.1

                if (param.debugOctomapGenerator)  cout << "voxel filter (ground): " << groundCloudLast->points.size(); 
                local_filter.setInputCloud(groundCloudLast);
                local_filter.filter(*groundCloud);
                if (param.debugOctomapGenerator)  cout << " ---> " << groundCloud->points.size() << endl;

                if (param.debugOctomapGenerator)  cout << "voxel filter (segment): " << segmentedCloudLast->points.size(); 
                local_filter.setInputCloud(segmentedCloudLast);
                local_filter.filter(*noGroundCloud);
                if (param.debugOctomapGenerator)  cout << " ---> " << noGroundCloud->points.size() << endl;
            }            


            vector<Segment> new_segments;
            Cluster cluster;
            cluster.geometricSegmentation(param, noGroundCloud, new_segments);
            // removeDynamicSegment(new_segments);


            *localMapCloud += *groundCloud;
            *localMapCloud += *noGroundCloud;
            pcl::VoxelGrid<PointType> local_filter;
            local_filter.setLeafSize(param.voxelLeafSize, param.voxelLeafSize, param.voxelLeafSize);  // default: 0.1
            local_filter.setInputCloud(localMapCloud);
            local_filter.filter(*localMapCloud);


            if (param.debugOctomapGenerator && param.debugOctomapTimeCost)
                generatorTime.printDuration("Generate local map cost: ");

            
            visualizeCloudRGB(groundCloud, pubGroundCloud, currentHeader.stamp, "/local_map");
            visualizeCloudRGB(noGroundCloud, pubNoGroundCloud, currentHeader.stamp, "/local_map");
            visualizeCloudRGB(localMapCloud, pubLocalMap, currentHeader.stamp, "/local_map");
            visualizeBox(new_segments, pubBoundingBox, currentHeader.stamp, "/local_map");

            newGroundCloudLast = false;
            newSegmentedCloudLast = false;
            newLaserOdomLast = false;
        }

        resetMemory();
    }


    // 根据当前位姿变换 mapSegments  mapSegmentsCenters localMapCloud，筛出半径外的点
    void transformLocalMapCloud() {
        // 将 local map 转到以当前位置为中心的坐标系下
        Eigen::Quaterniond q(geoQuat.w, geoQuat.x, geoQuat.y, geoQuat.z);
        Eigen::Matrix4d T_matrix = Eigen::Matrix4d::Identity();
        T_matrix.block<3, 3>(0, 0) = q.matrix();
        T_matrix(0, 3) = geoPose.x;
        T_matrix(1, 3) = geoPose.y;
        T_matrix(2, 3) = geoPose.z;
        Eigen::Matrix4d delta_matrix = T_matrix.inverse() * global_pose;
        
        size_t sz = mapSegments.size();
        vector<Segment> transformedSegments;
        pcl::PointCloud<PointType>::Ptr transformedCenters(new pcl::PointCloud<PointType>());
        transformedSegments.reserve(sz);
        for (size_t i = 0; i < sz; ++i) {
            Segment &seg = mapSegments[i];
            pcl::transformPointCloud(seg.segmentCloud, seg.segmentCloud, delta_matrix);
            seg.updateBox();
            float &center_x = seg.box.bboxTransform[0];
            float &center_y = seg.box.bboxTransform[1];
            float &center_z = seg.box.bboxTransform[2];
            if (sqrt(center_x * center_x + center_y * center_y + center_z * center_z) < param.localMapRadius) {
                transformedSegments.push_back(seg);
                PointType p;
                p.x = center_x;
                p.y = center_y;
                p.z = center_z;
                p.label = seg.semantic;
                p.intensity = (float)transformedCenters->points.size();
                transformedCenters->points.push_back(p);
            }
        }
        mapSegments.swap(transformedSegments);
        mapSegmentsCenters = transformedCenters;

        pcl::PointCloud<PointType>::Ptr transformedLocalMapCloud(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*localMapCloud, *transformedLocalMapCloud, delta_matrix);
        localMapCloud->clear();
        for (auto &p : transformedLocalMapCloud->points) {
            if (sqrt(p.x * p.x + p.y * p.y + p.z * p.z) < param.localMapRadius) {
                localMapCloud->points.push_back(p);
            }
        }

        // 更新当前位置
        global_pose = T_matrix;

    }


    // save ground points into groundCloud
    void groundRemoval(pcl::PointCloud<PointType>::Ptr laserCloudIn, pcl::PointCloud<PointType>::Ptr groundCloud, pcl::PointCloud<PointType>::Ptr noGroundCloud) {
        // 移除地面
        pcl::SACSegmentation<PointType> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointCloud<PointType>::Ptr cloud_plane(new pcl::PointCloud<PointType>);
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(param.extractPlaneThreshold);
        seg.setInputCloud(laserCloudIn);
        seg.segment(*inliers, *coefficients);

        pcl::PointCloud<PointType>::Ptr cloud_f(new pcl::PointCloud<PointType>);
        if (inliers->indices.size() == 0) {
            std::cout << "Could not estimate a planar anymore." << std::endl;
        } else {
            pcl::ExtractIndices<PointType> extract;
            extract.setInputCloud(laserCloudIn);
            extract.setIndices(inliers);
            // get 
            extract.setNegative(false);
            extract.filter(*groundCloud);
            // filter planar
            extract.setNegative(true);
            extract.filter(*noGroundCloud);
        }
    }

    void removeDynamicSegment(vector<Segment> &new_segments) {
        if (mapSegments.empty()) {
            return;
        }

        pcl::KdTreeFLANN<PointType> kdtreeSurroundSegments;
        kdtreeSurroundSegments.setInputCloud(mapSegmentsCenters);
        for (auto &new_seg : new_segments) {
            vector<int> centroidSearchInd;
            vector<float> centroidSearchSqDis;
            kdtreeSurroundSegments.radiusSearch(new_seg.centroid, (double)param.dynamicSegmentSearchRadius, centroidSearchInd, centroidSearchSqDis, 0);

            // for ()
            

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




            // 点云插入octomap
            // TimeRecorder octomapTime;

            // octomapTime.recordStart();
            // for (auto &p : noGroundCloud->points) {
            //     tree->updateNode(octomap::point3d(p.x, p.y, p.z), true);
            // }
            // // for (auto &p : noGroundCloud->points) {
            // //     tree->insertRay(octomap::point3d(p.x, p.y, p.z), octomap::point3d(geoPose.x, geoPose.y, geoPose.z));
            // // }
            // // void OccupancyOcTreeBase<NODE>::insertPointCloudRays(const Pointcloud& pc, const point3d& origin, double maxrange, bool lazy_eval)
            // // 创建八叉树对象，参数为分辨率，这里设成了0.05
            // for (auto &p : groundCloud->points) {
            //     // 将点云里的点插入到octomap中
            //     // tree->updateNode(octomap::point3d(p.x, p.y, p.z), true);

            //     // 插入射线
            //     tree->insertRay(octomap::point3d(p.x, p.y, p.z), octomap::point3d(geoPose.x, geoPose.y, geoPose.z));
            //     // tree->insertPointCloudRays();
            // }

            // // // 更新octomap,并输出
            // // tree->updateInnerOccupancy();
            // if (param.debugOctomapTimeCost)  octomapTime.printDuration("Octomap time cost");

            // pcl::PointCloud<PointType>::Ptr filterLocalMap(new pcl::PointCloud<PointType>());
            // for(octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs();it != end; ++it){
            //     PointType cube_center;
            //     cube_center.x = it.getX();
            //     cube_center.y = it.getY();
            //     cube_center.z = it.getZ();
            //     // cube_center.label = it.getDepth();
            
            //     if(tree->isNodeOccupied(*it)){
            //         filterLocalMap->points.push_back(cube_center);
            //     }
            // }
