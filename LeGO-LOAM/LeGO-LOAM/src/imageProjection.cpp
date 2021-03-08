// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following papers:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). October 2018.

#include "utility.h"

class ImageProjection{
private:

    ros::NodeHandle nh;

    ros::Subscriber subLaserCloud;
    
    ros::Publisher pubFullCloud;
    ros::Publisher pubFullInfoCloud;

    ros::Publisher pubGroundCloud;
    ros::Publisher pubSegmentedCloud;
    ros::Publisher pubSegmentedCloudPure;
    ros::Publisher pubSegmentedCloudInfo;
    ros::Publisher pubOutlierCloud;

#ifdef SEMANTIC_KITTI
    // SEMANTIC
    // publisher
    ros::Publisher pubColoredLaserCloud;
    ros::Publisher pubClassifiedCentroidRGB;
    ros::Publisher pubClassifiedCloud;
    ros::Publisher pubCentroid2Seg;
    ros::Publisher pubClassifiedCentroid; // for mapOptimization
    ros::Publisher pubNoGroundCloud;  // for mapOptimization
    // PointXYZRGB for visulizer
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudColor;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr classifiedCentroidRGB;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr classifiedCloud;
    visualization_msgs::Marker marker_Centroid2Seg;
    pcl::PointCloud<pcl::PointXYZL>::Ptr classifiedCentroid; // for mapOptimization
    pcl::PointCloud<pcl::PointXYZL>::Ptr laserCloud_noGround; // for mapOptimization

    vector<int> index_full;  // from valid point to raw cloud ID (fullCloud->laserCloudIn)
#endif

#ifdef SEMANTIC_KITTI
    pcl::PointCloud<pcl::PointXYZL>::Ptr laserCloudIn;
#else
    pcl::PointCloud<PointType>::Ptr laserCloudIn;
#endif
    pcl::PointCloud<PointXYZIR>::Ptr laserCloudInRing;


    pcl::PointCloud<PointType>::Ptr fullCloud; // projected velodyne raw cloud, but saved in the form of 1-D matrix
    pcl::PointCloud<PointType>::Ptr fullInfoCloud; // same as fullCloud, but with intensity - range

    pcl::PointCloud<PointType>::Ptr groundCloud;
    pcl::PointCloud<PointType>::Ptr segmentedCloud;
    pcl::PointCloud<PointType>::Ptr segmentedCloudPure;
    pcl::PointCloud<PointType>::Ptr outlierCloud;

    PointType nanPoint; // fill in fullCloud at each iteration

    cv::Mat rangeMat; // range matrix for range image
    cv::Mat labelMat; // label matrix for segmentaiton marking
    cv::Mat groundMat; // ground matrix for ground cloud marking
    int labelCount;

    float startOrientation;
    float endOrientation;

    cloud_msgs::cloud_info segMsg; // info of segmented cloud
    std_msgs::Header cloudHeader;

    std::vector<std::pair<int8_t, int8_t> > neighborIterator; // neighbor iterator for segmentaiton process

    uint16_t *allPushedIndX; // array for tracking points of a segmented object
    uint16_t *allPushedIndY;

    uint16_t *queueIndX; // array for breadth-first search process of segmentation, for speed
    uint16_t *queueIndY;

    // launch parameter
    bool param_semantic;
    // int N_SCAN;
    // int Horizon_SCAN;
    // float ang_res_x;
    // float ang_res_y;
    // float ang_bottom;
    // int groundScanInd;

public:
    ImageProjection():
        nh("~") {
        
        // nh.getParam("/sf_loam/semantic", param_semantic);
        // nh.getParam("")

        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 1, &ImageProjection::cloudHandler, this);

        pubFullCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_projected", 1);
        pubFullInfoCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_info", 1);

        pubGroundCloud = nh.advertise<sensor_msgs::PointCloud2> ("/ground_cloud", 1);
        pubSegmentedCloud = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud", 1);
        pubSegmentedCloudPure = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud_pure", 1);
        pubSegmentedCloudInfo = nh.advertise<cloud_msgs::cloud_info> ("/segmented_cloud_info", 1);
        pubOutlierCloud = nh.advertise<sensor_msgs::PointCloud2> ("/outlier_cloud", 1);

#ifdef SEMANTIC_KITTI
        // for visulize
        pubColoredLaserCloud = nh.advertise<sensor_msgs::PointCloud2> ("/A_laser_cloud_color", 1);
        pubClassifiedCentroidRGB = nh.advertise<sensor_msgs::PointCloud2> ("/A_classified_centroid_color", 1);
        pubClassifiedCloud = nh.advertise<sensor_msgs::PointCloud2> ("/A_classified_cloud", 1);
        pubCentroid2Seg = nh.advertise<visualization_msgs::Marker>("/A_marker_centroid2seg", 1);
        // for mapOptimization
        pubClassifiedCentroid = nh.advertise<sensor_msgs::PointCloud2> ("/A_classified_centroid", 1);
        pubNoGroundCloud = nh.advertise<sensor_msgs::PointCloud2> ("/A_laser_cloud_noground", 1);
#endif

        nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = -1;

        allocateMemory();
        resetParameters();
    }

    void allocateMemory(){

#ifdef SEMANTIC_KITTI
        laserCloudColor.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        classifiedCentroidRGB.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        classifiedCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

        classifiedCentroid.reset(new pcl::PointCloud<pcl::PointXYZL>());
        laserCloud_noGround.reset(new pcl::PointCloud<pcl::PointXYZL>());

        index_full.resize(N_SCAN*Horizon_SCAN);

        marker_Centroid2Seg.header.frame_id = "base_link";
        marker_Centroid2Seg.header.stamp = ros::Time();
        marker_Centroid2Seg.ns = "my_namespace";
        marker_Centroid2Seg.id = 0;
        marker_Centroid2Seg.type = visualization_msgs::Marker::LINE_LIST;
        marker_Centroid2Seg.action = visualization_msgs::Marker::ADD;
        marker_Centroid2Seg.pose.orientation.w = 1.0;
        marker_Centroid2Seg.scale.x = 0.1;
        marker_Centroid2Seg.color.r = 1.0f;
        marker_Centroid2Seg.color.a = 1.0f;
#endif

#ifdef SEMANTIC_KITTI
        laserCloudIn.reset(new pcl::PointCloud<pcl::PointXYZL>());
#else
        laserCloudIn.reset(new pcl::PointCloud<PointType>());
#endif
        laserCloudInRing.reset(new pcl::PointCloud<PointXYZIR>());


        fullCloud.reset(new pcl::PointCloud<PointType>());
        fullInfoCloud.reset(new pcl::PointCloud<PointType>());

        groundCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloudPure.reset(new pcl::PointCloud<PointType>());
        outlierCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCAN*Horizon_SCAN);
        fullInfoCloud->points.resize(N_SCAN*Horizon_SCAN);

        segMsg.startRingIndex.assign(N_SCAN, 0);
        segMsg.endRingIndex.assign(N_SCAN, 0);

        segMsg.segmentedCloudGroundFlag.assign(N_SCAN*Horizon_SCAN, false);
        segMsg.segmentedCloudColInd.assign(N_SCAN*Horizon_SCAN, 0);
        segMsg.segmentedCloudRange.assign(N_SCAN*Horizon_SCAN, 0);

        std::pair<int8_t, int8_t> neighbor;
        neighbor.first = -1; neighbor.second =  0; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second =  1; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second = -1; neighborIterator.push_back(neighbor);
        neighbor.first =  1; neighbor.second =  0; neighborIterator.push_back(neighbor);

        allPushedIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        allPushedIndY = new uint16_t[N_SCAN*Horizon_SCAN];

        queueIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        queueIndY = new uint16_t[N_SCAN*Horizon_SCAN];
    }

    void resetParameters(){
        laserCloudIn->clear();
        groundCloud->clear();
        segmentedCloud->clear();
        segmentedCloudPure->clear();
        outlierCloud->clear();

        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
        labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));
        labelCount = 1;

        std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
        std::fill(fullInfoCloud->points.begin(), fullInfoCloud->points.end(), nanPoint);


#ifdef SEMANTIC_KITTI
        laserCloudColor->clear();
        classifiedCentroidRGB->clear();
        classifiedCloud->clear();
        classifiedCentroid->clear();
        laserCloud_noGround->clear();
        index_full.assign(N_SCAN*Horizon_SCAN, -1);
        marker_Centroid2Seg.points.clear();
#endif
    }

    ~ImageProjection(){}

    void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        cloudHeader = laserCloudMsg->header;

        // cloudHeader.stamp = ros::Time::now(); // Ouster lidar users may need to uncomment this line
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
        // Remove Nan points
        std::vector<int> indices;
#ifdef DEBUG_CLUSTER
        cout << "bef: " << laserCloudIn->points.size() << endl;
#endif
        pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
#ifdef DEBUG_CLUSTER
        cout << "aft: " << laserCloudIn->points.size() << endl;
#endif
        // have "ring" channel in the cloud
        if (useCloudRing == true){
            pcl::fromROSMsg(*laserCloudMsg, *laserCloudInRing);
            if (laserCloudInRing->is_dense == false) {
                ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
                ros::shutdown();
            }
        }
    }
    
    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        // 1. Convert ros message to pcl point cloud
        copyPointCloud(laserCloudMsg);
        // 2. Start and end angle of a scan
        findStartEndAngle();
        // 3. Range image projection
        projectPointCloud();
        // 4. Mark ground points
        groundRemoval();
        // 5. Point cloud segmentation
        cloudSegmentation();
#ifdef SEMANTIC_KITTI
        // Semantic Segmentation
        geometricSegmentation();
#endif
        // 6. Publish all clouds
        publishCloud();
        // 7. Reset parameters for next iteration
        resetParameters();
    }


#ifdef SEMANTIC_KITTI
    void geometricSegmentation() {

#ifdef TIME_COST
        // Calculate time cost for segmentation
        double begin_time_ = ros::Time::now().toSec();
#endif

        if (laserCloud_noGround->points.size() == 0)  return;

#ifdef DEBUG_CLUSTER
        cout << "point size without ground: " << laserCloud_noGround->points.size() << endl;
#endif

        // PointType is PointXYZL
        pcl::search::KdTree<pcl::PointXYZL>::Ptr treeSegment(new pcl::search::KdTree<pcl::PointXYZL>);
        treeSegment->setInputCloud(laserCloud_noGround);
        std::vector<pcl::PointIndices> cluster_indices;
        float radius_for_growing_ = 0.2;
        unsigned int core_MinPts_ = 10;  // only valid in DBSCAN
        unsigned int min_segment_size_ = 100;
        unsigned int max_segment_size_ = 25000;
#ifdef DEBUG_CLUSTER
        cout << "extracting cluster" << endl;
#endif

        // DBSCAN
        DBSCANKdtreeCluster<pcl::PointXYZL> ec;
        ec.setCorePointMinPts(core_MinPts_);
        ec.setClusterTolerance(radius_for_growing_);
        ec.setMinClusterSize(min_segment_size_);
        ec.setMaxClusterSize(max_segment_size_);
        ec.setSearchMethod(treeSegment);
        ec.setInputCloud(laserCloud_noGround);
        ec.extract(cluster_indices);

        // EuclideanClusterExtraction
        // function pattern

        // pcl::extractEuclideanClusters<pcl::PointXYZL>(*laserCloud_noGround, treeSegment, radius_for_growing_, 
        //                                              cluster_indices, min_segment_size_, max_segment_size_);
#ifdef DEBUG_CLUSTER
        cout << "extracted cluster" << endl;
#endif

        for (auto &segment_to_add : cluster_indices) {
            unsigned int sz = segment_to_add.indices.size();

#ifdef DEBUG_CLUSTER
            cout << "cur_segment_size: " << sz << endl;
#endif

            pcl::PointXYZRGB centroid(0.0, 0.0, 0.0);
            vector<unsigned int> class_counts(26, 0);
            for (auto &index : segment_to_add.indices) {
                pcl::PointXYZL &cur_point = laserCloud_noGround->points[index];
                unsigned class_id = classes_map[cur_point.label];
                ++class_counts[class_id];
                centroid.x += cur_point.x;
                centroid.y += cur_point.y;
                centroid.z += cur_point.z;
            }
            centroid.x /= (float)sz;
            centroid.y /= (float)sz;
            centroid.z /= (float)sz;

            unsigned int segment_class = 0;
            for (int i = 0; i < 25; ++i) {
                if (class_counts[i] / (float)sz >= 0.6)
                    segment_class = i;
            }
            centroid.r = colors_map_tran[segment_class][0];
            centroid.g = colors_map_tran[segment_class][1];
            centroid.b = colors_map_tran[segment_class][2];

            // add marker
            geometry_msgs::Point marker_p;
            marker_p.x = centroid.x;
            marker_p.y = centroid.y;
            marker_p.z = centroid.z;
            marker_Centroid2Seg.points.push_back(marker_p);
            marker_p.z += 10.0;
            marker_Centroid2Seg.points.push_back(marker_p);

            classifiedCentroidRGB->push_back(centroid);
            centroid.z += 10.0;
            classifiedCentroidRGB->push_back(centroid);

            for (auto &index : segment_to_add.indices) {
                pcl::PointXYZL cur_point = laserCloud_noGround->points[index];
                pcl::PointXYZRGB cur_p;
                cur_p.x = cur_point.x;
                cur_p.y = cur_point.y;
                cur_p.z = cur_point.z;
                cur_p.r = colors_map_tran[segment_class][0];
                cur_p.g = colors_map_tran[segment_class][1];
                cur_p.b = colors_map_tran[segment_class][2];
                classifiedCloud->push_back(cur_p);
            }
        }

#ifdef TIME_COST
        // Calculate time cost for segmentation
        double end_time_ = ros::Time::now().toSec();
        std::cout << "Segmentation time: " << (end_time_ - begin_time_) * 1000 << std::endl;
#endif

    }
#endif

    void findStartEndAngle(){
        // start and end orientation of this cloud
        segMsg.startOrientation = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
        segMsg.endOrientation   = -atan2(laserCloudIn->points[laserCloudIn->points.size() - 1].y,
                                                     laserCloudIn->points[laserCloudIn->points.size() - 1].x) + 2 * M_PI;
        if (segMsg.endOrientation - segMsg.startOrientation > 3 * M_PI) {
            segMsg.endOrientation -= 2 * M_PI;
        } else if (segMsg.endOrientation - segMsg.startOrientation < M_PI)
            segMsg.endOrientation += 2 * M_PI;
        segMsg.orientationDiff = segMsg.endOrientation - segMsg.startOrientation;
    }

    // caculate the rowID and colID, delete invalid point, saved into fullCloud
    // rangeMat: init_value is FLT_MAX(float); if point has valid ID rangeMat(row, col) = range
    // fullCloud: intensity -> row + col / 10000; if no valid ID  -1
    // fullInfoCloud: intensity -> range
    void projectPointCloud(){
        // range image projection
        float verticalAngle, horizonAngle, range;
        size_t rowIdn, columnIdn, index, cloudSize; 
        PointType thisPoint;

        cloudSize = laserCloudIn->points.size();

        vector<int> ring_num(N_SCAN, 0);
        cout << "cloudSize: " << cloudSize << endl;
        int valid_size = 0;

        vector<vector<int>> counts(N_SCAN, vector<int>(Horizon_SCAN, 0));
        

        for (size_t i = 0; i < cloudSize; ++i){

            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;

#ifdef SEMANTIC_KITTI
            pcl::PointXYZRGB cur_p;
            cur_p.x = laserCloudIn->points[i].x;
            cur_p.y = laserCloudIn->points[i].y;
            cur_p.z = laserCloudIn->points[i].z;
            int class_id = classes_map[laserCloudIn->points[i].label];
            cur_p.r = colors_map_tran[class_id][0];
            cur_p.g = colors_map_tran[class_id][1];
            cur_p.b = colors_map_tran[class_id][2];
            laserCloudColor->push_back(cur_p);
#endif

            // find the row and column index in the image for this point
            if (useCloudRing == true){
                rowIdn = laserCloudInRing->points[i].ring;
            }
            else{
                verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
                rowIdn = (verticalAngle + ang_bottom) / ang_res_y + 0.3;
            }
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            ring_num[rowIdn]++;

            horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

            columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;

            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            
            counts[rowIdn][columnIdn]++;
            if (counts[rowIdn][columnIdn] == 1)
                valid_size++;

            range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
            if (range < sensorMinimumRange)
                continue;
            
            rangeMat.at<float>(rowIdn, columnIdn) = range;

            thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

            index = columnIdn  + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
            fullInfoCloud->points[index] = thisPoint;
            fullInfoCloud->points[index].intensity = range; // the corresponding range of a point is saved as "intensity"

#ifdef SEMANTIC_KITTI
            index_full[index] = i;
#endif
        }

        cout << "valid size: " << valid_size << endl;
        for (auto &r : ring_num)
            cout << r << " ";
        cout << endl;

        for (auto &row : counts) {
            for (auto &col : row) {
                cout << col;
            }
            cout << endl;
        }

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

#ifdef SEMANTIC_KITTI
        for (size_t i = 0; i <= groundScanInd; ++i) {
            for (size_t j = 0; j < Horizon_SCAN; ++j) {
                if (groundMat.at<int8_t>(i,j) == 0 && rangeMat.at<float>(i,j) != FLT_MAX) {
                    laserCloud_noGround->push_back(laserCloudIn->points[index_full[j + i * Horizon_SCAN]]);
                }
            }
        }
#endif

    }

    void cloudSegmentation(){
        // segmentation process
        for (size_t i = 0; i < N_SCAN; ++i)
            for (size_t j = 0; j < Horizon_SCAN; ++j)
                // init value is 0, ground is already set to -1
                if (labelMat.at<int>(i,j) == 0)
                    labelComponents(i, j);

        int sizeOfSegCloud = 0;
        // extract segmented cloud for lidar odometry
        for (size_t i = 0; i < N_SCAN; ++i) {

            segMsg.startRingIndex[i] = sizeOfSegCloud-1 + 5;

            for (size_t j = 0; j < Horizon_SCAN; ++j) {
                if (labelMat.at<int>(i,j) > 0 || groundMat.at<int8_t>(i,j) == 1){
                    // outliers that will not be used for optimization (always continue)
                    if (labelMat.at<int>(i,j) == 999999){
                        if (i > groundScanInd && j % 5 == 0){
                            outlierCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                            continue;
                        }else{
                            continue;
                        }
                    }
                    // majority of ground points are skipped
                    if (groundMat.at<int8_t>(i,j) == 1){
                        if (j%5!=0 && j>5 && j<Horizon_SCAN-5)
                            continue;
                    }
                    // mark ground points so they will not be considered as edge features later
                    segMsg.segmentedCloudGroundFlag[sizeOfSegCloud] = (groundMat.at<int8_t>(i,j) == 1);
                    // mark the points' column index for marking occlusion later
                    segMsg.segmentedCloudColInd[sizeOfSegCloud] = j;
                    // save range info
                    segMsg.segmentedCloudRange[sizeOfSegCloud]  = rangeMat.at<float>(i,j);
                    // save seg cloud
                    segmentedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                    // size of seg cloud
                    ++sizeOfSegCloud;
                }
            }

            segMsg.endRingIndex[i] = sizeOfSegCloud-1 - 5;
        }
        
        // extract segmented cloud for visualization
        if (pubSegmentedCloudPure.getNumSubscribers() != 0){
            for (size_t i = 0; i < N_SCAN; ++i){
                for (size_t j = 0; j < Horizon_SCAN; ++j){
                    if (labelMat.at<int>(i,j) > 0 && labelMat.at<int>(i,j) != 999999){
                        segmentedCloudPure->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                        segmentedCloudPure->points.back().intensity = labelMat.at<int>(i,j);
                    }
                }
            }
        }
    }

    void labelComponents(int row, int col){
        // use std::queue std::vector std::deque will slow the program down greatly
        float d1, d2, alpha, angle;
        int fromIndX, fromIndY, thisIndX, thisIndY; 
        bool lineCountFlag[N_SCAN] = {false};

        queueIndX[0] = row;
        queueIndY[0] = col;
        int queueSize = 1;
        int queueStartInd = 0;
        int queueEndInd = 1;

        allPushedIndX[0] = row;
        allPushedIndY[0] = col;
        int allPushedIndSize = 1;
        
        while(queueSize > 0){
            // Pop point
            fromIndX = queueIndX[queueStartInd];
            fromIndY = queueIndY[queueStartInd];
            --queueSize;
            ++queueStartInd;
            // Mark popped point
            labelMat.at<int>(fromIndX, fromIndY) = labelCount;
            // Loop through all the neighboring grids of popped grid
            for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter){
                // new index
                thisIndX = fromIndX + (*iter).first;
                thisIndY = fromIndY + (*iter).second;
                // index should be within the boundary
                if (thisIndX < 0 || thisIndX >= N_SCAN)
                    continue;
                // at range image margin (left or right side)
                if (thisIndY < 0)
                    thisIndY = Horizon_SCAN - 1;
                if (thisIndY >= Horizon_SCAN)
                    thisIndY = 0;
                // prevent infinite loop (caused by put already examined point back)
                if (labelMat.at<int>(thisIndX, thisIndY) != 0)
                    continue;

                d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY), 
                              rangeMat.at<float>(thisIndX, thisIndY));
                d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY), 
                              rangeMat.at<float>(thisIndX, thisIndY));

                if ((*iter).first == 0)
                    alpha = segmentAlphaX;
                else
                    alpha = segmentAlphaY;

                angle = atan2(d2*sin(alpha), (d1 -d2*cos(alpha)));

                if (angle > segmentTheta){

                    queueIndX[queueEndInd] = thisIndX;
                    queueIndY[queueEndInd] = thisIndY;
                    ++queueSize;
                    ++queueEndInd;

                    labelMat.at<int>(thisIndX, thisIndY) = labelCount;
                    lineCountFlag[thisIndX] = true;

                    allPushedIndX[allPushedIndSize] = thisIndX;
                    allPushedIndY[allPushedIndSize] = thisIndY;
                    ++allPushedIndSize;
                }
            }
        }

        // check if this segment is valid
        bool feasibleSegment = false;
        if (allPushedIndSize >= 30)
            feasibleSegment = true;
        else if (allPushedIndSize >= segmentValidPointNum){
            int lineCount = 0;
            for (size_t i = 0; i < N_SCAN; ++i)
                if (lineCountFlag[i] == true)
                    ++lineCount;
            if (lineCount >= segmentValidLineNum)
                feasibleSegment = true;            
        }
        // segment is valid, mark these points
        if (feasibleSegment == true){
            ++labelCount;
        }else{ // segment is invalid, mark these points
            for (size_t i = 0; i < allPushedIndSize; ++i){
                labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
            }
        }
    }

    
    void publishCloud(){
        // 1. Publish Seg Cloud Info
        segMsg.header = cloudHeader;
        pubSegmentedCloudInfo.publish(segMsg);
        // 2. Publish clouds
        sensor_msgs::PointCloud2 laserCloudTemp;

        pcl::toROSMsg(*outlierCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "base_link";
        pubOutlierCloud.publish(laserCloudTemp);
        // segmented cloud with ground
        pcl::toROSMsg(*segmentedCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "base_link";
        pubSegmentedCloud.publish(laserCloudTemp);
        // projected full cloud
        if (pubFullCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*fullCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubFullCloud.publish(laserCloudTemp);
        }
        // original dense ground cloud
        if (pubGroundCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*groundCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubGroundCloud.publish(laserCloudTemp);
        }
        // segmented cloud without ground
        if (pubSegmentedCloudPure.getNumSubscribers() != 0){
            pcl::toROSMsg(*segmentedCloudPure, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubSegmentedCloudPure.publish(laserCloudTemp);
        }
        // projected full cloud info
        if (pubFullInfoCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*fullInfoCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubFullInfoCloud.publish(laserCloudTemp);
        }

#ifdef SEMANTIC_KITTI
#ifdef DEBUG_PUBLISH
        cout << "classifiedCloud: " << classifiedCloud->points.size() << endl;
        cout << "classifiedCentroid: " << classifiedCentroid->points.size() << endl;
#endif
        if (pubColoredLaserCloud.getNumSubscribers() != 0) {
            pcl::toROSMsg(*laserCloudColor, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubColoredLaserCloud.publish(laserCloudTemp);
        }

        if (pubClassifiedCloud.getNumSubscribers() != 0) {
            pcl::toROSMsg(*classifiedCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubClassifiedCloud.publish(laserCloudTemp);
        }

        if (pubClassifiedCentroidRGB.getNumSubscribers() != 0) {
            pcl::toROSMsg(*classifiedCentroidRGB, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubClassifiedCentroidRGB.publish(laserCloudTemp);
        }



        if (pubClassifiedCentroid.getNumSubscribers() != 0) {
            pcl::toROSMsg(*classifiedCentroid, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubClassifiedCentroid.publish(laserCloudTemp);
        }
        
        if (pubCentroid2Seg.getNumSubscribers() != 0) {
            pubCentroid2Seg.publish(marker_Centroid2Seg);
        }

        if (pubNoGroundCloud.getNumSubscribers() != 0) {
            pcl::toROSMsg(*laserCloud_noGround, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubNoGroundCloud.publish(laserCloudTemp);
        }
#endif
    }
};




int main(int argc, char** argv){

    ros::init(argc, argv, "lego_loam");
    
    ImageProjection IP;

    ROS_INFO("\033[1;32m---->\033[0m Image Projection Started.");

    ros::spin();
    return 0;
}
