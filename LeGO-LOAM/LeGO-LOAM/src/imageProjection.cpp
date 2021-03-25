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
#include "parameter.h"
#include "kitti_label_color.h"

class ImageProjection {
private:
    ParamServer param;

    ros::NodeHandle nh;

    ros::Subscriber subLaserCloud;
    
    ros::Publisher pubFullCloud;
    ros::Publisher pubFullInfoCloud;

    ros::Publisher pubGroundCloud;
    ros::Publisher pubSegmentedCloud;
    ros::Publisher pubSegmentedCloudPure;
    ros::Publisher pubSegmentedCloudInfo;
    ros::Publisher pubOutlierCloud;

    // Semantic
    ros::Publisher pubColoredLaserCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudColor;


    pcl::PointCloud<PointType>::Ptr laserCloudIn;
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

public:
    ImageProjection():
        nh("~") {

        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(param.semanticPointCloudTopic, 1, &ImageProjection::cloudHandler, this);

        pubFullCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_projected", 1);
        pubFullInfoCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_info", 1);

        pubGroundCloud = nh.advertise<sensor_msgs::PointCloud2> ("/ground_cloud", 1);
        pubSegmentedCloud = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud", 1);
        pubSegmentedCloudPure = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud_pure", 1);
        pubSegmentedCloudInfo = nh.advertise<cloud_msgs::cloud_info> ("/segmented_cloud_info", 1);
        pubOutlierCloud = nh.advertise<sensor_msgs::PointCloud2> ("/outlier_cloud", 1);

        nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.label = 0;
        nanPoint.intensity = -1;

        semanticInit();

        allocateMemory();
        resetParameters();
    }

    void semanticInit() {

        pubColoredLaserCloud = nh.advertise<sensor_msgs::PointCloud2> ("/A_laser_cloud_color", 1);
        laserCloudColor.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

        // for visulize
        // pubColoredLaserCloud = nh.advertise<sensor_msgs::PointCloud2> ("/A_laser_cloud_color", 1);
        // pubClassifiedCentroidRGB = nh.advertise<sensor_msgs::PointCloud2> ("/A_classified_centroid_color", 1);
        // pubClassifiedCloud = nh.advertise<sensor_msgs::PointCloud2> ("/A_classified_cloud", 1);
        // pubCentroid2Seg = nh.advertise<visualization_msgs::Marker>("/A_marker_centroid2seg", 1);
        // // for mapOptimization
        // pubClassifiedCentroid = nh.advertise<sensor_msgs::PointCloud2> ("/A_classified_centroid", 1);
        // pubNoGroundCloud = nh.advertise<sensor_msgs::PointCloud2> ("/A_laser_cloud_noground", 1);

        
        // classifiedCentroidRGB.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        // classifiedCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        // laserCloudColor.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        // classifiedCentroid.reset(new pcl::PointCloud<pcl::PointXYZL>());
        // laserCloud_noGround.reset(new pcl::PointCloud<pcl::PointXYZL>());

        // // index_full.resize(param.N_SCAN*param.Horizon_SCAN);

        // marker_Centroid2Seg.header.frame_id = "base_link";
        // marker_Centroid2Seg.header.stamp = ros::Time();
        // marker_Centroid2Seg.ns = "my_namespace";
        // marker_Centroid2Seg.id = 0;
        // marker_Centroid2Seg.type = visualization_msgs::Marker::LINE_LIST;
        // marker_Centroid2Seg.action = visualization_msgs::Marker::ADD;
        // marker_Centroid2Seg.pose.orientation.w = 1.0;
        // marker_Centroid2Seg.scale.x = 0.1;
        // marker_Centroid2Seg.color.r = 1.0f;
        // marker_Centroid2Seg.color.a = 1.0f;
    }

    void allocateMemory(){
        laserCloudIn.reset(new pcl::PointCloud<PointType>());
        laserCloudInRing.reset(new pcl::PointCloud<PointXYZIR>());

        fullCloud.reset(new pcl::PointCloud<PointType>());
        fullInfoCloud.reset(new pcl::PointCloud<PointType>());

        groundCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloudPure.reset(new pcl::PointCloud<PointType>());
        outlierCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(param.N_SCAN*param.Horizon_SCAN);
        fullInfoCloud->points.resize(param.N_SCAN*param.Horizon_SCAN);

        segMsg.startRingIndex.assign(param.N_SCAN, 0);
        segMsg.endRingIndex.assign(param.N_SCAN, 0);

        segMsg.segmentedCloudGroundFlag.assign(param.N_SCAN*param.Horizon_SCAN, false);
        segMsg.segmentedCloudColInd.assign(param.N_SCAN*param.Horizon_SCAN, 0);
        segMsg.segmentedCloudRange.assign(param.N_SCAN*param.Horizon_SCAN, 0);

        std::pair<int8_t, int8_t> neighbor;
        neighbor.first = -1; neighbor.second =  0; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second =  1; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second = -1; neighborIterator.push_back(neighbor);
        neighbor.first =  1; neighbor.second =  0; neighborIterator.push_back(neighbor);

        allPushedIndX = new uint16_t[param.N_SCAN*param.Horizon_SCAN];
        allPushedIndY = new uint16_t[param.N_SCAN*param.Horizon_SCAN];

        queueIndX = new uint16_t[param.N_SCAN*param.Horizon_SCAN];
        queueIndY = new uint16_t[param.N_SCAN*param.Horizon_SCAN];
    }

    void resetParameters(){
        laserCloudIn->clear();
        groundCloud->clear();
        segmentedCloud->clear();
        segmentedCloudPure->clear();
        outlierCloud->clear();

        // semantic
        laserCloudColor->clear();

        rangeMat = cv::Mat(param.N_SCAN, param.Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        groundMat = cv::Mat(param.N_SCAN, param.Horizon_SCAN, CV_8S, cv::Scalar::all(0));
        labelMat = cv::Mat(param.N_SCAN, param.Horizon_SCAN, CV_32S, cv::Scalar::all(0));
        labelCount = 1;

        std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
        std::fill(fullInfoCloud->points.begin(), fullInfoCloud->points.end(), nanPoint);
    }

    ~ImageProjection(){}

    void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        cloudHeader = laserCloudMsg->header;

        // cloudHeader.stamp = ros::Time::now(); // Ouster lidar users may need to uncomment this line
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
        // Remove Nan points
        std::vector<int> indices;

        if (param.debugCluster) cout << "bef: " << laserCloudIn->points.size() << endl;
        pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
        if (param.debugCluster) cout << "aft: " << laserCloudIn->points.size() << endl;

        // have "ring" channel in the cloud
        if (param.useCloudRing == true){
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
        // 6. Publish all clouds
        publishCloud();
        // 7. Reset parameters for next iteration
        resetParameters();
    }


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

        for (size_t i = 0; i < cloudSize; ++i){

            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;

            if (param.semantic) {
                thisPoint.label = classes_map[laserCloudIn->points[i].label];

                pcl::PointXYZRGB cur_p;
                cur_p.x = laserCloudIn->points[i].x;
                cur_p.y = laserCloudIn->points[i].y;
                cur_p.z = laserCloudIn->points[i].z;
                int class_id = classes_map[laserCloudIn->points[i].label];
                cur_p.r = colors_map_tran[class_id][0];
                cur_p.g = colors_map_tran[class_id][1];
                cur_p.b = colors_map_tran[class_id][2];
                laserCloudColor->push_back(cur_p);
            }
            
            // 64线和普通的不一样
            if (param.isVLP64) {
                verticalAngle = atan(thisPoint.y / sqrt(thisPoint.x * thisPoint.x + thisPoint.z * thisPoint.z)) * 180 / PI_M;
                if (verticalAngle > -8.7) {
                    rowIdn = int(63 - fabs(verticalAngle - 2) * 3 + 0.5);
                } else {
                    rowIdn = int(0 + (fabs(verticalAngle + 24.87) * 2 + 0.5));
                }
                if (rowIdn >= param.N_SCAN || rowIdn < 0) {
                    continue;
                }
            }
            else {
                // find the row and column index in the image for this point
                if (param.useCloudRing == true){
                    rowIdn = laserCloudInRing->points[i].ring;
                }
                else{
                    verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
                    rowIdn = (verticalAngle + param.ang_bottom) / param.ang_res_y + 0.3;
                }
                if (rowIdn < 0 || rowIdn >= param.N_SCAN)
                    continue;
            }

            horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

            columnIdn = -round((horizonAngle-90.0)/param.ang_res_x) + param.Horizon_SCAN/2;
            if (columnIdn >= param.Horizon_SCAN)
                columnIdn -= param.Horizon_SCAN;

            if (columnIdn < 0 || columnIdn >= param.Horizon_SCAN)
                continue;
            
            range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
            if (range < param.sensorMinimumRange)
                continue;
            
            rangeMat.at<float>(rowIdn, columnIdn) = range;

            thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

            index = columnIdn  + rowIdn * param.Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
            fullInfoCloud->points[index] = thisPoint;
            fullInfoCloud->points[index].intensity = range; // the corresponding range of a point is saved as "intensity"

        }
    }

    // save ground points into groundCloud
    // groundMat: -1 no valid info; 1 ground; 0 init_value
    void groundRemoval() {
        size_t lowerInd, upperInd;
        float diffX, diffY, diffZ, angle;
        // groundMat
        // -1, no valid info to check if ground of not
        //  0, initial value, after validation, means not ground
        //  1, ground
        for (size_t j = 0; j < param.Horizon_SCAN; ++j){
            for (size_t i = 0; i < param.groundScanInd; ++i){

                lowerInd = j + ( i )*param.Horizon_SCAN;
                upperInd = j + (i+1)*param.Horizon_SCAN;

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

                if (abs(angle - param.sensorMountAngle) <= 10){
                //if (abs(angle - sensorMountAngle) <= 5){
                // if (abs(angle - sensorMountAngle) <= 8){
                    if (param.filterGroundNoise && param.useSimpleMethod)
                        if (fullCloud->points[upperInd].z > -param.sensorToGround || fullCloud->points[lowerInd].z > -param.sensorToGround)
                            continue;
                    groundMat.at<int8_t>(i,j) = 1;
                    groundMat.at<int8_t>(i+1,j) = 1;
                }
            }
        }



        // extract ground cloud (groundMat == 1)
        // mark entry that doesn't need to label (ground and invalid point) for segmentation
        // note that ground remove is from 0~N_SCAN-1, need rangeMat for mark label matrix for the 16th scan
        for (size_t i = 0; i < param.N_SCAN; ++i){
            for (size_t j = 0; j < param.Horizon_SCAN; ++j){
                if (groundMat.at<int8_t>(i,j) == 1 || rangeMat.at<float>(i,j) == FLT_MAX){
                    labelMat.at<int>(i,j) = -1;
                }
            }
        }

        if (pubGroundCloud.getNumSubscribers() != 0){
            for (size_t i = 0; i <= param.groundScanInd; ++i){
                for (size_t j = 0; j < param.Horizon_SCAN; ++j){
                    if (groundMat.at<int8_t>(i,j) == 1)
                        groundCloud->push_back(fullCloud->points[j + i*param.Horizon_SCAN]);
                }
            }
        }

    }

    void cloudSegmentation(){
        // segmentation process
        for (size_t i = 0; i < param.N_SCAN; ++i)
            for (size_t j = 0; j < param.Horizon_SCAN; ++j)
                // init value is 0, ground is already set to -1
                if (labelMat.at<int>(i,j) == 0)
                    labelComponents(i, j);

        int sizeOfSegCloud = 0;
        // extract segmented cloud for lidar odometry
        for (size_t i = 0; i < param.N_SCAN; ++i) {

            segMsg.startRingIndex[i] = sizeOfSegCloud-1 + 5;

            for (size_t j = 0; j < param.Horizon_SCAN; ++j) {
                if (labelMat.at<int>(i,j) > 0 || groundMat.at<int8_t>(i,j) == 1){
                    // outliers that will not be used for optimization (always continue)
                    if (labelMat.at<int>(i,j) == 999999){
                        if (i > param.groundScanInd && j % 5 == 0){
                            outlierCloud->push_back(fullCloud->points[j + i*param.Horizon_SCAN]);
                            continue;
                        }else{
                            continue;
                        }
                    }
                    // majority of ground points are skipped
                    if (groundMat.at<int8_t>(i,j) == 1){
                        if (j%5!=0 && j>5 && j<param.Horizon_SCAN-5)
                            continue;
                    }
                    // mark ground points so they will not be considered as edge features later
                    segMsg.segmentedCloudGroundFlag[sizeOfSegCloud] = (groundMat.at<int8_t>(i,j) == 1);
                    // mark the points' column index for marking occlusion later
                    segMsg.segmentedCloudColInd[sizeOfSegCloud] = j;
                    // save range info
                    segMsg.segmentedCloudRange[sizeOfSegCloud]  = rangeMat.at<float>(i,j);
                    // save seg cloud
                    segmentedCloud->push_back(fullCloud->points[j + i*param.Horizon_SCAN]);
                    // size of seg cloud
                    ++sizeOfSegCloud;
                }
            }

            segMsg.endRingIndex[i] = sizeOfSegCloud-1 - 5;
        }
        
        // extract segmented cloud for visualization
        if (pubSegmentedCloudPure.getNumSubscribers() != 0){
            for (size_t i = 0; i < param.N_SCAN; ++i){
                for (size_t j = 0; j < param.Horizon_SCAN; ++j){
                    if (labelMat.at<int>(i,j) > 0 && labelMat.at<int>(i,j) != 999999){
                        segmentedCloudPure->push_back(fullCloud->points[j + i*param.Horizon_SCAN]);
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
        bool lineCountFlag[param.N_SCAN] = {false};

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
                if (thisIndX < 0 || thisIndX >= param.N_SCAN)
                    continue;
                // at range image margin (left or right side)
                if (thisIndY < 0)
                    thisIndY = param.Horizon_SCAN - 1;
                if (thisIndY >= param.Horizon_SCAN)
                    thisIndY = 0;
                // prevent infinite loop (caused by put already examined point back)
                if (labelMat.at<int>(thisIndX, thisIndY) != 0)
                    continue;

                d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY), 
                              rangeMat.at<float>(thisIndX, thisIndY));
                d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY), 
                              rangeMat.at<float>(thisIndX, thisIndY));

                if ((*iter).first == 0)
                    alpha = param.segmentAlphaX;
                else
                    alpha = param.segmentAlphaY;

                angle = atan2(d2*sin(alpha), (d1 -d2*cos(alpha)));

                if (angle > param.segmentTheta){

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
        else if (allPushedIndSize >= param.segmentValidPointNum){
            int lineCount = 0;
            for (size_t i = 0; i < param.N_SCAN; ++i)
                if (lineCountFlag[i] == true)
                    ++lineCount;
            if (lineCount >= param.segmentValidLineNum)
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

        if (param.debugPublish) {
            cout << "# ImageProjection Publisher ######################" << endl;
            cout << "FullCloud: " << fullCloud->points.size() << endl;                      // N_SCAN * HORIZONTAL_SCAN
            cout << "SegmentedCloud: " << segmentedCloud->points.size() << endl;            // segmented + ground
            cout << "SegmentedCloudPure: " << segmentedCloudPure->points.size() << endl;    // segmented
            cout << "GroundCloud: " << groundCloud->points.size() << endl;                  // ground
            cout << "OutlierCloud: " << outlierCloud->points.size() << endl;                // outlier (after cluster)
        }

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

        
        if (pubColoredLaserCloud.getNumSubscribers() != 0) {
            pcl::toROSMsg(*laserCloudColor, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubColoredLaserCloud.publish(laserCloudTemp);
        }
        
    }
};




int main(int argc, char** argv){

    ros::init(argc, argv, "lego_loam");
    
    ImageProjection IP;

    ROS_INFO("\033[1;32m---->\033[0m Image Projection Started.");

    ros::spin();
    return 0;
}
