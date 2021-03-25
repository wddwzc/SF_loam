#ifndef _PARAMETER_H_
#define _PARAMETER_H_


#include "utility.h"

class ParamServer
{
public:

    ros::NodeHandle nh;

    // Semantic
    bool semantic;
    std::string pointCloudTopic;
    std::string semanticPointCloudTopic;
    std::string imuDataTopic;
    std::string odometryTopic;
    int imuQueLength;

    // laser 
    bool isVLP64;
    int N_SCAN;
    int Horizon_SCAN;
    float ang_res_x;
    float ang_res_y;
    float ang_top;
    float ang_bottom;
    int groundScanInd;
    float scanPeriod;

    // kittiProcess
    bool fusion;
    std::string laserTopic;
    std::string labelTopic;
    std::string allTopic;

    // imageProjection
    // Using velodyne cloud "ring" channel for image projection (other lidar may have different name for this channel, change "PointXYZIR" below)
    bool useCloudRing;  // if true, ang_res_y and ang_bottom are not used
    
    float sensorMinimumRange;
    float sensorMountAngle;
    float sensorToGround;
    bool filterGroundNoise;
    bool useSimpleMethod;

    float segmentTheta;
    int segmentValidPointNum;
    int segmentValidLineNum;
    float segmentAlphaX;
    float segmentAlphaY;
    
    bool debugPublish;
    bool debugCluster;
    bool debugTimeCost;


    // featureAssociation
    float edgeThreshold;
    float surfThreshold;
    float nearestFeatureSearchSqDist;

    // mapping
    double mappingProcessInterval;

    bool loopClosureEnableFlag;
    std::string scSourceData;
    int mappingFrequencyDivider;

    float surroundingKeyframeSearchRadius;
    int surroundingKeyframeSearchNum;

    float historyKeyframeSearchRadius;
    int historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;

    float globalMapVisualizationSearchRadius;

    std::string fileDirectory;

    bool debugLoopClosure;

    // transformFusion

    // local map
    float localMapRadius;

    bool usePclSegmentation;
    float extractPlaneThreshold;
    float octomapLeafSize;
    float voxelLeafSize;

    std::string clusterMethod;
    float radiusGrowing;
    int coreMinPts;
    int minClusterSize;
    int maxClusterSize;

    float dynamicSegmentSearchRadius;

    bool debugOctomapGenerator;
    bool debugOctomapCluster;
    bool debugOctomapPublish;
    bool debugOctomapTimeCost;


    ParamServer()
    {
        // if use semantic label
        nh.param<bool>("/sf_loam/semantic", semantic, false);
        
        // input for sf_loam
        nh.param<std::string>("/sf_loam/point_cloud_topic",
                                pointCloudTopic, "/kitti/velo/pointxyzi");
        nh.param<std::string>("/sf_loam/semantic_point_cloud_topic",
                                semanticPointCloudTopic, "/kitti/velo/pointall");
        nh.param<std::string>("/sf_loam/imu_data_topic", imuDataTopic, "/imu/data");
        nh.param<std::string>("/sf_loam/odometry_topic", odometryTopic, "/kitti/velodyne_poses");
        nh.param<int>("/sf_loam/imu_que_length", imuQueLength, 200);

        // laser parameters (default: VLP64)
        nh.param<bool>("/laser/is_vlp64", isVLP64, true);
        nh.param<int>("/laser/num_vertical_scans", N_SCAN, 64);
        nh.param<int>("/laser/num_horizontal_scans", Horizon_SCAN, 2250);
        nh.param<float>("/laser/ang_vertical_res", ang_res_y, 0.472);
        nh.param<float>("/laser/ang_horizontal_res", ang_res_x, 0.16);
        nh.param<float>("/laser/ang_vertical_top", ang_top, 2.0);
        nh.param<float>("/laser/ang_vertical_bottom", ang_bottom, 24.9);
        nh.param<int>("/laser/ground_scan_index", groundScanInd, 50);
        nh.param<float>("/laser/scan_period", scanPeriod, 0.1);

        // kittiProcess
        nh.param<bool>("/kittiProcess/fusion", fusion, false);

        // imageProjection
        nh.param<bool>("/sf_loam/imageProjection/use_cloud_ring", useCloudRing, false);
        nh.param<float>("/sf_loam/imageProjection/sensor_minimum_range", sensorMinimumRange, 1.0);
        nh.param<float>("/sf_loam/imageProjection/sensor_mount_angle", sensorMountAngle, 0.0);
        nh.param<float>("/sf_loam/imageProjection/sensor_to_ground", sensorToGround, 1.0);
        nh.param<bool>("/sf_loam/imageProjection/filter_ground_noise", filterGroundNoise, false);
        nh.param<bool>("/sf_loam/imageProjection/use_simple_method", useSimpleMethod, true);
        nh.param<float>("/sf_loam/imageProjection/segment_theta", segmentTheta, 60.0);
        segmentTheta /= 180.0 * M_PI;
        nh.param<int>("/sf_loam/imageProjection/segment_valid_point_num", segmentValidPointNum, 5);
        nh.param<int>("/sf_loam/imageProjection/segment_valid_line_num", segmentValidLineNum, 3);
        nh.param<float>("/sf_loam/imageProjection/segment_valid_line_num", segmentAlphaX, 1.0);
        nh.param<float>("/sf_loam/imageProjection/segment_valid_line_num", segmentAlphaY, 1.0);
        segmentAlphaX *= ang_res_x / 180.0 * M_PI;
        segmentAlphaY *= ang_res_y / 180.0 * M_PI;
        nh.param<bool>("/sf_loam/imageProjection/debug_publish", debugPublish, false);
        nh.param<bool>("/sf_loam/imageProjection/debug_cluster", debugCluster, false);
        nh.param<bool>("/sf_loam/imageProjection/debug_timecost", debugTimeCost, false);

        // featureAssociation
        nh.param<float>("/sf_loam/featureAssociation/edge_threshold", edgeThreshold, 0.1);
        nh.param<float>("/sf_loam/featureAssociation/surf_threshold", surfThreshold, 0.1);
        nh.param<float>("/sf_loam/featureAssociation/nearest_feature_search_square_distance", 
                            nearestFeatureSearchSqDist, 25);
        // nh.param<bool>("/sf_loam/featureAssociation/debug_cluster", debug_cluster, false);
        // nh.param<bool>("/sf_loam/featureAssociation/debug_cluster", debug_cluster, false);
        // nh.param<bool>("/sf_loam/featureAssociation/debug_cluster", debug_cluster, false);
        // nh.param<bool>("/sf_loam/featureAssociation/debug_cluster", debug_cluster, false);
        // nh.param<bool>("/sf_loam/featureAssociation/debug_cluster", debug_cluster, false);
        // nh.param<bool>("/sf_loam/featureAssociation/debug_cluster", debug_cluster, false);

        // Mapping
        nh.param<double>("/sf_loam/mapping/mapping_process_interval", 
                        mappingProcessInterval, 0.3);
        nh.param<std::string>("/sf_loam/mapping/sc_source_data", 
                        scSourceData, "raw");
        nh.param<bool>("/sf_loam/mapping/enable_loop_closure", 
                        loopClosureEnableFlag, false);
        nh.param<float>("/sf_loam/mapping/surrounding_keyframe_search_radius", 
                        surroundingKeyframeSearchRadius, 50.0);
        nh.param<int>("/sf_loam/mapping/surrounding_keyframe_search_num", 
                        surroundingKeyframeSearchNum, 50);
        nh.param<float>("/sf_loam/mapping/history_keyframe_search_radius", 
                        historyKeyframeSearchRadius, 7.0);
        nh.param<int>("/sf_loam/mapping/history_keyframe_search_num", 
                        historyKeyframeSearchNum, 25);
        nh.param<float>("/sf_loam/mapping/history_keyframe_fitness_score", 
                        historyKeyframeFitnessScore, 0.3);
        nh.param<float>("/sf_loam/mapping/global_map_visualization_search_radius", 
                        globalMapVisualizationSearchRadius, 500.0);
        nh.param<std::string>("/sf_loam/mapping/file_directory", 
                        fileDirectory, "/tmp/");
        nh.param<bool>("/sf_loam/mapping/debug_loopclosure", 
                        debugLoopClosure, false);

        // LocalMap
        nh.param<float>("/local_map/local_map_radius", localMapRadius, 50.0);

        nh.param<bool>("/local_map/use_pcl_segmentation", usePclSegmentation, true);
        nh.param<float>("/local_map/extract_plane_threashold", extractPlaneThreshold, 0.05);
        nh.param<float>("/local_map/voxel_leaf_size", voxelLeafSize, 0.1);
        nh.param<float>("/local_map/octomap_leaf_size", octomapLeafSize, 0.1);

        nh.param<std::string>("/local_map/cluster_method", clusterMethod, "DBSCAN");  
        nh.param<float>("/local_map/radius_growing", radiusGrowing, 0.2);
        nh.param<int>("/local_map/core_min_points", coreMinPts, 10);
        nh.param<int>("/local_map/min_cluster_size", minClusterSize, 100);
        nh.param<int>("/local_map/max_cluster_size", maxClusterSize, 20000);

        nh.param<float>("/local_map/dynamic_segment_search_radius", dynamicSegmentSearchRadius, 2.0);
        
        nh.param<bool>("/local_map/debug_generator", debugOctomapGenerator, false);
        nh.param<bool>("/local_map/debug_cluster", debugOctomapCluster, false);
        nh.param<bool>("/local_map/debug_publish", debugOctomapPublish, false);
        nh.param<bool>("/local_map/debug_time_cost", debugOctomapTimeCost, false);

        usleep(100);
    }

    ~ParamServer() {

    }
};

#endif
