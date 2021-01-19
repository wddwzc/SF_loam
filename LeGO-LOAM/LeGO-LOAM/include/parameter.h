#include "utility.h"

class ParamServer
{
public:

    ros::NodeHandle nh;

    bool semantic;


    // laser 
    int N_SCAN;
    int Horizon_SCAN;
    float ang_res_x;
    float ang_res_y;
    float ang_bottom;
    int groundScanInd;
    


    // kittiProcess
    bool fusion;

    // imageProjection
    bool use_cloud_ring;
    int segment_valid_point_num;
    int segment_valid_line_num;
    double segment_theta;
    
    bool debug_publish;
    bool debug_cluster;


    // featureAssociation
    double edge_threshold;
    double surf_threshold;
    int nearest_feature_search_distance;

    // mapping
    bool enable_loop_closure;
    int mapping_frequency_divider;

    double surrounding_keyframe_search_radius;
    int surrounding_keyframe_search_num;

    double history_keyframe_search_radius;
    int history_keyframe_search_num;
    double history_keyframe_fitness_score;

    double global_map_visualization_search_radius;


    // transformFusion




    ParamServer()
    {
        nh.param<bool>("/sf_loam/semantic", semantic, false);

        // kittiProcess
        nh.param<bool>("/kittiProcess/fusion", fusion, false);

        // imageProjection



        nh.param<std::string>("/robot_id", robot_id, "roboat");

        nh.param<std::string>("lio_sam/pointCloudTopic", pointCloudTopic, "points_raw");
        nh.param<std::string>("lio_sam/imuTopic", imuTopic, "imu_correct");
        nh.param<std::string>("lio_sam/odomTopic", odomTopic, "odometry/imu");
        nh.param<std::string>("lio_sam/gpsTopic", gpsTopic, "odometry/gps");


        nh.param<bool>("lio_sam/savePCD", savePCD, false);
        nh.param<std::string>("lio_sam/savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/");

        nh.param<int>("lio_sam/N_SCAN", N_SCAN, 16);
        nh.param<int>("lio_sam/Horizon_SCAN", Horizon_SCAN, 1800);
        nh.param<std::string>("lio_sam/timeField", timeField, "time");
        nh.param<int>("lio_sam/downsampleRate", downsampleRate, 1);

        nh.param<float>("lio_sam/imuAccNoise", imuAccNoise, 0.01);
        nh.param<float>("lio_sam/imuGyrNoise", imuGyrNoise, 0.001);
        nh.param<float>("lio_sam/imuAccBiasN", imuAccBiasN, 0.0002);
        nh.param<float>("lio_sam/imuGyrBiasN", imuGyrBiasN, 0.00003);
        nh.param<float>("lio_sam/imuGravity", imuGravity, 9.80511);
        nh.param<vector<double>>("lio_sam/extrinsicRot", extRotV, vector<double>());
        nh.param<vector<double>>("lio_sam/extrinsicRPY", extRPYV, vector<double>());
        nh.param<vector<double>>("lio_sam/extrinsicTrans", extTransV, vector<double>());
        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY);

        nh.param<float>("lio_sam/edgeThreshold", edgeThreshold, 0.1);
        nh.param<float>("lio_sam/surfThreshold", surfThreshold, 0.1);
        nh.param<int>("lio_sam/edgeFeatureMinValidNum", edgeFeatureMinValidNum, 10);
        nh.param<int>("lio_sam/surfFeatureMinValidNum", surfFeatureMinValidNum, 100);

        nh.param<float>("lio_sam/odometrySurfLeafSize", odometrySurfLeafSize, 0.2);
        nh.param<float>("lio_sam/mappingCornerLeafSize", mappingCornerLeafSize, 0.2);
        nh.param<float>("lio_sam/mappingSurfLeafSize", mappingSurfLeafSize, 0.2);

        nh.param<float>("lio_sam/z_tollerance", z_tollerance, FLT_MAX);
        nh.param<float>("lio_sam/rotation_tollerance", rotation_tollerance, FLT_MAX);

        nh.param<int>("lio_sam/numberOfCores", numberOfCores, 2);
        nh.param<double>("lio_sam/mappingProcessInterval", mappingProcessInterval, 0.15);

        nh.param<float>("lio_sam/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0);
        nh.param<float>("lio_sam/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);
        nh.param<float>("lio_sam/surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0);
        nh.param<float>("lio_sam/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);

        nh.param<bool>("lio_sam/loopClosureEnableFlag", loopClosureEnableFlag, false);
        nh.param<int>("lio_sam/surroundingKeyframeSize", surroundingKeyframeSize, 50);
        nh.param<float>("lio_sam/historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);
        nh.param<float>("lio_sam/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
        nh.param<int>("lio_sam/historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
        nh.param<float>("lio_sam/historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3);

        nh.param<float>("lio_sam/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3);
        nh.param<float>("lio_sam/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
        nh.param<float>("lio_sam/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);

        usleep(100);
    }





    // string pointCloudTopic;
    // string imuTopic;
    // string odomTopic;

    // // Save pcd
    // bool savePCD;
    // string savePCDDirectory;

    // // Velodyne Sensor Configuration: Velodyne
    // int N_SCAN;
    // int Horizon_SCAN;
    // string timeField;
    // int downsampleRate;

    // // IMU
    // float imuAccNoise;
    // float imuGyrNoise;
    // float imuAccBiasN;
    // float imuGyrBiasN;
    // float imuGravity;
    // vector<double> extRotV;
    // vector<double> extRPYV;
    // vector<double> extTransV;
    // Eigen::Matrix3d extRot;
    // Eigen::Matrix3d extRPY;
    // Eigen::Vector3d extTrans;
    // Eigen::Quaterniond extQRPY;

    // // LOAM
    // float edgeThreshold;
    // float surfThreshold;
    // int edgeFeatureMinValidNum;
    // int surfFeatureMinValidNum;

    // // voxel filter paprams
    // float odometrySurfLeafSize;
    // float mappingCornerLeafSize;
    // float mappingSurfLeafSize ;

    // float z_tollerance; 
    // float rotation_tollerance;

    // // CPU Params
    // int numberOfCores;
    // double mappingProcessInterval;

    // // Surrounding map
    // float surroundingkeyframeAddingDistThreshold; 
    // float surroundingkeyframeAddingAngleThreshold; 
    // float surroundingKeyframeDensity;
    // float surroundingKeyframeSearchRadius;
    
    // // Loop closure
    // bool loopClosureEnableFlag;
    // int   surroundingKeyframeSize;
    // float historyKeyframeSearchRadius;
    // float historyKeyframeSearchTimeDiff;
    // int   historyKeyframeSearchNum;
    // float historyKeyframeFitnessScore;

    // // global map visualization radius
    // float globalMapVisualizationSearchRadius;
    // float globalMapVisualizationPoseDensity;
    // float globalMapVisualizationLeafSize;

    // ParamServer()
    // {

    //     // kittiProcess
    //     nh.param<bool>("/kittiProcess/fusion", fusion, false);

    //     // 





    //     nh.param<std::string>("/robot_id", robot_id, "roboat");

    //     nh.param<std::string>("lio_sam/pointCloudTopic", pointCloudTopic, "points_raw");
    //     nh.param<std::string>("lio_sam/imuTopic", imuTopic, "imu_correct");
    //     nh.param<std::string>("lio_sam/odomTopic", odomTopic, "odometry/imu");
    //     nh.param<std::string>("lio_sam/gpsTopic", gpsTopic, "odometry/gps");


    //     nh.param<bool>("lio_sam/savePCD", savePCD, false);
    //     nh.param<std::string>("lio_sam/savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/");

    //     nh.param<int>("lio_sam/N_SCAN", N_SCAN, 16);
    //     nh.param<int>("lio_sam/Horizon_SCAN", Horizon_SCAN, 1800);
    //     nh.param<std::string>("lio_sam/timeField", timeField, "time");
    //     nh.param<int>("lio_sam/downsampleRate", downsampleRate, 1);

    //     nh.param<float>("lio_sam/imuAccNoise", imuAccNoise, 0.01);
    //     nh.param<float>("lio_sam/imuGyrNoise", imuGyrNoise, 0.001);
    //     nh.param<float>("lio_sam/imuAccBiasN", imuAccBiasN, 0.0002);
    //     nh.param<float>("lio_sam/imuGyrBiasN", imuGyrBiasN, 0.00003);
    //     nh.param<float>("lio_sam/imuGravity", imuGravity, 9.80511);
    //     nh.param<vector<double>>("lio_sam/extrinsicRot", extRotV, vector<double>());
    //     nh.param<vector<double>>("lio_sam/extrinsicRPY", extRPYV, vector<double>());
    //     nh.param<vector<double>>("lio_sam/extrinsicTrans", extTransV, vector<double>());
    //     extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
    //     extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
    //     extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
    //     extQRPY = Eigen::Quaterniond(extRPY);

    //     nh.param<float>("lio_sam/edgeThreshold", edgeThreshold, 0.1);
    //     nh.param<float>("lio_sam/surfThreshold", surfThreshold, 0.1);
    //     nh.param<int>("lio_sam/edgeFeatureMinValidNum", edgeFeatureMinValidNum, 10);
    //     nh.param<int>("lio_sam/surfFeatureMinValidNum", surfFeatureMinValidNum, 100);

    //     nh.param<float>("lio_sam/odometrySurfLeafSize", odometrySurfLeafSize, 0.2);
    //     nh.param<float>("lio_sam/mappingCornerLeafSize", mappingCornerLeafSize, 0.2);
    //     nh.param<float>("lio_sam/mappingSurfLeafSize", mappingSurfLeafSize, 0.2);

    //     nh.param<float>("lio_sam/z_tollerance", z_tollerance, FLT_MAX);
    //     nh.param<float>("lio_sam/rotation_tollerance", rotation_tollerance, FLT_MAX);

    //     nh.param<int>("lio_sam/numberOfCores", numberOfCores, 2);
    //     nh.param<double>("lio_sam/mappingProcessInterval", mappingProcessInterval, 0.15);

    //     nh.param<float>("lio_sam/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0);
    //     nh.param<float>("lio_sam/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);
    //     nh.param<float>("lio_sam/surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0);
    //     nh.param<float>("lio_sam/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);

    //     nh.param<bool>("lio_sam/loopClosureEnableFlag", loopClosureEnableFlag, false);
    //     nh.param<int>("lio_sam/surroundingKeyframeSize", surroundingKeyframeSize, 50);
    //     nh.param<float>("lio_sam/historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);
    //     nh.param<float>("lio_sam/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
    //     nh.param<int>("lio_sam/historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
    //     nh.param<float>("lio_sam/historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3);

    //     nh.param<float>("lio_sam/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3);
    //     nh.param<float>("lio_sam/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
    //     nh.param<float>("lio_sam/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);

    //     usleep(100);
    // }
};

