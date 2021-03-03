#ifndef FEATUREASSOCIATION_H
#define FEATUREASSOCIATION_H

#include "utility.h"
#include "channel.h"
#include "nanoflann_pcl.h"
#include <Eigen/Eigenvalues>
#include <Eigen/QR>

class FeatureAssociation {

 public:
  FeatureAssociation( ros::NodeHandle& node,
                     Channel<ProjectionOut>& input_channel,
                     Channel<AssociationOut>& output_channel);

  ~FeatureAssociation();

  void imuHandler(const sensor_msgs::Imu::ConstPtr &imuIn) ;
  void runFeatureAssociation();

 private:

  enum {imuQueLength = 200};

  ros::NodeHandle& nh;

  int _vertical_scans;
  int _horizontal_scans;
  float _scan_period;
  float _edge_threshold;
  float _surf_threshold;
  float _nearest_feature_dist_sqr;
  int _mapping_frequency_div;

  std::mutex _imu_mutex;
  std::thread _run_thread;

  Channel<ProjectionOut>& _input_channel;
  Channel<AssociationOut>& _output_channel;

  ros::Subscriber subImu;

  ros::Publisher pubCornerPointsSharp;
  ros::Publisher pubCornerPointsLessSharp;
  ros::Publisher pubSurfPointsFlat;
  ros::Publisher pubSurfPointsLessFlat;

  pcl::PointCloud<PointType>::Ptr segmentedCloud;
  pcl::PointCloud<PointType>::Ptr outlierCloud;

  pcl::PointCloud<PointType>::Ptr cornerPointsSharp;
  pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;
  pcl::PointCloud<PointType>::Ptr surfPointsFlat;
  pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;

  pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan;
  pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScanDS;

  pcl::VoxelGrid<PointType> downSizeFilter;

  double timeScanCur;

  cloud_msgs::cloud_info segInfo;
  std_msgs::Header cloudHeader;

  int systemInitCount;
  bool systemInited;

  std::vector<smoothness_t> cloudSmoothness;
  std::vector<float> cloudCurvature;
  std::vector<int> cloudNeighborPicked;
  std::vector<int> cloudLabel;

  int imuPointerFront;
  int imuPointerLast;
  int imuPointerLastIteration;

  float imuRollStart, imuPitchStart, imuYawStart;
  float cosImuRollStart, cosImuPitchStart, cosImuYawStart, sinImuRollStart,
      sinImuPitchStart, sinImuYawStart;
  float imuRollCur, imuPitchCur, imuYawCur;

  Vector3 imuVeloStart;
  Vector3 imuShiftStart;

  Vector3 imuVeloCur;
  Vector3 imuShiftCur;

  Vector3 imuShiftFromStartCur;
  Vector3 imuVeloFromStartCur;

  Vector3 imuAngularRotationCur;
  Vector3 imuAngularRotationLast;
  Vector3 imuAngularFromStart;

  double imuTime[imuQueLength];
  float imuRoll[imuQueLength];
  float imuPitch[imuQueLength];
  float imuYaw[imuQueLength];

  Vector3 imuAcc[imuQueLength];
  Vector3 imuVelo[imuQueLength];
  Vector3 imuShift[imuQueLength];
  Vector3 imuAngularVelo[imuQueLength];
  Vector3 imuAngularRotation[imuQueLength];

  ros::Publisher _pub_cloud_corner_last;
  ros::Publisher _pub_cloud_surf_last;
  ros::Publisher pubLaserOdometry;
  ros::Publisher _pub_outlier_cloudLast;

  int skipFrameNum;
  bool systemInitedLM;

  int laserCloudCornerLastNum;
  int laserCloudSurfLastNum;

  std::vector<int> pointSelCornerInd;
  std::vector<float> pointSearchCornerInd1;
  std::vector<float> pointSearchCornerInd2;

  std::vector<int> pointSelSurfInd;
  std::vector<float> pointSearchSurfInd1;
  std::vector<float> pointSearchSurfInd2;
  std::vector<float> pointSearchSurfInd3;

  float transformCur[6];
  float transformSum[6];

  float imuRollLast, imuPitchLast, imuYawLast;
  Vector3 imuShiftFromStart;
  Vector3 imuVeloFromStart;

  pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
  pcl::PointCloud<PointType>::Ptr laserCloudOri;
  pcl::PointCloud<PointType>::Ptr coeffSel;

  nanoflann::KdTreeFLANN<PointType> kdtreeCornerLast;
  nanoflann::KdTreeFLANN<PointType> kdtreeSurfLast;

  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;

  nav_msgs::Odometry laserOdometry;

  tf::TransformBroadcaster tfBroadcaster;
  tf::StampedTransform laserOdometryTrans;

  bool isDegenerate;

  int frameCount;
  size_t _cycle_count;

 private:
  void initializationValue();
  void updateImuRollPitchYawStartSinCos();
  void ShiftToStartIMU(float pointTime);
  void VeloToStartIMU();
  void TransformToStartIMU(PointType *p);
  void AccumulateIMUShiftAndRotation();
  void adjustDistortion();
  void calculateSmoothness();
  void markOccludedPoints();
  void extractFeatures();

  void TransformToStart(PointType const *const pi, PointType *const po);
  void TransformToEnd(PointType const *const pi, PointType *const po);

  void PluginIMURotation(float bcx, float bcy, float bcz, float blx, float bly,
                         float blz, float alx, float aly, float alz, float &acx,
                         float &acy, float &acz);
  void AccumulateRotation(float cx, float cy, float cz, float lx, float ly,
                          float lz, float &ox, float &oy, float &oz);

  void findCorrespondingCornerFeatures(int iterCount);
  void findCorrespondingSurfFeatures(int iterCount);

  bool calculateTransformationSurf(int iterCount);
  bool calculateTransformationCorner(int iterCount);
  bool calculateTransformation(int iterCount);

  void checkSystemInitialization();
  void updateInitialGuess();
  void updateTransformation();

  void integrateTransformation();
  void publishCloud();
  void publishOdometry();

  void adjustOutlierCloud();
  void publishCloudsLast();

};

#endif // FEATUREASSOCIATION_H
