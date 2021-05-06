#include "utility.h"
#include "parameter.h"

class KittiProcess{

private:
    ros::NodeHandle nh;

    ros::Subscriber subKittiCameraOdom;     // 在相机坐标系下
    ros::Subscriber subKittiLaserOdom;      // 在激光坐标系下的odom
    ros::Subscriber subKittiPointCloud;		// 
    ros::Subscriber subKittiPointLabel;
	ros::Subscriber subKittiPointAll;		// 

    ros::Publisher pubLaserOdomGT;
	ros::Publisher pubKittiPointCloud;
	ros::Publisher pubKittiPointLabel;
	ros::Publisher pubKittiPointAll;

	pcl::PointCloud<pcl::PointXYZI>::Ptr kittiPointCloudIn;
	pcl::PointCloud<pcl::PointXYZL>::Ptr kittiPointLabelIn;
	pcl::PointCloud<PointType>::Ptr kittiPointAllIn;
	pcl::PointCloud<pcl::PointXYZI>::Ptr kittiPointCloudOut;
	pcl::PointCloud<pcl::PointXYZL>::Ptr kittiPointLabelOut;
	pcl::PointCloud<PointType>::Ptr kittiPointAllOut;

	nav_msgs::Odometry kittiLaserOdometry2;

	bool newKittiPointCloud;
	bool newKittiPointLabel;
	bool newKittiPointAll;

	double timeNewKittiPointCloud;
	double timeNewKittiPointLabel;
	double timeNewKittiPointAll;

    std_msgs::Header currentHeader;

	ParamServer param;

public:

    KittiProcess() {
        
        nh.getParam("/kittiProcess/fusion", param.fusion);
        
		if (param.fusion) {
			subKittiPointCloud = nh.subscribe<sensor_msgs::PointCloud2>("/kitti/velo/pointxyzi", 1, &KittiProcess::kittiPointCloudHandler, this);
			subKittiPointLabel = nh.subscribe<sensor_msgs::PointCloud2>("/kitti/velo/pointxyzl", 1, &KittiProcess::kittiPointLabelHandler, this);
			pubKittiPointAll = nh.advertise<sensor_msgs::PointCloud2>("/kitti/velo/pointall", 2);
		}
		else {
			subKittiPointAll = nh.subscribe<sensor_msgs::PointCloud2>("/kitti/velo/pointall", 1, &KittiProcess::kittiPointAllHandler, this);
			pubKittiPointCloud = nh.advertise<sensor_msgs::PointCloud2>("/kitti/velo/pointxyzi", 2);
			pubKittiPointLabel = nh.advertise<sensor_msgs::PointCloud2>("/kitti/velo/pointxyzl", 2);
		}

        // 订阅包中的原始数据
        subKittiCameraOdom = nh.subscribe<nav_msgs::Odometry>("/kitti/camera_left_poses", 5, &KittiProcess::kittiCameraOdometryHandler, this);
        subKittiLaserOdom = nh.subscribe<nav_msgs::Odometry>("/kitti/velodyne_poses", 5, &KittiProcess::kittiLaserOdometryHandler, this);
		
        // 发布 ground truth
        pubLaserOdomGT = nh.advertise<nav_msgs::Odometry>("/kitti/laser_GT", 5);
		
		kittiPointCloudIn.reset(new pcl::PointCloud<pcl::PointXYZI>());
		kittiPointLabelIn.reset(new pcl::PointCloud<pcl::PointXYZL>());
		kittiPointAllIn.reset(new pcl::PointCloud<PointType>());
		kittiPointCloudOut.reset(new pcl::PointCloud<pcl::PointXYZI>());
		kittiPointLabelOut.reset(new pcl::PointCloud<pcl::PointXYZL>());
		kittiPointAllOut.reset(new pcl::PointCloud<PointType>());

        kittiLaserOdometry2.header.frame_id = "/velodyne_init";
        kittiLaserOdometry2.child_frame_id = "/velodyne";

		newKittiPointCloud = false;
		newKittiPointLabel = false;
		newKittiPointAll = false;
    }

    void kittiCameraOdometryHandler(const nav_msgs::Odometry::ConstPtr& kittiCameraOdometryMsg) {
        currentHeader = kittiCameraOdometryMsg->header;

        double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat = kittiCameraOdometryMsg->pose.pose.orientation;
        tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

        // transformSum[0] = -pitch;
        // transformSum[1] = -yaw;
        // transformSum[2] = roll;

        // transformSum[3] = laserOdometry->pose.pose.position.x;
        // transformSum[4] = laserOdometry->pose.pose.position.y;
        // transformSum[5] = laserOdometry->pose.pose.position.z;

        // transformAssociateToMap();

        // geoQuat = tf::createQuaternionMsgFromRollPitchYaw
        //           (transformMapped[2], -transformMapped[0], -transformMapped[1]);

        // laserOdometry2.header.stamp = laserOdometry->header.stamp;
        // laserOdometry2.pose.pose.orientation.x = -geoQuat.y;
        // laserOdometry2.pose.pose.orientation.y = -geoQuat.z;
        // laserOdometry2.pose.pose.orientation.z = geoQuat.x;
        // laserOdometry2.pose.pose.orientation.w = geoQuat.w;
        // laserOdometry2.pose.pose.position.x = transformMapped[3];
        // laserOdometry2.pose.pose.position.y = transformMapped[4];
        // laserOdometry2.pose.pose.position.z = transformMapped[5];
        // pubLaserOdometry2.publish(laserOdometry2);

        // laserOdometryTrans2.stamp_ = laserOdometry->header.stamp;
        // laserOdometryTrans2.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
        // laserOdometryTrans2.setOrigin(tf::Vector3(transformMapped[3], transformMapped[4], transformMapped[5]));
        // tfBroadcaster2.sendTransform(laserOdometryTrans2);
    }

    void kittiLaserOdometryHandler(const nav_msgs::Odometry::ConstPtr& kittiLaserOdometryMsg) {
        double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat = kittiLaserOdometryMsg->pose.pose.orientation;
        tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);
        // cout << "# # # # #" << endl;
        // cout << "roll: " << roll << "  pitch: " << pitch << "  yaw: " << yaw << endl;

        kittiLaserOdometry2.header.stamp = kittiLaserOdometryMsg->header.stamp;
        kittiLaserOdometry2.pose.pose.orientation.x = geoQuat.x;
        kittiLaserOdometry2.pose.pose.orientation.y = geoQuat.y;
        kittiLaserOdometry2.pose.pose.orientation.z = geoQuat.z;
        kittiLaserOdometry2.pose.pose.orientation.w = geoQuat.w;
        kittiLaserOdometry2.pose.pose.position.x = kittiLaserOdometryMsg->pose.pose.position.x;
        kittiLaserOdometry2.pose.pose.position.y = kittiLaserOdometryMsg->pose.pose.position.y;
        kittiLaserOdometry2.pose.pose.position.z = kittiLaserOdometryMsg->pose.pose.position.z;
        pubLaserOdomGT.publish(kittiLaserOdometry2);
    }

	void kittiPointCloudHandler(const sensor_msgs::PointCloud2ConstPtr& kittiPointCloudMsg) {
        currentHeader = kittiPointCloudMsg->header;
		timeNewKittiPointCloud = currentHeader.stamp.toSec();

		kittiPointCloudIn->clear();
        pcl::fromROSMsg(*kittiPointCloudMsg, *kittiPointCloudIn);

        newKittiPointCloud = true;
        if (param.fusion)
		    mergePointAndLabel();
	}

	void kittiPointLabelHandler(const sensor_msgs::PointCloud2ConstPtr& kittiPointLabelMsg) {
		currentHeader = kittiPointLabelMsg->header;
		timeNewKittiPointLabel = currentHeader.stamp.toSec();

		kittiPointLabelIn->clear();
		pcl::fromROSMsg(*kittiPointLabelMsg, *kittiPointLabelIn);

		newKittiPointLabel = true;

        if (param.fusion)
		    mergePointAndLabel();
	}

	void kittiPointAllHandler(const sensor_msgs::PointCloud2ConstPtr& kittiPointAllMsg) {
		currentHeader = kittiPointAllMsg->header;
		timeNewKittiPointAll = currentHeader.stamp.toSec();

		kittiPointAllIn->clear();
		pcl::fromROSMsg(*kittiPointAllMsg, *kittiPointAllIn);

		newKittiPointAll = true;

		if (!param.fusion) {
			splitPointAndLabel();
		}
	}

	void mergePointAndLabel() {
		if (newKittiPointCloud && newKittiPointLabel) {
			if (fabs(timeNewKittiPointLabel - timeNewKittiPointCloud) < 1e-4) {
				uint sz = kittiPointLabelIn->points.size();
				kittiPointAllOut->clear();
				kittiPointAllOut->points.resize(sz);
				
				for (size_t i = 0; i < sz; ++i) {
					pcl::PointXYZI &pCloud = kittiPointCloudIn->points[i];
					pcl::PointXYZL &pLabel = kittiPointLabelIn->points[i];
					PointType &pOut = kittiPointAllOut->points[i];
					pOut.x = pCloud.x;
					pOut.y = pCloud.y;
					pOut.z = pCloud.z;
					pOut.intensity = pCloud.intensity;
					pOut.label = pLabel.label;
				}

				sensor_msgs::PointCloud2 laserCloudTemp;
				if (pubKittiPointAll.getNumSubscribers() != 0){
					pcl::toROSMsg(*kittiPointAllOut, laserCloudTemp);
					laserCloudTemp.header.stamp = currentHeader.stamp;
					laserCloudTemp.header.frame_id = "velodyne";
					pubKittiPointAll.publish(laserCloudTemp);
				}
                kittiPointAllOut->clear();
			}
			newKittiPointCloud = false;
			newKittiPointLabel = false;
		}
		return;
	}

	void splitPointAndLabel() {
		uint sz = kittiPointAllIn->points.size();
		kittiPointCloudOut->clear();
		kittiPointLabelOut->clear();
		kittiPointCloudOut->points.resize(sz);
		kittiPointLabelOut->points.resize(sz);

		for (size_t i = 0; i < sz; ++i) {
			pcl::PointXYZI &pCloud = kittiPointCloudOut->points[i];
			pcl::PointXYZL &pLabel = kittiPointLabelOut->points[i];
			PointType &pAll = kittiPointAllIn->points[i];

			pCloud.x = pAll.x;
			pCloud.y = pAll.y;
			pCloud.z = pAll.z;
			pCloud.intensity = pAll.intensity;

			pLabel.x = pAll.x;
			pLabel.y = pAll.y;
			pLabel.z = pAll.z;
			pLabel.label = pAll.label;
		}

		sensor_msgs::PointCloud2 laserCloudTemp;
		if (pubKittiPointCloud.getNumSubscribers() != 0) {
			pcl::toROSMsg(*kittiPointCloudOut, laserCloudTemp);
			laserCloudTemp.header.stamp = currentHeader.stamp;
			laserCloudTemp.header.frame_id = "velodyne";
			pubKittiPointCloud.publish(laserCloudTemp);
		}
		if (pubKittiPointLabel.getNumSubscribers() != 0) {
			pcl::toROSMsg(*kittiPointLabelOut, laserCloudTemp);
			laserCloudTemp.header.stamp = currentHeader.stamp;
			laserCloudTemp.header.frame_id = "velodyne";
			pubKittiPointLabel.publish(laserCloudTemp);
		}
	}
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "kitti");
    
    KittiProcess kittiproc;

    ROS_INFO("Kitti Preprocess Started.");

    ros::spin();

    return 0;
}
