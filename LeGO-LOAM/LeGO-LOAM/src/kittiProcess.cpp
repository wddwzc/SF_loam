#include "utility.h"

class KittiProcess{

private:
    ros::NodeHandle nh;

    ros::Subscriber subKittiPointCloud;
    ros::Subscriber subKittiPointLabel;
	ros::Publisher pubKittiPointAll;

	pcl::PointCloud<pcl::PointXYZI>::Ptr kittiPointCloudIn;
	pcl::PointCloud<pcl::PointXYZL>::Ptr kittiPointLabelIn;
	pcl::PointCloud<PointType>::Ptr kittiPointAllOut;

	bool newKittiPointCloud;
	bool newKittiPointLabel;

	double timeNewKittiPointCloud;
	double timeNewKittiPointLabel;

    std_msgs::Header currentHeader;

public:

    KittiProcess() {
        
		subKittiPointCloud = nh.subscribe<sensor_msgs::PointCloud2>("/kitti/velo/pointxyzi", 1, &KittiProcess::kittiPointCloudHandler, this);
		subKittiPointLabel = nh.subscribe<sensor_msgs::PointCloud2>("/kitti/velo/pointxyzl", 1, &KittiProcess::kittiPointLabelHandler, this);
		pubKittiPointAll = nh.advertise<sensor_msgs::PointCloud2>("/kitti/velo/pointall", 2);
		
		kittiPointCloudIn.reset(new pcl::PointCloud<pcl::PointXYZI>());
		kittiPointLabelIn.reset(new pcl::PointCloud<pcl::PointXYZL>());
		kittiPointAllOut.reset(new pcl::PointCloud<PointType>());

		newKittiPointCloud = false;
		newKittiPointLabel = false;
    }

	void kittiPointCloudHandler(const sensor_msgs::PointCloud2ConstPtr& kittiPointCloudMsg) {
        currentHeader = kittiPointCloudMsg->header;
		timeNewKittiPointCloud = currentHeader.stamp.toSec();

		kittiPointCloudIn->clear();
        pcl::fromROSMsg(*kittiPointCloudMsg, *kittiPointCloudIn);

        newKittiPointCloud = true;
		mergePointAndLabel();
	}

	void kittiPointLabelHandler(const sensor_msgs::PointCloud2ConstPtr& kittiPointLabelMsg) {
		currentHeader = kittiPointLabelMsg->header;
		timeNewKittiPointLabel = currentHeader.stamp.toSec();

		kittiPointLabelIn->clear();
		pcl::fromROSMsg(*kittiPointLabelMsg, *kittiPointLabelIn);

		newKittiPointLabel = true;
		mergePointAndLabel();
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
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "kitti");
    
    KittiProcess kittiproc;

    ROS_INFO("Kitti Preprocess Started.");

    ros::spin();

    return 0;
}
