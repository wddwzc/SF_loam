#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Core>
#include <Eigen/Geometry> 

using namespace std;

ros::Publisher pubLaserOdomTrans;

string odometryTopic;
string odometryTopicTrans;
string odometryType;
double gnssAlignedAngle;
int odometryPubSkip;
bool outPut;
string outputFile;
string ouputTimeFile;


ofstream outf;
ofstream outft;

vector<Eigen::Matrix4d> odomety_data;
vector<double> times;

void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry)
{
    geometry_msgs::Point geoPose = laserOdometry->pose.pose.position;
    geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;

    Eigen::Quaterniond q(1.0, 0.0, 0.0, 0.0);
    Eigen::Vector3d t{0.0, 0.0, 0.0};
    //  loam  kitti_laser_pose
    if (odometryType == "loam") {
        q.x() = geoQuat.z;
        q.y() = geoQuat.x;
        q.z() = geoQuat.y;
        q.w() = geoQuat.w;
        t[0] = geoPose.z;
        t[1] = geoPose.x;
        t[2] = geoPose.y;
    }
    else if (odometryType == "ls_loam") {
        q.x() = -geoQuat.y;
        q.y() = geoQuat.x;
        q.z() = geoQuat.z;
        q.w() = geoQuat.w;
        t[0] = -geoPose.y;
        t[1] = geoPose.x;
        t[2] = geoPose.z;
    }
    else if (odometryType == "lego") {
        q.x() = -geoQuat.x;
        q.y() = geoQuat.z;
        q.z() = geoQuat.y;
        q.w() = geoQuat.w;
        t[0] = -geoPose.x;
        t[1] = geoPose.z;
        t[2] = geoPose.y;
    }
    else if (odometryType == "kitti_laser_pose") {
        q.x() = geoQuat.x;
        q.y() = geoQuat.y;
        q.z() = geoQuat.z;
        q.w() = geoQuat.w;
        t[0] = geoPose.x;
        t[1] = geoPose.y;
        t[2] = geoPose.z;
    }
    else if (odometryType == "gnss") {
        Eigen::AngleAxisd gnss_angle_axis(gnssAlignedAngle, Eigen::Vector3d(0, 0, 1));
        q.x() = -geoQuat.x;
        q.y() = geoQuat.z;
        q.z() = geoQuat.y;
        q.w() = geoQuat.w;
        Eigen::Matrix3d temp_R = q.toRotationMatrix();
        temp_R = gnss_angle_axis.matrix() * temp_R;
        q = temp_R;
        
        Eigen::AngleAxisd gnss_angle_pose(gnssAlignedAngle, Eigen::Vector3d(0, 0, 1));
        Eigen::Vector3d v_pose(-geoPose.x, geoPose.z, geoPose.y);
        Eigen::Vector3d rotated_v_pose = gnss_angle_pose.matrix() * v_pose;
        t[0] = rotated_v_pose[0];
        t[1] = rotated_v_pose[1];
        t[2] = rotated_v_pose[2];
    }
    else {
        q.x() = geoQuat.x;
        q.y() = geoQuat.y;
        q.z() = geoQuat.z;
        q.w() = geoQuat.w;
        t[0] = geoPose.x;
        t[1] = geoPose.y;
        t[2] = geoPose.z;
    }

    Eigen::Matrix4d T_matrix = Eigen::Matrix4d::Identity();
    T_matrix.block<3, 3>(0, 0) = q.matrix();
    T_matrix(0, 3) = t[0];
    T_matrix(1, 3) = t[1];
    T_matrix(2, 3) = t[2];
    if (outPut) {
        // outf << T_matrix(0, 0) << " " << T_matrix(0, 1) << " " << T_matrix(0, 2) << " " << T_matrix(0, 3) << " "
        //     << T_matrix(1, 0) << " " << T_matrix(1, 1) << " " << T_matrix(1, 2) << " " << T_matrix(1, 3) << " "
        //     << T_matrix(2, 0) << " " << T_matrix(2, 1) << " " << T_matrix(2, 2) << " " << T_matrix(2, 3) << endl;
        odomety_data.push_back(T_matrix);
        times.push_back(laserOdometry->header.stamp.toSec());
    }

    static int counts = 1;
    static int skip = odometryPubSkip;
    ++skip;
    cout << "\r" << counts++;
    cout.flush();

    if (pubLaserOdomTrans.getNumSubscribers() != 0 && skip >= odometryPubSkip){
        skip = 0;
        nav_msgs::Odometry laserOdometry2;
        laserOdometry2.header.stamp = laserOdometry->header.stamp;
        laserOdometry2.header.frame_id = "base_link";
        laserOdometry2.pose.pose.orientation.x = q.x();
        laserOdometry2.pose.pose.orientation.y = q.y();
        laserOdometry2.pose.pose.orientation.z = q.z();
        laserOdometry2.pose.pose.orientation.w = q.w();
        laserOdometry2.pose.pose.position.x = t[0];
        laserOdometry2.pose.pose.position.y = t[1];
        laserOdometry2.pose.pose.position.z = t[2];
        pubLaserOdomTrans.publish(laserOdometry2);
    }
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "pose2lidarframe");
    ros::NodeHandle n("~");

    ROS_INFO("\033[1;32m---->\033[0m Pose Transformer Started.");

    n.param<string>("odometry_topic", odometryTopic, "/aft_mapped_to_init");
    n.param<string>("odometry_topic_trans", odometryTopicTrans, "/laser_odometry_trans");
    n.param<string>("odometry_type", odometryType, "loam");
    n.param<double>("gnss_aligned_angle", gnssAlignedAngle, 0);
    n.param<int>("odometry_pub_skip", odometryPubSkip, 1);
    n.param<bool>("ouput", outPut, false);
    n.param<string>("output_file", outputFile, "/home/wzc/00_lidar.txt");
    n.param<string>("time_file", ouputTimeFile, "/home/wzc/00_times.txt");

    cout << "### Parameter list ###" << endl;
    cout << "odometry_topic: " << odometryTopic << endl;
    cout << "odometry_topic_trans: " << odometryTopicTrans << endl;
    cout << "odometry_type: " << odometryType << endl;
    cout << "gnss aligned angle: " << gnssAlignedAngle << endl;
    cout << "odometry_pub_skip: " << odometryPubSkip << endl;
    cout << "ouput: " << outPut << endl;
    cout << "output_file: " << outputFile << endl;
    cout << "output_time: " << ouputTimeFile << endl;

    ros::Subscriber subLaserOdom = n.subscribe<nav_msgs::Odometry>(odometryTopic, 10, laserOdometryHandler);
    pubLaserOdomTrans = n.advertise<nav_msgs::Odometry>(odometryTopicTrans, 5);

    odomety_data.reserve(30000);
    times.reserve(30000);

    if (outPut)
        outf.open(outputFile, ios::out);
        outft.open(ouputTimeFile, ios::out);

    cout << "read!!!" << endl;

    ros::spin();
    
    if (outPut) {
        cout << endl;
        cout << odomety_data.size() << " scan" << endl;
        double cur_time = 0.0;
        double origin_time = 0.0;
        int missCount = 0;
        if (!odomety_data.empty()) {
            cur_time = times[0];
            origin_time = times[0];
        }
        for (int i = 0; i < odomety_data.size(); ++i) {
            if (times[i] > cur_time + 0.15) {
                cout << "Missing " << i << " scan" << endl;
                missCount++;
            }
            cur_time = times[i];
            auto &T_M = odomety_data[i];
            outf << T_M(0, 0) << " " << T_M(0, 1) << " " << T_M(0, 2) << " " << T_M(0, 3) << " "
                << T_M(1, 0) << " " << T_M(1, 1) << " " << T_M(1, 2) << " " << T_M(1, 3) << " "
                << T_M(2, 0) << " " << T_M(2, 1) << " " << T_M(2, 2) << " " << T_M(2, 3) << endl;
            outft << times[i] - origin_time << endl;
        }
        cout << "Total missed: " << missCount << endl;
        outf.close();
        outft.close();
    }

    return 0;
}
