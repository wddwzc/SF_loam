#include <string>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#define PI 3.14159265
#define QUE_MAX 10

// std::string output_dir = "/home/robotugv/ICICIP/dataOutput/08";
// std::string output_GroundTruth = "/GroundTruth_raw.txt";
// std::string output_LaserOdometry = "/LaserOdometry_raw.txt";

std::string output_dir = "/home/wzc/";
std::string output_GroundTruth = "/GroundTruth_label.txt";
std::string output_LaserOdometry = "/LaserOdometry_label.txt";

bool init = false;
tf::Transform init_tf;

std::ofstream lo_opf;
std::ofstream gt_opf;

tf::Matrix3x3 R_que[QUE_MAX];
tf::Vector3 T_que[QUE_MAX];
double que_time[QUE_MAX];
int que_front = 0, que_rear = 0;

tf::Quaternion tf_quat(0.499888324808,
                       -0.503701389036,
                       0.496055497539,
                       0.500325380382);
// tf::Vector3 tf_vec(-0.0119845992771,
//                   -0.0540398472975,
//                   -0.292196864869);
tf::Vector3 tf_vec(0.0, 0.0, 0.0);
tf::Transform left2laser(tf_quat, tf_vec);

void laserOdometry_subCallback(const nav_msgs::Odometry::ConstPtr& laserOdometry) {
    tf::Quaternion Quat;
    tf::quaternionMsgToTF(laserOdometry->pose.pose.orientation, Quat);
    tf::Vector3 Pose(laserOdometry->pose.pose.position.x,
                     laserOdometry->pose.pose.position.y,
                     laserOdometry->pose.pose.position.z);

    tf::Quaternion odom2laser_quat;
    odom2laser_quat.setEulerZYX(-PI/2, -PI/2, 0.0);
    tf::Vector3 odom2laser_vec(0.0, 0.0, 0.0);
    tf::Transform odom2laser(odom2laser_quat, odom2laser_vec);

    tf::Transform cur_T(Quat, Pose);
    // cur_T = odom2laser.inverse() * cur_T;
    tf::Matrix3x3 R = cur_T.getBasis();
    tf::Vector3 T = cur_T.getOrigin();

    double laserOdometry_time = laserOdometry->header.stamp.toSec();
    while (que_front != que_rear) {
        if (fabs(laserOdometry_time - que_time[que_front]) < 0.05) {
            if (!init) {
                init = true;
                init_tf.setBasis(R_que[que_front]);
                init_tf.setOrigin(T_que[que_front]);
            }
            R_que[que_front] = init_tf.getBasis().inverse() * R_que[que_front];
            T_que[que_front] = T_que[que_front] - init_tf.getOrigin();

            double roll0, pitch0, yaw0;
            double roll1, pitch1, yaw1;
            R_que[que_front].getRPY(roll0, pitch0, yaw0);
            R.getRPY(roll1, pitch1, yaw1);
            R.setRPY(-roll1, -pitch1, yaw1);

            std::cout << "-------------------------------" << laserOdometry_time << " " << que_time[que_front] << " " << que_front << std::endl;
            // std::cout << T[0] << " " << T[1] << " " << T[2] << std::endl;
            // std::cout << T_que[que_front][0] << " " << T_que[que_front][1] << " " << T_que[que_front][2] << std::endl;
            std::cout << roll0 << " " << -roll1 << std::endl;
            std::cout << pitch0 << " " << -pitch1 << std::endl;
            std::cout << yaw0 << " " << yaw1 << std::endl;

            lo_opf << R[0][0] << " " << R[0][1] << " " << R[0][2] << " " << -T[0] << " "
                   << R[1][0] << " " << R[1][1] << " " << R[1][2] << " " << -T[1] << " "
                   << R[2][0] << " " << R[2][1] << " " << R[2][2] << " " << T[2] << std::endl;
            gt_opf << R_que[que_front][0][0] << " " << R_que[que_front][0][1] << " " << R_que[que_front][0][2] << " " << T_que[que_front][0] << " "
                   << R_que[que_front][1][0] << " " << R_que[que_front][1][1] << " " << R_que[que_front][1][2] << " " << T_que[que_front][1] << " "
                   << R_que[que_front][2][0] << " " << R_que[que_front][2][1] << " " << R_que[que_front][2][2] << " " << T_que[que_front][2] << std::endl;

            // lo_opf << R[0][0] << " " << R[0][1] << " " << R[0][2] << " " << -T[1] << " "
            //        << R[1][0] << " " << R[1][1] << " " << R[1][2] << " " << -T[2] << " "
            //        << R[2][0] << " " << R[2][1] << " " << R[2][2] << " " << T[0] << std::endl;
            // gt_opf << R_que[que_front][0][0] << " " << R_que[que_front][0][1] << " " << R_que[que_front][0][2] << " " << -T_que[que_front][1] << " "
            //        << R_que[que_front][1][0] << " " << R_que[que_front][1][1] << " " << R_que[que_front][1][2] << " " << -T_que[que_front][2] << " "
            //        << R_que[que_front][2][0] << " " << R_que[que_front][2][1] << " " << R_que[que_front][2][2] << " " << T_que[que_front][0] << std::endl;

            // for (int i = 0; i < 3; ++i) {
            //     lo_opf << R[i][0] << " " << R[i][1] << " " << R[i][2] << " " << T[i] << " ";
            //     gt_opf << R_que[que_front][i][0] << " " << R_que[que_front][i][1] << " "
            //             << R_que[que_front][i][2] << " " << T_que[que_front][i] << " ";
            // }
            // lo_opf << std::endl;
            // gt_opf << std::endl;

            que_front = (que_front + 1) % QUE_MAX;

            break;
        }
        que_front = (que_front + 1) % QUE_MAX;
    }
    // std::cout << T[0] << " " << T[1] << " " << T[2] << std::endl;
}

void groundTruth_subCallback (const nav_msgs::Odometry::ConstPtr& groundTruth) {
    tf::Quaternion Quat;
    tf::quaternionMsgToTF(groundTruth->pose.pose.orientation, Quat);
    tf::Vector3 Pose(groundTruth->pose.pose.position.x,
                     groundTruth->pose.pose.position.y,
                     groundTruth->pose.pose.position.z);

    tf::Transform cur_T(Quat, Pose);
    // cur_T = left2laser.inverse() * cur_T;
    tf::Matrix3x3 R = cur_T.getBasis();
    tf::Vector3 T = cur_T.getOrigin();

    que_time[que_rear] = groundTruth->header.stamp.toSec();
    R_que[que_rear] = R;
    T_que[que_rear] = T;
    if ((que_rear + 1) % QUE_MAX == que_front) {
        que_front = (que_front + 1) % QUE_MAX;
    }
    que_rear = (que_rear + 1) % QUE_MAX;
    // std::cout << "front:" << que_front << " rear:" << que_rear << std::endl;
    // std::cout << T[0] << " " << T[1] << " " << T[2] << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry2kitti");
    ros::NodeHandle n;

    ros::Subscriber subLaserOdometry = n.subscribe<nav_msgs::Odometry>
                                        ("/integrated_to_init", 5, laserOdometry_subCallback);

    // ros::Subscriber subLaserOdometry = n.subscribe<nav_msgs::Odometry>
    //                                     ("/laser_odom_to_init", 5, laserOdometry_subCallback);

    ros::Subscriber subGroundTruth = n.subscribe<nav_msgs::Odometry>
                                        ("/kitti/camera_left_poses", 5, groundTruth_subCallback);

    lo_opf.open((output_dir + output_LaserOdometry).c_str());
    gt_opf.open((output_dir + output_GroundTruth).c_str());

    ros::spin();

    lo_opf.close();
    gt_opf.close();

    return 0;
}
