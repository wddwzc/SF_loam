#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#define WITH_LABEL

#define SCALED(x) x<180?x:(360-x)

std::string input_dir = "/home/wzc/Project/ICICIP/dataOutput3/";
std::vector<std::string> sequences = {"00", "01", "02", "03", "04", "05",
                                        "06", "07", "08", "09", "10"};

std::string groundTruth_loam = "/GroundTruth_raw.txt";
std::string laserOdometry_loam = "/LaserOdometry_raw.txt";
std::string mappingIter_loam = "/MappingIter_raw.txt";
std::string odometryIter_loam = "/OdometryIter_raw.txt";

std::string groundTruth_label = "/GroundTruth_label.txt";
std::string laserOdometry_label = "/LaserOdometry_label.txt";
std::string mappingIter_label = "/MappingIter_label.txt";
std::string odometryIter_label = "/OdometryIter_label.txt";

int main (int argc, char** argv)
{
    ros::init(argc, argv, "data2value");
    ros::NodeHandle n;

    if ((argc - 1)) {
        sequences.clear();
        std::string sequence = argv[1];
        sequences.push_back(sequence);
    }

    for (auto sque : sequences) {
        std::cout << "### Read the data file:" << input_dir + sque << std::endl;
        std::ifstream ipf_groundTruthLoam((input_dir + sque + groundTruth_loam).c_str());
        std::ifstream ipf_laserOdometryLoam((input_dir + sque + laserOdometry_loam).c_str());
        std::ifstream ipf_mappingIterLoam((input_dir + sque + mappingIter_loam).c_str());
        std::ifstream ipf_odometryIterLoam((input_dir + sque + odometryIter_loam).c_str());

#ifdef WITH_LABEL
        std::ifstream ipf_groundTruthLabel((input_dir + sque + groundTruth_label).c_str());
        std::ifstream ipf_laserOdometryLabel((input_dir + sque + laserOdometry_label).c_str());
        std::ifstream ipf_mappingIterLabel((input_dir + sque + mappingIter_label).c_str());
        std::ifstream ipf_odometryIterLabel((input_dir + sque + odometryIter_label).c_str());
#endif //WITH_LABEL

        std::ofstream opf_statistic((input_dir + sque + "/statistic.txt"));

        // Read position
        double r11, r12, r13, r21, r22, r23, r31, r32, r33, t1, t2, t3;
        std::vector<tf::Vector3> TGT_loam, TLO_loam;
        std::vector<tf::Vector3> EulerGT_loam, EulerLO_loam;
        tf::Vector3 T_temp, Euler_temp;
        double roll_temp, pitch_temp, yaw_temp;

        while (!ipf_groundTruthLoam.eof()) {
            ipf_groundTruthLoam >> r11 >> r12 >> r13 >> t1
                                >> r21 >> r22 >> r23 >> t2
                                >> r31 >> r32 >> r33 >> t3;

            T_temp.setValue(t1, t2, t3);
            tf::Matrix3x3 R(r11, r12, r13, r21, r22, r23, r31, r32, r33);
            R.getRPY(roll_temp, pitch_temp, yaw_temp);
            Euler_temp.setValue(roll_temp, pitch_temp, yaw_temp);

            TGT_loam.push_back(T_temp);
            EulerGT_loam.push_back(Euler_temp);
        }

        std::cout << "GroundTruth_raw size:" << TGT_loam.size() << std::endl;

        while (!ipf_laserOdometryLoam.eof()) {
            ipf_laserOdometryLoam >> r11 >> r12 >> r13 >> t1
                                  >> r21 >> r22 >> r23 >> t2
                                  >> r31 >> r32 >> r33 >> t3;
            T_temp.setValue(t1, t2, t3);
            tf::Matrix3x3 R(r11, r12, r13, r21, r22, r23, r31, r32, r33);
            R.getRPY(roll_temp, pitch_temp, yaw_temp);
            Euler_temp.setValue(roll_temp, pitch_temp, yaw_temp);

            TLO_loam.push_back(T_temp);
            EulerLO_loam.push_back(Euler_temp);
        }

        std::cout << "LaserOdometry_raw size:" << TLO_loam.size() << std::endl;

#ifdef WITH_LABEL
        std::vector<tf::Vector3> TGT_label, TLO_label;
        std::vector<tf::Vector3> EulerGT_label, EulerLO_label;
        while (!ipf_groundTruthLabel.eof()) {
            ipf_groundTruthLabel >> r11 >> r12 >> r13 >> t1
                                 >> r21 >> r22 >> r23 >> t2
                                 >> r31 >> r32 >> r33 >> t3;

            T_temp.setValue(t1, t2, t3);
            tf::Matrix3x3 R(r11, r12, r13, r21, r22, r23, r31, r32, r33);
            R.getRPY(roll_temp, pitch_temp, yaw_temp);
            Euler_temp.setValue(roll_temp, pitch_temp, yaw_temp);

            TGT_label.push_back(T_temp);
            EulerGT_label.push_back(Euler_temp);
        }

        std::cout << "GroundTruth_label size:" << TGT_label.size() << std::endl;

        while (!ipf_laserOdometryLabel.eof()) {
            ipf_laserOdometryLabel >> r11 >> r12 >> r13 >> t1
                                   >> r21 >> r22 >> r23 >> t2
                                   >> r31 >> r32 >> r33 >> t3;
            T_temp.setValue(t1, t2, t3);
            tf::Matrix3x3 R(r11, r12, r13, r21, r22, r23, r31, r32, r33);
            R.getRPY(roll_temp, pitch_temp, yaw_temp);
            Euler_temp.setValue(roll_temp, pitch_temp, yaw_temp);

            TLO_label.push_back(T_temp);
            EulerLO_label.push_back(Euler_temp);
        }

        std::cout << "LaserOdometry_label size:" << TLO_label.size() << std::endl;

#endif //WITH_LABEL

        // Read iteration data
        std::vector<double> odometryIter_loam, mappingIter_loam, odometryTime_loam, mappingTime_loam;
        int times_temp;
        double time_temp;

        while (!ipf_odometryIterLoam.eof()) {
            ipf_odometryIterLoam >> times_temp >> time_temp;
            odometryIter_loam.push_back(double(times_temp));
            odometryTime_loam.push_back(time_temp);
        }

        std::cout << "OdometryIter_raw size:" << odometryIter_loam.size() << std::endl;

        while (!ipf_mappingIterLoam.eof()) {
            ipf_mappingIterLoam >> times_temp >> time_temp;
            mappingIter_loam.push_back(double(times_temp));
            mappingTime_loam.push_back(time_temp);
        }

        std::cout << "MappingIter_raw size:" << mappingIter_loam.size() << std::endl;

#ifdef WITH_LABEL
        std::vector<double> odometryIter_label, mappingIter_label, odometryTime_label, mappingTime_label;
        while (!ipf_odometryIterLabel.eof()) {
            ipf_odometryIterLabel >> times_temp >> time_temp;
            odometryIter_label.push_back(double(times_temp));
            odometryTime_label.push_back(time_temp);
        }

        std::cout << "OdometryIter_label size:" << odometryIter_label.size() << std::endl;

        while (!ipf_mappingIterLabel.eof()) {
            ipf_mappingIterLabel >> times_temp >> time_temp;
            mappingIter_label.push_back(double(times_temp));
            mappingTime_label.push_back(time_temp);
        }

        std::cout << "MappingIter_label size:" << mappingIter_label.size() << std::endl;
#endif //WITH_LABEL

        // Caculate the error of yaw pitch roll and x y z
        std::cout << sque << ": The LOAM error of Euler and Pose:(roll pitch yaw Rot, x y z t)" << std::endl;
        opf_statistic << sque << ": The LOAM error of Euler and Pose:(roll pitch yaw Rot, x y z t)" << std::endl;
        double error_x, error_y, error_z, error_T, error_roll, error_pitch, error_yaw, error_R;
        error_x = error_y = error_z = error_T = error_roll = error_pitch = error_yaw = error_R = 0.0;
        int i = 0;
        for (i = 0; i < TGT_loam.size() - 1; ++i) {
            error_x += fabs(TGT_loam[i].x() - TLO_loam[i].x());
            error_y += fabs(TGT_loam[i].y() - TLO_loam[i].y());
            error_z += fabs(TGT_loam[i].z() - TLO_loam[i].z());
            error_roll += SCALED(fabs(EulerGT_loam[i].x() - EulerLO_loam[i].x()));
            error_pitch += SCALED(fabs(EulerGT_loam[i].y() - EulerLO_loam[i].y()));
            error_yaw += SCALED(fabs(EulerGT_loam[i].z() - EulerLO_loam[i].z()));
        }
        error_x /= i;
        error_y /= i;
        error_z /= i;
        error_roll /= i;
        error_pitch /= i;
        error_yaw /= i;
        error_T = sqrt(error_x * error_x + error_y * error_y + error_z * error_z);
        error_R = sqrt(error_roll * error_roll + error_pitch * error_pitch + error_yaw * error_yaw);

        std::cout << "Mean: " << std::endl;
        std::cout << error_roll << " " << error_pitch << " " << error_yaw << " " << error_R << std::endl;
        std::cout << error_x << " " << error_y << " " << error_z << " " << error_T << std::endl;
        opf_statistic << "Mean: " << std::endl;
        opf_statistic << error_roll << " " << error_pitch << " " << error_yaw << " " << error_R << std::endl;
        opf_statistic << error_x << " " << error_y << " " << error_z << " " << error_T << std::endl;


        i = TGT_loam.size() - 1;
        error_x = fabs(TGT_loam[i].x() - TLO_loam[i].x());
        error_y = fabs(TGT_loam[i].y() - TLO_loam[i].y());
        error_z = fabs(TGT_loam[i].z() - TLO_loam[i].z());
        error_roll = SCALED(fabs(EulerGT_loam[i].x() - EulerLO_loam[i].x()));
        error_pitch = SCALED(fabs(EulerGT_loam[i].y() - EulerLO_loam[i].y()));
        error_yaw = SCALED(fabs(EulerGT_loam[i].z() - EulerLO_loam[i].z()));
        error_T = sqrt(error_x * error_x + error_y * error_y + error_z * error_z);
        error_R = sqrt(error_roll * error_roll + error_pitch * error_pitch + error_yaw * error_yaw);

        std::cout << "Last: " << std::endl;
        std::cout << error_roll << " " << error_pitch << " " << error_yaw << " " << error_R << std::endl;
        std::cout << error_x << " " << error_y << " " << error_z << " " << error_T << std::endl;
        opf_statistic << "Last: " << std::endl;
        opf_statistic << error_roll << " " << error_pitch << " " << error_yaw << " " << error_R << std::endl;
        opf_statistic << error_x << " " << error_y << " " << error_z << " " << error_T << std::endl;

        error_x = error_y = error_z = error_T = error_roll = error_pitch = error_yaw = error_R = 0.0;

        // Caculate the iteration mean times and time
        std::cout << sque << ": The LOAM means of times and time" << std::endl;
        opf_statistic << sque << ": The LOAM means of times and time" << std::endl;
        double meanTimes = 0, meanTime = 0;

        for (i = 0; i < odometryIter_loam.size() - 1; ++i) {
            meanTimes += odometryIter_loam[i];
            meanTime += odometryTime_loam[i];
        }
        meanTimes /= i;
        meanTime /= i;
        std::cout << "Odometry_raw: " << meanTimes << " " << meanTime << std::endl;
        opf_statistic << "Odometry_raw: " << meanTimes << " " << meanTime << std::endl;
        meanTimes = meanTime = 0.0;

        for (i = 0; i < mappingIter_loam.size(); ++i) {
            meanTimes += mappingIter_loam[i];
            meanTime += mappingTime_loam[i];
        }
        meanTimes /= i;
        meanTime /= i;
        std::cout << "Mapping_raw: " << meanTimes << " " << meanTime << std::endl;
        opf_statistic << "Mapping_raw: " << meanTimes << " " << meanTime << std::endl;
        meanTimes = meanTime = 0.0;

        ipf_groundTruthLoam.close();
        ipf_laserOdometryLoam.close();
        ipf_mappingIterLoam.close();
        ipf_odometryIterLoam.close();

#ifdef WITH_LABEL
        // Caculate the error of yaw pitch roll and x y z
        std::cout << sque << ": The LOAM_label error of Euler and Pose:(roll pitch yaw Rot, x y z t)" << std::endl;
        opf_statistic << sque << ": The LOAM_label error of Euler and Pose:(roll pitch yaw Rot, x y z t)" << std::endl;

        error_x = error_y = error_z = error_T = error_roll = error_pitch = error_yaw = error_R = 0.0;
        for (i = 0; i < TGT_label.size() - 1; ++i) {
            error_x += fabs(TGT_label[i].x() - TLO_label[i].x());
            error_y += fabs(TGT_label[i].y() - TLO_label[i].y());
            error_z += fabs(TGT_label[i].z() - TLO_label[i].z());
            error_roll += SCALED(fabs(EulerGT_label[i].x() - EulerLO_label[i].x()));
            error_pitch += SCALED(fabs(EulerGT_label[i].y() - EulerLO_label[i].y()));
            error_yaw += SCALED(fabs(EulerGT_label[i].z() - EulerLO_label[i].z()));
        }

        error_x /= i;
        error_y /= i;
        error_z /= i;
        error_roll /= i;
        error_pitch /= i;
        error_yaw /= i;
        error_T = sqrt(error_x * error_x + error_y * error_y + error_z * error_z);
        error_R = sqrt(error_roll * error_roll + error_pitch * error_pitch + error_yaw * error_yaw);

        std::cout << "Mean: " << std::endl;
        std::cout << error_roll << " " << error_pitch << " " << error_yaw << " " << error_R << std::endl;
        std::cout << error_x << " " << error_y << " " << error_z << " " << error_T << std::endl;
        opf_statistic << "Mean: " << std::endl;
        opf_statistic << error_roll << " " << error_pitch << " " << error_yaw << " " << error_R << std::endl;
        opf_statistic << error_x << " " << error_y << " " << error_z << " " << error_T << std::endl;

        i = TGT_label.size() - 1;
        error_x = fabs(TGT_label[i].x() - TLO_label[i].x());
        error_y = fabs(TGT_label[i].y() - TLO_label[i].y());
        error_z = fabs(TGT_label[i].z() - TLO_label[i].z());
        error_roll = SCALED(fabs(EulerGT_label[i].x() - EulerLO_label[i].x()));
        error_pitch = SCALED(fabs(EulerGT_label[i].y() - EulerLO_label[i].y()));
        error_yaw = SCALED(fabs(EulerGT_label[i].z() - EulerLO_label[i].z()));
        error_T = sqrt(error_x * error_x + error_y * error_y + error_z * error_z);
        error_R = sqrt(error_roll * error_roll + error_pitch * error_pitch + error_yaw * error_yaw);

        std::cout << "Last: " << std::endl;
        std::cout << error_roll << " " << error_pitch << " " << error_yaw << " " << error_R << std::endl;
        std::cout << error_x << " " << error_y << " " << error_z << " " << error_T << std::endl;
        opf_statistic << "Last: " << std::endl;
        opf_statistic << error_roll << " " << error_pitch << " " << error_yaw << " " << error_R << std::endl;
        opf_statistic << error_x << " " << error_y << " " << error_z << " " << error_T << std::endl;

        error_x = error_y = error_z = error_T = error_roll = error_pitch = error_yaw = error_R = 0.0;

        // Caculate the iteration mean times and time
        std::cout << sque << ": The LOAM_label means of times and time" << std::endl;
        opf_statistic << sque << ": The LOAM_label means of times and time" << std::endl;

        for (i = 0; i < odometryIter_label.size() - 1; ++i) {
            meanTimes += odometryIter_label[i];
            meanTime += odometryTime_label[i];
        }
        meanTimes /= i;
        meanTime /= i;
        std::cout << "Odometry_label: " << meanTimes << " " << meanTime << std::endl;
        opf_statistic << "Odometry_label: " << meanTimes << " " << meanTime << std::endl;
        meanTimes = meanTime = 0.0;

        for (i = 0; i < mappingIter_label.size(); ++i) {
            meanTimes += mappingIter_label[i];
            meanTime += mappingTime_label[i];
        }
        meanTimes /= i;
        meanTime /= i;
        std::cout << "Mapping_label: " << meanTimes << " " << meanTime << std::endl;
        opf_statistic << "Mapping_label: " << meanTimes << " " << meanTime << std::endl;
        meanTimes = meanTime = 0.0;

        ipf_groundTruthLabel.close();
        ipf_laserOdometryLabel.close();
        ipf_mappingIterLabel.close();
        ipf_odometryIterLabel.close();
#endif //WITH_LABEL

        opf_statistic.close();
    }
    return 0;
}
