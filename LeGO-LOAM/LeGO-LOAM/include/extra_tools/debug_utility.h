#ifndef _DEBUG_UTILITY_H_
#define _DEBUG_UTILITY_H_

#include "utility.h"
#include "cluster_method/segment.h"
#include "kitti_label_color.h"

// record time
class TimeRecorder {
public:
    TimeRecorder() {
        begin = ros::Time::now();
        duration_ms = 0.0;
    }

    ros::Time recordStart() {
        begin = ros::Time::now();
        return begin;
    }
    
    double calculateDuration() {
        duration_ms = (ros::Time::now() - begin).toSec() * 1000;
        return duration_ms;
    }

    void printDuration(string task_name)
    {
        double duration_ms = (ros::Time::now() - begin).toSec() * 1000;

        // std::cout.precision(3); // 10 for sec, 3 for ms 
        std::cout << task_name << ": " << duration_ms << " ms." << std::endl;
    }

private:
    ros::Time begin;
    double duration_ms;
};

void visualizeCloudRGB(pcl::PointCloud<PointType>::Ptr laserCloud, ros::Publisher &pub,
                        ros::Time time, string frame_id) {
    if (pub.getNumSubscribers() != 0) {
        pcl::PointCloud<pcl::PointXYZRGB> laserCloudColor;
        for (auto &p : laserCloud->points) {
            pcl::PointXYZRGB cur_p;
            cur_p.x = p.x;
            cur_p.y = p.y;
            cur_p.z = p.z;
            int class_id = p.label;
            cur_p.r = colors_map_tran[class_id][0];
            cur_p.g = colors_map_tran[class_id][1];
            cur_p.b = colors_map_tran[class_id][2];
            laserCloudColor.push_back(cur_p);
        }
        sensor_msgs::PointCloud2 tempLaserCloud;
        pcl::toROSMsg(laserCloudColor, tempLaserCloud);
        tempLaserCloud.header.stamp = time;
        tempLaserCloud.header.frame_id = frame_id;
        pub.publish(tempLaserCloud);
    }
}

void visualizeCloud(pcl::PointCloud<PointType>::Ptr laserCloud, ros::Publisher &pub,
                    ros::Time time, string frame_id) {
    if (pub.getNumSubscribers() != 0) {
        sensor_msgs::PointCloud2 tempLaserCloud;
        pcl::toROSMsg(*laserCloud, tempLaserCloud);
        tempLaserCloud.header.stamp = time;
        tempLaserCloud.header.frame_id = frame_id;
        pub.publish(tempLaserCloud);
    }
}

void visualizeBox(vector<Segment> &segments, ros::Publisher &pub,
                    ros::Time time, string frame_id) {

    if (pub.getNumSubscribers() != 0) {
        visualization_msgs::MarkerArray boundingBoxArray;
        visualization_msgs::Marker boundingBox;
        boundingBox.header.frame_id = frame_id;
        boundingBox.type = visualization_msgs::Marker::CUBE;
        boundingBox.ns = "/local_map";
        boundingBox.action = visualization_msgs::Marker::ADD;
        

        unsigned int id = 1;
        for (auto &segment : segments) {
            Box &cur_box = segment.box;

            boundingBox.id = id++;
            boundingBox.pose.position.x = cur_box.bboxTransform[0]; 
            boundingBox.pose.position.y = cur_box.bboxTransform[1]; 
            boundingBox.pose.position.z = cur_box.bboxTransform[2]; 
            boundingBox.pose.orientation.x = cur_box.bboxQuaternion.x();
            boundingBox.pose.orientation.y = cur_box.bboxQuaternion.y();
            boundingBox.pose.orientation.z = cur_box.bboxQuaternion.z();
            boundingBox.pose.orientation.w = cur_box.bboxQuaternion.w();

            boundingBox.scale.x = cur_box.cube_width; 
            boundingBox.scale.y = cur_box.cube_length; 
            boundingBox.scale.z = cur_box.cube_height; 

            if (boundingBox.scale.x == 0)
                boundingBox.scale.x=0.1;

            if (boundingBox.scale.y == 0)
                boundingBox.scale.y=0.1;

            if (boundingBox.scale.z == 0)
                boundingBox.scale.z=0.1;

            boundingBox.color.r = colors_map_tran[segment.semantic][0];
            boundingBox.color.g = colors_map_tran[segment.semantic][1];
            boundingBox.color.b = colors_map_tran[segment.semantic][2];
            boundingBox.color.a = 0.5;

            boundingBox.lifetime = ros::Duration(0.5);
            boundingBoxArray.markers.push_back(boundingBox);
        }

        pub.publish(boundingBoxArray);
    }

}

void visualizeSegments(vector<Segment> &segments, ros::Publisher &pub,
                        ros::Time time, string frame_id) {
    if (pub.getNumSubscribers() != 0) {
        pcl::PointCloud<pcl::PointXYZRGB> laserCloudColor;
        for (auto &seg : segments) {
            int class_id = seg.semantic;
            for (auto &p : seg.segmentCloud) {
                pcl::PointXYZRGB cur_p;
                cur_p.x = seg.centroid.x;
                cur_p.y = seg.centroid.y;
                cur_p.z = seg.centroid.z;
                cur_p.r = colors_map_tran[class_id][0];
                cur_p.g = colors_map_tran[class_id][1];
                cur_p.b = colors_map_tran[class_id][2];
                laserCloudColor.push_back(cur_p);
            }
        }
        sensor_msgs::PointCloud2 tempLaserCloud;
        pcl::toROSMsg(laserCloudColor, tempLaserCloud);
        tempLaserCloud.header.stamp = time;
        tempLaserCloud.header.frame_id = frame_id;
        pub.publish(tempLaserCloud);
    }
}


#endif


        // for (auto &segment_to_add : cluster_indices) {
        //     unsigned int sz = segment_to_add.indices.size();

        //     if (param.debugOctomapCluster) cout << "cur_segment_size: " << sz << endl;

            

        //     pcl::PointXYZRGB centroid(0.0, 0.0, 0.0);
        //     vector<unsigned int> class_counts(26, 0);
        //     for (auto &index : segment_to_add.indices) {
        //         pcl::PointXYZL &cur_point = laserCloud_noGround->points[index];
        //         unsigned class_id = classes_map[cur_point.label];
        //         ++class_counts[class_id];
        //         centroid.x += cur_point.x;
        //         centroid.y += cur_point.y;
        //         centroid.z += cur_point.z;
        //     }
        //     centroid.x /= (float)sz;
        //     centroid.y /= (float)sz;
        //     centroid.z /= (float)sz;

        //     unsigned int segment_class = 0;
        //     for (int i = 0; i < 25; ++i) {
        //         if (class_counts[i] / (float)sz >= 0.6)
        //             segment_class = i;
        //     }
        //     centroid.r = colors_map_tran[segment_class][0];
        //     centroid.g = colors_map_tran[segment_class][1];
        //     centroid.b = colors_map_tran[segment_class][2];

        //     // add marker
        //     geometry_msgs::Point marker_p;
        //     marker_p.x = centroid.x;
        //     marker_p.y = centroid.y;
        //     marker_p.z = centroid.z;
        //     marker_Centroid2Seg.points.push_back(marker_p);
        //     marker_p.z += 10.0;
        //     marker_Centroid2Seg.points.push_back(marker_p);

        //     classifiedCentroidRGB->push_back(centroid);
        //     centroid.z += 10.0;
        //     classifiedCentroidRGB->push_back(centroid);

        //     for (auto &index : segment_to_add.indices) {
        //         pcl::PointXYZL cur_point = laserCloud_noGround->points[index];
        //         pcl::PointXYZRGB cur_p;
        //         cur_p.x = cur_point.x;
        //         cur_p.y = cur_point.y;
        //         cur_p.z = cur_point.z;
        //         cur_p.r = colors_map_tran[segment_class][0];
        //         cur_p.g = colors_map_tran[segment_class][1];
        //         cur_p.b = colors_map_tran[segment_class][2];
        //         classifiedCloud->push_back(cur_p);
        //     }
        // }