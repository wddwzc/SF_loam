#ifndef _CLUSTER_HPP_
#define _CLUSTER_HPP_

#include "utility.h"
#include "parameter.h"
#include "extra_tools/nanoflann_pcl.h"
#include "extra_tools/debug_utility.h"
#include "extra_tools/iou.hpp"
#include "cluster_method/segment.h"
#include "cluster_method/dbscan.h"
#include "cluster_method/CVC.hpp"

#include <Eigen/Geometry>

class Cluster {
public:
    Cluster() = default;

    void geometricSegmentation(ParamServer &param,
                                pcl::PointCloud<PointType>::Ptr laserCloudIn,
                                vector<Segment> &segments) {

        // Calculate time cost for segmentation
        TimeRecorder segmentationTime;
        segmentationTime.recordStart();

        if (laserCloudIn->points.size() == 0)  return;

        if (param.debugOctomapCluster)
            cout << "Point Size for Segment: " << laserCloudIn->points.size() << endl;

        // PointType is PointXYZIL
        pcl::search::KdTree<PointType>::Ptr treeSegmentPcl(new pcl::search::KdTree<PointType>);
        treeSegmentPcl->setInputCloud(laserCloudIn);
        boost::shared_ptr<nanoflann::KdTreeFLANN<PointType>> treeSegmentPtr(new nanoflann::KdTreeFLANN<PointType>());
        treeSegmentPtr->setInputCloud(laserCloudIn);
        std::vector<pcl::PointIndices> cluster_indices;

        if (param.clusterMethod == "NANO_DBSCAN") {  // DBSCAN
            DBSCANKdtreeCluster<PointType> dbscan;
            dbscan.setCorePointMinPts(param.coreMinPts);
            dbscan.setClusterTolerance(param.radiusGrowing);
            dbscan.setMinClusterSize(param.minClusterSize);
            dbscan.setMaxClusterSize(param.maxClusterSize);
            dbscan.setSearchMethodNano(treeSegmentPtr);
            dbscan.setInputCloud(laserCloudIn);
            dbscan.extract(cluster_indices);
        }
        else if (param.clusterMethod == "DBSCAN") {
            DBSCANKdtreeCluster<PointType> dbscan;
            dbscan.setCorePointMinPts(param.coreMinPts);
            dbscan.setClusterTolerance(param.radiusGrowing);
            dbscan.setMinClusterSize(param.minClusterSize);
            dbscan.setMaxClusterSize(param.maxClusterSize);
            dbscan.setSearchMethod(treeSegmentPcl);
            dbscan.setInputCloud(laserCloudIn);
            dbscan.extract(cluster_indices);
        }
        else if (param.clusterMethod == "CVC") {
            CVCCluster<PointType> cvc;
            cvc.setInputCloud(laserCloudIn);
            cvc.setMinClusterSize(param.minClusterSize);
            cvc.extract(cluster_indices);
        }
        else if (param.clusterMethod == "EUCLIDEAN") {   // euclidean clusters
            
            // pcl::extractEuclideanClusters<PointType>(*laserCloudIn, treeSegment, param.radiusGrowing, cluster_indices, 
            //                                                 param.minClusterSize, param.maxClusterSize);
            
            pcl::EuclideanClusterExtraction<PointType> ec; // clustering object
            ec.setClusterTolerance(param.radiusGrowing);
            ec.setMinClusterSize(param.minClusterSize);
            ec.setMaxClusterSize(param.maxClusterSize);
            ec.setSearchMethod(treeSegmentPcl);
            ec.setInputCloud(laserCloudIn); // feed point cloud
            ec.extract(cluster_indices); // get all clusters Indice
        }

        if (param.debugOctomapCluster) {
            cout << "extracted cluster: " << cluster_indices.size() << endl;
            if (param.debugOctomapTimeCost)  segmentationTime.printDuration("Segmentation time cost: ");
        }
        segmentationTime.recordStart();

        // 统计质心 的 label，不对segmentCloud的label重赋值
        for (auto &segment_to_add : cluster_indices) {
            size_t sz = segment_to_add.indices.size();
            Segment new_segment;

            // if (param.debugOctomapCluster) cout << "cur_segment_size: " << sz << endl;

            PointType centroid;
            centroid.x = 0.0; centroid.y = 0.0; centroid.z = 0.0;
            vector<unsigned int> class_counts(26, 0);
            for (auto &index : segment_to_add.indices) {
                PointType &cur_point = laserCloudIn->points[index];

                new_segment.segmentCloud.points.push_back(cur_point);

                // 统计标签 和 质心
                unsigned class_id = cur_point.label;
                ++class_counts[class_id];
                centroid.x += cur_point.x;
                centroid.y += cur_point.y;
                centroid.z += cur_point.z;
            }
            centroid.x /= (float)sz;
            centroid.y /= (float)sz;
            centroid.z /= (float)sz;

            float max_ratio = 0.0;
            for (int i = 1; i < 25; ++i) {
                float cur_ratio = class_counts[i] / (float)sz;
                if (cur_ratio > max_ratio) {
                    centroid.label = i;
                    max_ratio = cur_ratio;
                }
            }
            if (max_ratio < 0.5)
                centroid.label = 0;
            
            new_segment.centroid = centroid;
            new_segment.semantic = centroid.label;
            new_segment.box = BoundingBox2D(new_segment.segmentCloud);
            new_segment.boxCenter.x = new_segment.box.bboxTransform[0];
            new_segment.boxCenter.y = new_segment.box.bboxTransform[1];
            new_segment.boxCenter.z = new_segment.box.bboxTransform[2];

            segments.push_back(new_segment);
        }

        // Calculate time cost for segmentation
        if (param.debugOctomapTimeCost && param.debugOctomapCluster)
            segmentationTime.printDuration("PCA BoundingBox: ");

    }

    // PCA method
    Box BoundingBox2D(pcl::PointCloud<PointType> &cluster) {
        // PCA：计算主方向

        // ###### 3D 版本
        Eigen::Vector4f centroid;							// 质心
        pcl::compute3DCentroid(cluster, centroid);     	// 齐次坐标，（c0,c1,c2,1）

        Eigen::Matrix3f covariance;
        computeCovarianceMatrixNormalized(cluster, centroid, covariance);		// 计算归一化协方差矩阵
        // 去掉了 z轴 的影响
        covariance(0, 2) = 0.0;
        covariance(1, 2) = 0.0;
        covariance(2, 0) = 0.0;
        covariance(2, 1) = 0.0;
        covariance(2, 2) = 0.0;

        // 计算主方向：特征向量和特征值
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eigen_vectors = eigen_solver.eigenvectors();
        //Eigen::Vector3f eigen_values = eigen_solver.eigenvalues();

        eigen_vectors.col(2) = eigen_vectors.col(0).cross(eigen_vectors.col(1));	// 校正主方向间垂直（特征向量方向： (e0, e1, e0 × e1) --- note: e0 × e1 = +/- e2）

        // 转到参考坐标系，将点云主方向与参考坐标系的坐标轴进行对齐
        Eigen::Matrix4f transformation(Eigen::Matrix4f::Identity());

        transformation.block<3, 3>(0, 0) = eigen_vectors.transpose();										// R^(-1) = R^T
        transformation.block<3, 1>(0, 3) = -1.f * (transformation.block<3, 3>(0, 0) * centroid.head<3>());	// t^(-1) = -R^T * t

        pcl::PointCloud<PointType> transformed_cloud;	// 变换后的点云
        pcl::transformPointCloud(cluster, transformed_cloud, transformation);

        PointType min_pt, max_pt;						// 沿参考坐标系坐标轴的边界值
        pcl::getMinMax3D(transformed_cloud, min_pt, max_pt);
        const Eigen::Vector3f mean_diag = 0.5f * (max_pt.getVector3fMap() + min_pt.getVector3fMap());	// 形心

        // 参考坐标系到主方向坐标系的变换关系
        const Eigen::Quaternionf qfinal(eigen_vectors);
        const Eigen::Vector3f tfinal = eigen_vectors * mean_diag + centroid.head<3>();

        // 显示点云主方向
        Eigen::Vector3f whd;		// 3个方向尺寸：宽高深
        whd = max_pt.getVector3fMap() - min_pt.getVector3fMap();// getVector3fMap:返回Eigen::Map<Eigen::Vector3f>
        float scale = (whd(0) + whd(1) + whd(2)) / 3;			// 点云平均尺度，用于设置主方向箭头大小

        Box box;
        box.bboxTransform = tfinal;
        box.bboxQuaternion = qfinal;
        box.cube_width = whd(0);
        box.cube_length = whd(1);
        box.cube_height = whd(2);

        return box;
    }

    bool Box2Vertexes(Box &box, IOU::Vertexes &vertex, double &z_upper, double &z_lower) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointXYZ cur_p;
        for (int i = -1; i <= 1; i += 2) {
            for (int j = -1; j <= 1; j += 2) {
                cur_p.x = i * box.cube_width / 2.0;
                cur_p.y = j * box.cube_length / 2.0;
                cur_p.z = 0;
                cloud.points.push_back(cur_p);
            }
        }
        
        z_upper = box.bboxTransform[2] + box.cube_height / 2.0;
        z_lower = box.bboxTransform[2] - box.cube_height / 2.0;

        Eigen::Matrix4f transformation(Eigen::Matrix4f::Identity());
        transformation.block<3, 3>(0, 0) = box.bboxQuaternion.toRotationMatrix();
        transformation(0, 3) = box.bboxTransform[0];
        transformation(1, 3) = box.bboxTransform[1];
        transformation(2, 3) = box.bboxTransform[2];
        pcl::PointCloud<pcl::PointXYZ> transformed_cloud;	// 变换后的点云
        pcl::transformPointCloud(cloud, transformed_cloud, transformation);

        for (auto &p : transformed_cloud.points)
            vertex.push_back(IOU::Point(p.x, p.y));
        IOU::beInSomeWiseEx(vertex, IOU::ClockWise);
        if (IOU::whichWiseEx(vertex) == IOU::NoneWise)
            return false;

        return true;
    }
};


#endif






