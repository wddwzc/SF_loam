#ifndef _SEGMENT_H_
#define _SEGMENT_H_

#include "utility.h"
#include <Eigen/Geometry>

struct BoxSimple
{
    // 中心位置
	float x;
	float y;
	float z;
	float cube_width;
    float cube_length;
    float cube_height;
};

struct Box
{
	Eigen::Vector3f bboxTransform;      // box 中心
	Eigen::Quaternionf bboxQuaternion;  // box 的朝向
	float cube_width;
    float cube_length;
    float cube_height;
};

class Segment {
public:
    Segment() {
        centroid.x = 0.0;
        centroid.y = 0.0;
        centroid.z = 0.0;
    }

    // 更新当前 segment 的box和centroids
    void updateBox() {
        if (segmentCloud.empty())
            return;
        boxS = BoundingBoxSimple();
        box = BoundingBox();
        boxCenter.x = box.bboxTransform[0];
        boxCenter.y = box.bboxTransform[1];
        boxCenter.z = box.bboxTransform[2];
    }

    PointType centroid;                         // 点云的质心
    PointType boxCenter;                        // box 的中心
    pcl::PointCloud<PointType> segmentCloud;    // segment 中包含的点云

    BoxSimple boxS;
    Box box;
    unsigned int semantic;                      // 语义标签


private:

    // 根据当前 segmentCloud 计算 BoxSimple
    BoxSimple BoundingBoxSimple() {

        // 直接根据 xyz 边界值画框
        // Find bounding box for one of the clusters
        PointType minPoint, maxPoint;
        pcl::getMinMax3D(segmentCloud, minPoint, maxPoint);

        BoxSimple tempBox;
        tempBox.x = (maxPoint.x + minPoint.x) / 2;
        tempBox.y = (maxPoint.y + minPoint.y) / 2;
        tempBox.z = (maxPoint.z + minPoint.z) / 2;
        tempBox.cube_width = maxPoint.x;
        tempBox.cube_length = maxPoint.y;
        tempBox.cube_height = maxPoint.z;

        return tempBox;
    }


    // 根据当前 segmentCloud 重新计算 box
    Box BoundingBox() {
        // PCA：计算主方向

        // ###### 3D 版本
        Eigen::Vector4f new_centroid;							// 质心
        pcl::compute3DCentroid(segmentCloud, new_centroid);     	// 齐次坐标，（c0,c1,c2,1）

        centroid.x = new_centroid[0];
        centroid.y = new_centroid[1];
        centroid.z = new_centroid[2];

        Eigen::Matrix3f covariance;
        computeCovarianceMatrixNormalized(segmentCloud, new_centroid, covariance);		// 计算归一化协方差矩阵
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
        transformation.block<3, 1>(0, 3) = -1.f * (transformation.block<3, 3>(0, 0) * new_centroid.head<3>());	// t^(-1) = -R^T * t

        pcl::PointCloud<PointType> transformed_cloud;	// 变换后的点云
        pcl::transformPointCloud(segmentCloud, transformed_cloud, transformation);

        PointType min_pt, max_pt;						// 沿参考坐标系坐标轴的边界值
        pcl::getMinMax3D(transformed_cloud, min_pt, max_pt);
        const Eigen::Vector3f mean_diag = 0.5f * (max_pt.getVector3fMap() + min_pt.getVector3fMap());	// 形心

        // 参考坐标系到主方向坐标系的变换关系
        const Eigen::Quaternionf qfinal(eigen_vectors);
        const Eigen::Vector3f tfinal = eigen_vectors * mean_diag + new_centroid.head<3>();

        // 显示点云主方向
        Eigen::Vector3f whd;		// 3个方向尺寸：宽高深
        whd = max_pt.getVector3fMap() - min_pt.getVector3fMap();// getVector3fMap:返回Eigen::Map<Eigen::Vector3f>
        float scale = (whd(0) + whd(1) + whd(2)) / 3;			// 点云平均尺度，用于设置主方向箭头大小

        Box tempBox;
        tempBox.bboxTransform = tfinal;
        tempBox.bboxQuaternion = qfinal;
        tempBox.cube_width = whd(0);
        tempBox.cube_length = whd(1);
        tempBox.cube_height = whd(2);

        return box;
    }

};

#endif
