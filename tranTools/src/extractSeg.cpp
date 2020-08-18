#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/common/common.h>

using namespace std;

//存放计算点云的idx和点云编号的cloud_point_index的结构头
struct cloud_point_index_idx
{
	unsigned int idx;
	unsigned int cloud_point_index;

	cloud_point_index_idx(unsigned int idx_, unsigned int cloud_point_index_) : idx(idx_), cloud_point_index(cloud_point_index_) {}
	bool operator < (const cloud_point_index_idx &p) const { return (idx < p.idx); }
};

//中间参数的结构体
struct Array4f
{
	float x;
	float y;
	float z;
	float C;
};

struct _VoxelPoint              //定义点类型结构
{
    PCL_ADD_POINT4D                // 该点类型有4个元素
    PCL_ADD_INTENSITY;

    int label_counter[6];
    int pointNum;
    int label;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW// 确保new操作符对齐操作
}EIGEN_ALIGN16;// 强制SSE对齐

struct VoxelPoint : public _VoxelPoint {
	inline VoxelPoint() {
		x= y = z = intensity = 0.0f;
		pointNum = 0;
		label = 0;
		for (int i = 0; i < 6; ++i) {
			label_counter[6] = 0;
		}
	}
};

/* 添加了语义标签的累计 */
template<typename InputPointT, typename OutputPointT>
void VoxelGrid_LabelFilter(pcl::PointCloud<InputPointT> &InputCloudPoint, pcl::PointCloud<OutputPointT> &OutPointCloud, float X_Voxel, float Y_Voxel, float Z_Voxel)
{
	//先判断输入的点云是否为空
	if (InputCloudPoint.points.size()==0)
	{
		// std::cout << "输入点云为空！" << std::endl;
		return;
	}
	//存放输入点云的最大与最小坐标
	// Array4f min_p, max_p;
	// GetMaxMinLabel(InputCloudPoint, min_p, max_p);
    pcl::PointXYZL min_p, max_p;
    pcl::getMinMax3D(InputCloudPoint, min_p, max_p);

	Array4f inverse_leaf_size_;
	inverse_leaf_size_.x = 1 / X_Voxel;
	inverse_leaf_size_.y = 1 / Y_Voxel;
	inverse_leaf_size_.z = 1 / Z_Voxel;
	inverse_leaf_size_.C = 1;

    //计算最小和最大边界框值
	Array4f min_b_, max_b_, div_b_, divb_mul_;
	min_b_.x = static_cast<int> (floor(min_p.x * inverse_leaf_size_.x));
	max_b_.x = static_cast<int> (floor(max_p.x * inverse_leaf_size_.x));
	min_b_.y = static_cast<int> (floor(min_p.y * inverse_leaf_size_.y));
	max_b_.y = static_cast<int> (floor(max_p.y * inverse_leaf_size_.y));
	min_b_.z = static_cast<int> (floor(min_p.z * inverse_leaf_size_.z));
	max_b_.z = static_cast<int> (floor(max_p.z * inverse_leaf_size_.z));

	//计算沿所有轴所需的分割数
	div_b_.x = max_b_.x - min_b_.x + 1;
	div_b_.y = max_b_.y - min_b_.y + 1;
	div_b_.z = max_b_.z - min_b_.z + 1;
	div_b_.C= 0;

	//设置除法乘数
	divb_mul_.x = 1;
	divb_mul_.y = div_b_.x;
	divb_mul_.z =div_b_.x * div_b_.y;
	divb_mul_.C = 0;

	//用于计算idx和pointcloud索引的存储
	std::vector<cloud_point_index_idx> index_vector;
	index_vector.reserve(InputCloudPoint.points.size());

	//第一步：遍历所有点并将它们插入到具有计算idx的index_vector向量中;具有相同idx值的点将有助于产生CloudPoint的相同点
	for (int i = 0; i < InputCloudPoint.points.size();i++)
	{
		int ijk0 = static_cast<int> (floor(InputCloudPoint.points[i].x * inverse_leaf_size_.x) - static_cast<float> (min_b_.x));
		int ijk1 = static_cast<int> (floor(InputCloudPoint.points[i].y * inverse_leaf_size_.y) - static_cast<float> (min_b_.y));
		int ijk2 = static_cast<int> (floor(InputCloudPoint.points[i].z * inverse_leaf_size_.z) - static_cast<float> (min_b_.z));

		//计算质心叶索引
		int idx = ijk0 * divb_mul_.x + ijk1 * divb_mul_.y + ijk2 * divb_mul_.z;
		index_vector.push_back(cloud_point_index_idx(static_cast<unsigned int> (idx), i));
	}
	//第二步：使用表示目标单元格的值作为索引对index_vector向量进行排序;实际上属于同一输出单元格的所有点都将彼此相邻
	std::sort(index_vector.begin(), index_vector.end(), std::less<cloud_point_index_idx>());

	//第三步：计数输出单元格，我们需要跳过所有相同的，相邻的idx值
	unsigned int total = 0;
	unsigned int index = 0;
	unsigned int min_points_per_voxel_ = 0;
	//first_and_last_indices_vector [i]表示属于对应于第i个输出点的体素的index_vector中的第一个点的index_vector中的索引，以及不属于第一个点的索引
	std::vector<std::pair<unsigned int, unsigned int> > first_and_last_indices_vector;
	first_and_last_indices_vector.reserve(index_vector.size());                              //分配内存空间

	while (index < index_vector.size())
	{
		unsigned int i = index + 1;
		while (i < index_vector.size() && index_vector[i].idx == index_vector[index].idx)
			++i;
		if (i - index >= min_points_per_voxel_)
		{
			++total;
			first_and_last_indices_vector.push_back(std::pair<unsigned int, unsigned int>(index, i));
		}
		index = i;
	}

	// 第四步：计算质心，将它们插入最终位置
	// OutPointCloud.resize(total);      //给输出点云分配内存空间
    // 处理过程需要根据OutputPointT类型修改
	float x_Sum, y_Sum, z_Sum;
	unsigned int first_index, last_index;
	for (unsigned int cp = 0; cp < first_and_last_indices_vector.size(); ++cp)
	{
		// 计算质心 - 来自所有输入点的和值，这些值在index_vector数组中具有相同的idx值
		first_index = first_and_last_indices_vector[cp].first;
		last_index = first_and_last_indices_vector[cp].second;
		OutputPointT PointCloud;
		x_Sum = 0;
		y_Sum = 0;
		z_Sum = 0;
		for (unsigned int li = first_index; li < last_index; ++li)
		{
			x_Sum += InputCloudPoint[index_vector[li].cloud_point_index].x;
			y_Sum += InputCloudPoint[index_vector[li].cloud_point_index].y;
			z_Sum += InputCloudPoint[index_vector[li].cloud_point_index].z;

			// PointCloud.pointNum += InputCloudPoint[index_vector[li].cloud_point_index].pointNum;
			// for (int j = 0; j < 6; ++j) {
			// 	PointCloud.label_counter[j] += InputCloudPoint[index_vector[li].cloud_point_index].label_counter[j];
			// }

		}

		PointCloud.x = x_Sum / (last_index - first_index);
		PointCloud.y = y_Sum / (last_index - first_index);
		PointCloud.z = z_Sum / (last_index - first_index);

		// for (int j = 0; j < 6; ++j) {
		// 	if (float(PointCloud.label_counter[j]) / float(PointCloud.pointNum) > 0.5) {
		// 		PointCloud.label = j + 1;
		// 		PointCloud.intensity = float((j + 1) * 100);
		// 		break;
		// 	}
		// }

		OutPointCloud.push_back(PointCloud);
	}

	return;
}

string input_cloud = "/home/wzc/Project/ICICIP/labelMap09.txt";
ifstream if_cloud;

int main (int argc, char** argv)
{
    cout << sizeof(int) << endl;
    if_cloud.open(input_cloud.c_str());
    unsigned int count = 0, count_w = 0;
    pcl::PointCloud<pcl::PointXYZL>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZL>);
    cloud->reserve(100000000);
    while (!if_cloud.eof()) {
        pcl::PointXYZL pointIn;
        if_cloud >> pointIn.x >> pointIn.y >> pointIn.z >> pointIn.label;
        cloud->push_back(pointIn);
        ++count;
        if (count == 10000) {
            count = 0;
            ++count_w;
            cout << count_w << " w  loaded..." << endl;
        }
    }
    cout << "Total points loaded: " << cloud->size() << endl;

    // 体素滤波
    pcl::PointCloud<pcl::PointXYZL>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZL>);
    filteredCloud->reserve(100000000);
    VoxelGrid_LabelFilter<pcl::PointXYZL, pcl::PointXYZL>(*cloud, *filteredCloud, 0.1, 0.1, 0.1);
    cout << filteredCloud->size() << " filtered points..." << endl;

    // 设置搜索的方式或者说是结构
    pcl::search::Search<pcl::PointXYZL>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZL> >(new pcl::search::KdTree<pcl::PointXYZL>);
    // 求法线
    pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZL, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(filteredCloud);
	normal_estimator.setKSearch(50);
	normal_estimator.compute(*normals);
    cout << normals->size() << " normals was computed..." << endl;

    // 直通滤波在Z轴的0到1米之间
	// pcl::IndicesPtr indices(new std::vector <int>);
	// pcl::PassThrough<pcl::PointXYZ> pass;
	// pass.setInputCloud(cloud);
	// pass.setFilterFieldName("z");
	// pass.setFilterLimits(0.0, 1.0);
	// pass.filter(*indices);

    // 聚类对象<点，法线>
	pcl::RegionGrowing<pcl::PointXYZL, pcl::Normal> reg;
	reg.setMinClusterSize(50);  //最小的聚类的点数
	reg.setMaxClusterSize(1000000);  //最大的
	reg.setSearchMethod(tree);    //搜索方式
	reg.setNumberOfNeighbours(20);    //设置搜索的邻域点的个数
	reg.setInputCloud(filteredCloud);         //输入点
	//reg.setIndices (indices);
	reg.setInputNormals(normals);     //输入的法线
	reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);  //设置平滑度
	reg.setCurvatureThreshold(0.5);     //设置曲率的阈值

	std::vector <pcl::PointIndices> clusters;
    clusters.reserve(10000000);
	reg.extract(clusters);
    cout << "Finish extracting clusters..." << endl;
    cout << "Total amount of clusters: " << clusters.size() << endl;


    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
    pcl::visualization::CloudViewer viewer ("Cluster viewer");
    viewer.showCloud(colored_cloud);
    while (!viewer.wasStopped ()) {

    }

    return 0;
}
