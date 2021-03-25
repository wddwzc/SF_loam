/****************** Semantic ********************************/
    ros::Publisher pubColoredLaserCloud;
    ros::Publisher pubClassifiedCentroidRGB;
    ros::Publisher pubClassifiedCloud;
    ros::Publisher pubCentroid2Seg;
    ros::Publisher pubClassifiedCentroid; // for mapOptimization
    ros::Publisher pubNoGroundCloud;  // for mapOptimization
    // PointXYZRGB for visulizer
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudColor;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr classifiedCentroidRGB;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr classifiedCloud;
    visualization_msgs::Marker marker_Centroid2Seg;
    pcl::PointCloud<pcl::PointXYZL>::Ptr classifiedCentroid; // for mapOptimization
    pcl::PointCloud<pcl::PointXYZL>::Ptr laserCloud_noGround; // for mapOptimization

    pcl::PointCloud<PointType>::Ptr laserCloudIn;
    pcl::PointCloud<PointXYZIR>::Ptr laserCloudInRing;
/************************************************************/


/****************** Semantic ********************************/
        // laserCloudColor->clear();
        // classifiedCentroidRGB->clear();
        // classifiedCloud->clear();
        // classifiedCentroid->clear();
        // laserCloud_noGround->clear();
        // // index_full.assign(param.N_SCAN*param.Horizon_SCAN, -1);
        // marker_Centroid2Seg.points.clear();
/************************************************************/



//    void geometricSegmentation() {

//         // Calculate time cost for segmentation
//         double begin_time_ = ros::Time::now().toSec();

//         if (laserCloud_noGround->points.size() == 0)  return;

//         if (param.debugCluster)
//             cout << "point size without ground: " << laserCloud_noGround->points.size() << endl;

//         // PointType is PointXYZL
//         pcl::search::KdTree<pcl::PointXYZL>::Ptr treeSegment(new pcl::search::KdTree<pcl::PointXYZL>);
//         treeSegment->setInputCloud(laserCloud_noGround);
//         std::vector<pcl::PointIndices> cluster_indices;
//         float radius_for_growing_ = 0.2;
//         unsigned int core_MinPts_ = 10;  // only valid in DBSCAN
//         unsigned int min_segment_size_ = 100;
//         unsigned int max_segment_size_ = 25000;

//         if (param.debugCluster) cout << "extracting cluster" << endl;

//         // DBSCAN
//         DBSCANKdtreeCluster<pcl::PointXYZL> ec;
//         ec.setCorePointMinPts(core_MinPts_);
//         ec.setClusterTolerance(radius_for_growing_);
//         ec.setMinClusterSize(min_segment_size_);
//         ec.setMaxClusterSize(max_segment_size_);
//         ec.setSearchMethod(treeSegment);
//         ec.setInputCloud(laserCloud_noGround);
//         ec.extract(cluster_indices);

//         // EuclideanClusterExtraction
//         // function pattern

//         // pcl::extractEuclideanClusters<pcl::PointXYZL>(*laserCloud_noGround, treeSegment, radius_for_growing_, 
//         //                                              cluster_indices, min_segment_size_, max_segment_size_);
//         if (param.debugCluster) cout << "extracted cluster" << endl;

//         for (auto &segment_to_add : cluster_indices) {
//             unsigned int sz = segment_to_add.indices.size();

//             if (param.debugCluster) cout << "cur_segment_size: " << sz << endl;

//             pcl::PointXYZRGB centroid(0.0, 0.0, 0.0);
//             vector<unsigned int> class_counts(26, 0);
//             for (auto &index : segment_to_add.indices) {
//                 pcl::PointXYZL &cur_point = laserCloud_noGround->points[index];
//                 unsigned class_id = classes_map[cur_point.label];
//                 ++class_counts[class_id];
//                 centroid.x += cur_point.x;
//                 centroid.y += cur_point.y;
//                 centroid.z += cur_point.z;
//             }
//             centroid.x /= (float)sz;
//             centroid.y /= (float)sz;
//             centroid.z /= (float)sz;

//             unsigned int segment_class = 0;
//             for (int i = 0; i < 25; ++i) {
//                 if (class_counts[i] / (float)sz >= 0.6)
//                     segment_class = i;
//             }
//             centroid.r = colors_map_tran[segment_class][0];
//             centroid.g = colors_map_tran[segment_class][1];
//             centroid.b = colors_map_tran[segment_class][2];

//             // add marker
//             geometry_msgs::Point marker_p;
//             marker_p.x = centroid.x;
//             marker_p.y = centroid.y;
//             marker_p.z = centroid.z;
//             marker_Centroid2Seg.points.push_back(marker_p);
//             marker_p.z += 10.0;
//             marker_Centroid2Seg.points.push_back(marker_p);

//             classifiedCentroidRGB->push_back(centroid);
//             centroid.z += 10.0;
//             classifiedCentroidRGB->push_back(centroid);

//             for (auto &index : segment_to_add.indices) {
//                 pcl::PointXYZL cur_point = laserCloud_noGround->points[index];
//                 pcl::PointXYZRGB cur_p;
//                 cur_p.x = cur_point.x;
//                 cur_p.y = cur_point.y;
//                 cur_p.z = cur_point.z;
//                 cur_p.r = colors_map_tran[segment_class][0];
//                 cur_p.g = colors_map_tran[segment_class][1];
//                 cur_p.b = colors_map_tran[segment_class][2];
//                 classifiedCloud->push_back(cur_p);
//             }
//         }

//         // Calculate time cost for segmentation
//         double end_time_ = ros::Time::now().toSec();
//         if (param.debugTimeCost)
//             std::cout << "Segmentation time: " << (end_time_ - begin_time_) * 1000 << std::endl;
//     }




        // if (pubClassifiedCloud.getNumSubscribers() != 0) {
        //     pcl::toROSMsg(*classifiedCloud, laserCloudTemp);
        //     laserCloudTemp.header.stamp = cloudHeader.stamp;
        //     laserCloudTemp.header.frame_id = "base_link";
        //     pubClassifiedCloud.publish(laserCloudTemp);
        // }

        // if (pubClassifiedCentroidRGB.getNumSubscribers() != 0) {
        //     pcl::toROSMsg(*classifiedCentroidRGB, laserCloudTemp);
        //     laserCloudTemp.header.stamp = cloudHeader.stamp;
        //     laserCloudTemp.header.frame_id = "base_link";
        //     pubClassifiedCentroidRGB.publish(laserCloudTemp);
        // }


        // if (pubClassifiedCentroid.getNumSubscribers() != 0) {
        //     pcl::toROSMsg(*classifiedCentroid, laserCloudTemp);
        //     laserCloudTemp.header.stamp = cloudHeader.stamp;
        //     laserCloudTemp.header.frame_id = "base_link";
        //     pubClassifiedCentroid.publish(laserCloudTemp);
        // }
        
        // if (pubCentroid2Seg.getNumSubscribers() != 0) {
        //     pubCentroid2Seg.publish(marker_Centroid2Seg);
        // }

        // // if (pubNoGroundCloud.getNumSubscribers() != 0) {
        // //     pcl::toROSMsg(*laserCloud_noGround, laserCloudTemp);
        // //     laserCloudTemp.header.stamp = cloudHeader.stamp;
        // //     laserCloudTemp.header.frame_id = "base_link";
        // //     pubNoGroundCloud.publish(laserCloudTemp);
        // // }



        // // 更新拟合平面的A B C D
// void
// GroundFit::estimate_plane_()
// {
//     // Create covarian matrix in single pass.
//     // TODO: compare the efficiency.
//     Eigen::Matrix3f cov;
//     Eigen::Vector4f pc_mean;   // 归一化坐标值
//     pcl::computeMeanAndCovarianceMatrix(*g_ground_pc, cov, pc_mean); // 对地面点(最小的n个点)进行计算协方差和平均值
//     // Singular Value Decomposition: SVD
//     JacobiSVD<MatrixXf> svd(cov,Eigen::DecompositionOptions::ComputeFullU);
//     // use the least singular vector as normal
//     normal_ = (svd.matrixU().col(2));   // 取最小的特征值对应的特征向量作为法向量
//     // cout<<"normal_ \n"<<normal_<<endl;
//     // mean ground seeds value
//     Eigen::Vector3f seeds_mean = pc_mean.head<3>();   // seeds_mean 地面点的平均值
//     // cout<<"seeds_mean \n"<<seeds_mean<<endl;

//     // according to normal.T*[x,y,z] = -d
//     d_ = -(normal_.transpose()*seeds_mean)(0,0);  // 计算d   D=d
// //    std::cout<<"d_: "<<d_<<std::endl;
//     // set distance threhold to `th_dist - d`
//     th_dist_d_ = th_dist_ - d_;   // ------------------------------? // 这里只考虑在拟合的平面上方的点 小于这个范围的点当做地面
// //    std::cout<<"th_dist_d_=th_dist_ - d_ : "<<th_dist_d_<<std::endl;

//     // return the equation parameters

// }





#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <iostream>
#include <unordered_map>
#include <algorithm>
#include <pcl/filters/voxel_grid.h> 

using namespace std;
const float PI = 3.1415926;
template<typename T> string toString(const T& t) {
	ostringstream oss;
	oss << t;
	return oss.str();
}
static int64_t gtm() {
	struct timeval tm;
	gettimeofday(&tm, 0);
	int64_t re = (((int64_t) tm.tv_sec) * 1000 * 1000 + tm.tv_usec);
	return re;
}

float Polar_angle_cal(float x, float y){
  float temp_tangle = 0;
  if(x== 0 && y ==0){
     temp_tangle = 0;
  }else if(y>=0){
     temp_tangle = (float)atan2(y,x);
  }else if(y<=0){
     temp_tangle = (float)atan2(y,x)+2*PI;
  }
 return temp_tangle;
}




struct PointAPR{
   float azimuth;
   float polar_angle;
   float range;
};

struct Voxel{
   bool haspoint = false;
   int cluster = -1;
   vector<int> index;
};


float minrange = 3;
float maxrange = 3;
float minazimuth = 0;
float maxazimuth = 0;
float deltaA = 2;
float deltaR = 0.35;
float deltaP = 1.2;
int length = 0;
int width = 0;
int height = 0;

bool compare_cluster(pair<int,int> a,pair<int,int> b){
    return a.second>b.second;
}//升序


template <typename PointT>
void calculateAPR(const pcl::PointCloud<PointT>& cloud_IN, vector<PointAPR>& vapr){
     for (int i =0; i<cloud_IN.points.size(); ++i){
           PointAPR par;
           par.polar_angle = Polar_angle_cal(cloud_IN.points[i].x, cloud_IN.points[i].y);
           par.range = sqrt(cloud_IN.points[i].x*cloud_IN.points[i].x+cloud_IN.points[i].y*cloud_IN.points[i].y);
           par.azimuth =(float) atan2(cloud_IN.points[i].z,par.range);
           if(par.azimuth < minazimuth){
                 minazimuth = par.azimuth;
           }
           if(par.azimuth > maxazimuth){
                 maxazimuth = par.azimuth;
           }
           if(par.range < minrange){
                 minrange = par.range;
           }
           if(par.range > maxrange){
                 maxrange = par.range;
           }
           vapr.push_back(par);
    }

  length = round((maxrange - minrange)/deltaR);
  width = 301;
  height = round((maxazimuth - minazimuth)/deltaA);
  
}


void build_hash_table(const vector<PointAPR>& vapr, unordered_map<int, Voxel> &map_out){
     vector<int> ri;
     vector<int> pi;
     vector<int> ai;
     for(int i =0; i< vapr.size(); ++i){
           int azimuth_index = round(((vapr[i].azimuth-minazimuth)*180/PI)/deltaA);
           int polar_index = round(vapr[i].polar_angle*180/PI/deltaP);
           int range_index = round((vapr[i].range-minrange)/deltaR);
           int voxel_index = (polar_index*(length+1)+range_index)+azimuth_index*(length+1)*(width+1);
           ri.push_back(range_index);
           pi.push_back(polar_index);           
           ai.push_back(azimuth_index);
           unordered_map<int, Voxel>::iterator it_find;
           it_find = map_out.find(voxel_index);
           if (it_find != map_out.end()){
                it_find->second.index.push_back(i);

           }else{
                Voxel vox;
                vox.haspoint =true;
                vox.index.push_back(i); 
                vox.index.swap(vox.index);
                map_out.insert(make_pair(voxel_index,vox));
           }

    }
    auto maxPosition = max_element(ai.begin(), ai.end());
    auto maxPosition1 = max_element(ri.begin(), ri.end());
    auto maxPosition2 = max_element(pi.begin(), pi.end());
    cout<<*maxPosition<<" "<<*maxPosition1<<" "<<*maxPosition2<<endl;

}
     

void find_neighbors(int polar, int range, int azimuth, vector<int>& neighborindex){
	for (int z = azimuth - 1; z <= azimuth + 1; z++){
		if (z < 0 || z >round((maxazimuth-minazimuth)*180/PI/deltaA)){
			continue;
		}

		for (int y = range - 1; y <= range + 1; y++){
			if (y < 0 || y >round((50-minrange)/deltaR)){
				continue;
			}

			for (int x = polar - 1; x <= polar + 1; x++){
                                int px = x;
				if (x < 0 ){
					px=300;
				}
                                if(x>300){
                                        px=0;

                                }

				neighborindex.push_back((px*(length+1)+y)+z*(length+1)*(width+1));
                        }
               }
        }
}


bool most_frequent_value(vector<int> values, vector<int> &cluster_index) {
	unordered_map<int, int> histcounts;
	for (int i = 0; i < values.size(); i++) {
		if (histcounts.find(values[i]) == histcounts.end()) {
			histcounts[values[i]] = 1;
		}
		else {
			histcounts[values[i]] += 1;
		}
	}

	int max = 0, maxi;
	vector<pair<int, int>> tr(histcounts.begin(), histcounts.end());
        sort(tr.begin(),tr.end(),compare_cluster);
        for(int i = 0 ; i< tr.size(); ++i){
             if(tr[i].second>10){
             cluster_index.push_back(tr[i].first);
             }
        }

	return true;
}


void mergeClusters(vector<int>& cluster_indices, int idx1, int idx2) {
	for (int i = 0; i < cluster_indices.size(); i++) {
		if (cluster_indices[i] == idx1) {
			cluster_indices[i] = idx2;
		}
	}
}

vector<int>  CVC(unordered_map<int, Voxel> &map_in,const vector<PointAPR>& vapr){
     int current_cluster = 0;
     cout<<"CVC"<<endl;
     vector<int> cluster_indices = vector<int>(vapr.size(), -1);

     for(int i = 0; i< vapr.size(); ++i){

           if (cluster_indices[i] != -1)
			   continue;
           int azimuth_index = round((vapr[i].azimuth+fabs(minazimuth))*180/PI/deltaA);
           int polar_index = round(vapr[i].polar_angle*180/PI/deltaP);
           int range_index = round((vapr[i].range-minrange)/deltaR);
           int voxel_index = (polar_index*(length+1)+range_index)+azimuth_index*(length+1)*(width+1);
           
           unordered_map<int, Voxel>::iterator it_find;
           unordered_map<int, Voxel>::iterator it_find2;

           it_find = map_in.find(voxel_index);
           vector<int> neightbors;

           if (it_find != map_in.end()){

               vector<int> neighborid;
               find_neighbors(polar_index, range_index, azimuth_index, neighborid);
               for (int k =0; k<neighborid.size(); ++k){
                     
                  it_find2 = map_in.find(neighborid[k]);

                  if (it_find2 != map_in.end()){

                     for(int j =0 ; j<it_find2->second.index.size(); ++j){
                        neightbors.push_back(it_find2->second.index[j]);
                      }
                   }
                }
            }
       
            neightbors.swap(neightbors);

            if(neightbors.size()>0){
                   for(int j =0 ; j<neightbors.size(); ++j){
                      int oc = cluster_indices[i] ;
                      int nc = cluster_indices[neightbors[j]];
		      if (oc != -1 && nc != -1) {
				if (oc != nc)
					mergeClusters(cluster_indices, oc, nc);
				}
		      else {
				if (nc != -1) {
					cluster_indices[i] = nc;
				}
				else {
					if (oc != -1) {
						cluster_indices[neightbors[j]] = oc;
					}
				}
			}

                   }
                }
                          
 		if (cluster_indices[i] == -1) {
			current_cluster++;
			cluster_indices[i] = current_cluster;
                   for(int s =0 ; s<neightbors.size(); ++s){             
                        cluster_indices[neightbors[s]] = current_cluster;
		   }
               }


          }
	return cluster_indices;
}


vector<float> hsv2rgb(vector<float>& hsv){
vector<float> rgb(3);
float R,G,B,H,S,V;
H = hsv[0];
S = hsv[1];
V = hsv[2];
if(S == 0){
rgb[0]=rgb[1]=rgb[2]=V;
}else{

int i = int(H*6);
float f = (H*6) - i;
float a = V * ( 1 - S );
float b = V * ( 1 - S * f );
float c = V * ( 1 - S * (1 - f ) );
i = i%6;
switch(i){
   case 0: {rgb[0] = V; rgb[1]= c; rgb[2] = a; break;}
   case 1: {rgb[0] = b; rgb[1] = V; rgb[2] = a;break;}
   case 2: {rgb[0] = a; rgb[1] = V; rgb[2] = c;break;}
   case 3: {rgb[0] = a; rgb[1] = b; rgb[2] = V;break;}
   case 4: {rgb[0] = c; rgb[1] = a; rgb[2] = V;break;}
   case 5: {rgb[0] = V; rgb[1] = a; rgb[2] = b;break;}
  }
}

return rgb;
}
