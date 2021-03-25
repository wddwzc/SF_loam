#include "utility.h"

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

struct PointAPR{
   float azimuth;
   float range;
   float polar_angle;
};

struct Voxel{
   bool haspoint = false;
   int cluster = -1;
   vector<int> index;
};

bool compare_cluster(pair<int,int> a, pair<int,int> b){
    return a.second > b.second;
} // 升序

template<typename PointT>
class CVCCluster 
{
public:
   CVCCluster() {}

   float minrange {2};
   float maxrange {3};
   float minazimuth {0};
   float maxazimuth {0};
   float deltaA {2};
   float deltaR {0.35};
   float deltaP {1.2};
   int ASize{0};
   int RSize{0};
   int PSize{0};
   int minPointsNum{10};

   typename pcl::PointCloud<PointT>::Ptr pointCloudPtr;

   void extract(std::vector<pcl::PointIndices> &indices) {
      vector<PointAPR> papr;
      calculateAPR(papr);
      unordered_map<int, Voxel> hvoxel;
      build_hash_table(papr, hvoxel);
      vector<int> cluster_index = CVC(hvoxel, papr);
      vector<int> cluster_id;
      most_frequent_value(cluster_index, cluster_id);
   }

   void calculateAPR(vector<PointAPR>& vapr){
      for (int i = 0; i < pointCloudPtr->points.size(); ++i) {
         PointAPR par;
         PointT &p = pointCloudPtr->points[i];
         par.polar_angle = Polar_angle_cal(p.x, p.y);
         par.range = sqrt(p.x * p.x + p.y * p.y);
         par.azimuth =(float)atan2(p.z, par.range);
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

      ASize = round((maxazimuth - minazimuth) / deltaA);
      RSize = round((maxrange - minrange) / deltaR);
      PSize = round(360.0 / deltaP);

   }

   void build_hash_table(const vector<PointAPR>& vapr, unordered_map<int, Voxel> &map_out) {
      vector<int> ai;
      vector<int> ri;
      vector<int> pi;
      for(int i = 0; i < vapr.size(); ++i) {
            int azimuth_index = round(((vapr[i].azimuth - minazimuth) * 180 / PI_M) / deltaA);
            int polar_index = round(vapr[i].polar_angle * 180 / PI_M / deltaP);
            int range_index = round((vapr[i].range - minrange) / deltaR);
            int voxel_index = range_index + polar_index * (RSize + 1) + azimuth_index * (RSize + 1) * (PSize + 1);
            ri.push_back(range_index);
            pi.push_back(polar_index);
            ai.push_back(azimuth_index);
            unordered_map<int, Voxel>::iterator it_find;
            it_find = map_out.find(voxel_index);
            if (it_find != map_out.end()){
               it_find->second.index.push_back(i);
            }
            else {
               Voxel vox;
               vox.haspoint = true;
               vox.index.push_back(i); 
               vox.index.swap(vox.index);
               map_out.insert(make_pair(voxel_index, vox));
            }
      }
      auto maxPosition = max_element(ai.begin(), ai.end());
      auto maxPosition1 = max_element(ri.begin(), ri.end());
      auto maxPosition2 = max_element(pi.begin(), pi.end());
      cout<< *maxPosition << " " << *maxPosition1 << " " << *maxPosition2 << endl;
   }

   vector<int> CVC(unordered_map<int, Voxel> &map_in, const vector<PointAPR>& vapr){
      int current_cluster = 0;
      cout << "CVC" << endl;
      vector<int> cluster_indices = vector<int>(vapr.size(), -1);

      for(int i = 0; i < vapr.size(); ++i){

         if (cluster_indices[i] != -1)
            continue;
         int azimuth_index = round((vapr[i].azimuth + fabs(minazimuth)) * 180 / PI_M / deltaA);
         int polar_index = round(vapr[i].polar_angle * 180 / PI_M / deltaP);
         int range_index = round((vapr[i].range - minrange) / deltaR);
         int voxel_index = range_index + polar_index * (RSize + 1) + azimuth_index * (RSize + 1) * (PSize + 1);
         
         unordered_map<int, Voxel>::iterator it_find;
         unordered_map<int, Voxel>::iterator it_find2;

         it_find = map_in.find(voxel_index);
         vector<int> neightbors;

         if (it_find != map_in.end()){
            vector<int> neighborid;
            find_neighbors(polar_index, range_index, azimuth_index, neighborid);
            for (int k = 0; k < neighborid.size(); ++k){
                  
               it_find2 = map_in.find(neighborid[k]);

               if (it_find2 != map_in.end()){
                  for(int j = 0 ; j < it_find2->second.index.size(); ++j){
                     neightbors.push_back(it_find2->second.index[j]);
                  }
               }
            }
         }
         
         neightbors.swap(neightbors);

         if(neightbors.size() > 0) {
            for(int j =0 ; j < neightbors.size(); ++j) {
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
            for(int s =0 ; s < neightbors.size(); ++s) {             
                  cluster_indices[neightbors[s]] = current_cluster;
            }
         }

      }
      return cluster_indices;
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

      vector<pair<int, int>> tr(histcounts.begin(), histcounts.end());
      sort(tr.begin(), tr.end(), compare_cluster);
      for(int i = 0 ; i< tr.size(); ++i) {
         if(tr[i].second > minPointsNum) {
            cluster_index.push_back(tr[i].first);
         }
      }

      return true;
   }

   void setDeltaAzimuth(float newDeltaAzimuth) {
      deltaA = newDeltaAzimuth;
   }

   void setDeltaRange(float newDeltaRange) {
      deltaR = newDeltaRange;
   }

   void setDeltaPolar(float newDeltaPolar) {
      deltaP = newDeltaPolar;
   }

   void setMinClusterSize(int minpts) {
      minPointsNum = minpts;
   }

   void setInputCloud(typename pcl::PointCloud<PointT>::Ptr cloud) {
      pointCloudPtr = cloud;
   }


private:
   void mergeClusters(vector<int>& cluster_indices, int idx1, int idx2) {
      for (int i = 0; i < cluster_indices.size(); i++) {
         if (cluster_indices[i] == idx1) {
            cluster_indices[i] = idx2;
         }
      }
   }

   float Polar_angle_cal(float x, float y){
      float temp_tangle = 0;
      if(x == 0 && y ==0){
         temp_tangle = 0;
      } else if(y >= 0){
         temp_tangle = (float)atan2(y, x);
      } else if (y <= 0){
         temp_tangle = (float)atan2(y, x) + 2 * PI_M;
      }
      return temp_tangle;
   }

   void find_neighbors(int polar, int range, int azimuth, vector<int>& neighborindex) {
      for (int tempA = azimuth - 1; tempA <= azimuth + 1; tempA++){
         if (tempA < 0 || tempA > round((maxazimuth - minazimuth) * 180 / PI_M / deltaA)){
            continue;
         }

         for (int tempR = range - 1; tempR <= range + 1; tempR++){
            if (tempR < 0 || tempR > round((maxrange - minrange) / deltaR)){
               continue;
            }

            for (int tempP = polar - 1; tempP <= polar + 1; tempP++){
               int px = tempP;
               if (tempP < 0 ){
                  px = PSize;
               }
               if(tempP > 300){
                  px = 0;
               }
               neighborindex.push_back(tempR + px * (RSize+1) + tempA * (RSize + 1) * (PSize +1));
            }
         }
      }
   }

};
