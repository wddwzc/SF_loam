#ifndef DBSCAN_H
#define DBSCAN_H

#include "extra_tools/debug_utility.h"
#include "extra_tools/nanoflann_pcl.h"
#include <pcl/point_types.h>

#define UN_PROCESSED 0
#define PROCESSING 1
#define PROCESSED 2

inline bool comparePointClusters (const pcl::PointIndices &a, const pcl::PointIndices &b) {
    return (a.indices.size () < b.indices.size ());
}

template <typename PointT>
class DBSCANKdtreeCluster {
public:
    bool use_nano;

    typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
    typedef typename pcl::search::KdTree<PointT>::Ptr KdTreePtr;
    virtual void setInputCloud(PointCloudPtr cloud) {
        input_cloud_ = cloud;
    }

    void setSearchMethod(KdTreePtr tree) {
        search_method_ = tree;
        use_nano = false;
    }

    void setSearchMethodNano(boost::shared_ptr<nanoflann::KdTreeFLANN<PointT>> tree) {
        search_method_nano_ = tree;
        use_nano = true;
    }

    void extract(std::vector<pcl::PointIndices>& cluster_indices) {
        std::vector<int> nn_indices;
        std::vector<float> nn_distances;
        std::vector<bool> is_noise(input_cloud_->points.size(), false);
        std::vector<int> types(input_cloud_->points.size(), UN_PROCESSED);
        for (int i = 0; i < input_cloud_->points.size(); i++) {
            
            // caculate range
            PointT &p1 = input_cloud_->points[i];
            double range1 = sqrt(p1.x * p1.x + p1.y * p1.y + p1.z * p1.z);
            double new_eps_1 = eps_ * range1 / 5.0;
            // double new_eps_1 = eps_;
            
            
            if (types[i] == PROCESSED) {
                continue;
            }
            int nn_size;
            if (use_nano)   nn_size = radiusSearchNano(p1, new_eps_1, nn_indices, nn_distances);
            else    nn_size = radiusSearch(i, new_eps_1, nn_indices, nn_distances);
            // 如果密度过小，暂时先视为噪声
            if (nn_size < minPts_) {
                is_noise[i] = true;
                continue;
            }

            // 大致的聚类过程：从一个seed区域生长，邻域搜索到的

            // 如果密度足够，可以当作核心点，作为seed进行区域生长
            std::vector<int> seed_queue;
            seed_queue.push_back(i);
            types[i] = PROCESSED;
            
            // seed的邻域加入队列
            for (int j = 0; j < nn_size; j++) {
                if (nn_indices[j] != i) {
                    seed_queue.push_back(nn_indices[j]);
                    types[nn_indices[j]] = PROCESSING;
                }
            } // for every point near the chosen core point.

            // 相当于一直向外扩展，直到找不到密度相连区域
            int sq_idx = 1;
            while (sq_idx < seed_queue.size()) {
                int cloud_index = seed_queue[sq_idx];


                // caculate range
                PointT &p2 = input_cloud_->points[cloud_index];
                double range2 = sqrt(p2.x * p2.x + p2.y * p2.y + p2.z * p2.z);
                double new_eps_2 = eps_ * range2 / 5.0;
                // double new_eps_2 = eps_;



                // 如果是噪声或者已处理过的，直接跳过
                if (is_noise[cloud_index] || types[cloud_index] == PROCESSED) {
                    // seed_queue.push_back(cloud_index);
                    types[cloud_index] = PROCESSED;
                    sq_idx++;
                    continue; // no need to check neighbors.
                }

                // 如果是 un_process 或 processing，才搜索邻域
                if (use_nano)   nn_size = radiusSearchNano(p2, new_eps_2, nn_indices, nn_distances);
                else    nn_size = radiusSearch(cloud_index, new_eps_2, nn_indices, nn_distances);
                // 发现了新的seed，把它的邻域加入序列
                if (nn_size >= minPts_) {
                    for (int j = 0; j < nn_size; j++) {
                        // 还没处理的加入序列
                        if (types[nn_indices[j]] == UN_PROCESSED) {
                            
                            seed_queue.push_back(nn_indices[j]);
                            types[nn_indices[j]] = PROCESSING;
                        }
                    }
                }
                
                types[cloud_index] = PROCESSED;
                sq_idx++;
            }

            if (seed_queue.size() >= min_pts_per_cluster_ && seed_queue.size () <= max_pts_per_cluster_) {
                pcl::PointIndices r;
                r.indices.resize(seed_queue.size());
                for (int j = 0; j < seed_queue.size(); ++j) {
                    r.indices[j] = seed_queue[j];
                }
                // These two lines should not be needed: (can anyone confirm?) -FF
                std::sort (r.indices.begin (), r.indices.end ());
                r.indices.erase (std::unique (r.indices.begin (), r.indices.end ()), r.indices.end ());

                r.header = input_cloud_->header;
                cluster_indices.push_back (r);   // We could avoid a copy by working directly in the vector
            }
        } // for every point in input cloud
        std::sort (cluster_indices.rbegin (), cluster_indices.rend (), comparePointClusters);
    }

    void setClusterTolerance(double tolerance) {
        eps_ = tolerance;
    }

    void setMinClusterSize (int min_cluster_size) { 
        min_pts_per_cluster_ = min_cluster_size; 
    }

    void setMaxClusterSize (int max_cluster_size) { 
        max_pts_per_cluster_ = max_cluster_size; 
    }
    
    void setCorePointMinPts(int core_point_min_pts) {
        minPts_ = core_point_min_pts;
    }

protected:
    PointCloudPtr input_cloud_;
    
    double eps_ {0.0};
    int minPts_ {1}; // not including the point itself.
    int min_pts_per_cluster_ {1};
    int max_pts_per_cluster_ {std::numeric_limits<int>::max()};

    KdTreePtr search_method_;
    boost::shared_ptr<nanoflann::KdTreeFLANN<PointT>> search_method_nano_;

    // KD-TREE搜索，log(n)
    virtual int radiusSearch (
        int index, double radius, std::vector<int> &k_indices,
        std::vector<float> &k_sqr_distances) const 
    {
        return this->search_method_->radiusSearch(index, radius, k_indices, k_sqr_distances);
    }

    virtual int radiusSearchNano(PointT &point, double radius, std::vector<int> &k_indices,
                                std::vector<float> &k_sqr_distances)
    {
        return this->search_method_nano_->radiusSearch(point, radius, k_indices, k_sqr_distances);
    }

}; // class DBSCANCluster

#endif // DBSCAN_H