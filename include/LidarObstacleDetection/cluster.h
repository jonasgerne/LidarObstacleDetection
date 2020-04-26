/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include <chrono>
#include <string>
#include <vector>
#include "LidarObstacleDetection/kdtree.h"

template <typename PointT>
void proximityHelper(int idx, const typename pcl::PointCloud<PointT>::Ptr points, std::vector<int>& cluster, KdTree<PointT> *tree, float distanceTol, std::vector<bool> &processed){
    // mark point as processed
    processed[idx] = true;
    // add point to cluster
    cluster.emplace_back(idx);

    // search nearby points
    std::vector<int> neighbors = tree->search((*points)[idx], distanceTol);

    for (auto nidx : neighbors) {
        if(processed[nidx] == false)
            proximityHelper(nidx, points, cluster, tree, distanceTol, processed);
    }
}

template <typename PointT>
std::vector<std::vector<int>>
euclideanCluster(const typename pcl::PointCloud<PointT>::Ptr points, KdTree<PointT> *tree, float distanceTol, int minSize = 0) {
    std::vector<bool> processed(points->size(), false);
    std::vector<std::vector<int>> cluster_indices;
    for (int i = 0; i < points->size(); ++i) {
        if(!processed[i]){
            std::vector<int> cluster;
            proximityHelper(i, points, cluster, tree, distanceTol, processed);
            if (cluster.size() >= minSize)
                cluster_indices.emplace_back(cluster);
        }
    }
    return cluster_indices;
}
