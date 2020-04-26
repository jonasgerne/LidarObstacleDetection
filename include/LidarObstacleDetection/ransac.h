#include <unordered_set>

template<typename PointT>
std::unordered_set<int> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol) {
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // For max iterations
    for (int i = 0; i < maxIterations; ++i) {
        std::unordered_set<int> inliers;

        // Randomly sample subset and fit line
        while (inliers.size() < 3)
            inliers.insert(rand() % (cloud->points.size()));

        auto itr = inliers.begin();
        const auto& point_1 = cloud->points[*itr].getVector3fMap();
        itr++;
        const auto& point_2 = cloud->points[*itr].getVector3fMap();
        itr++;
        const auto& point_3 = cloud->points[*itr].getVector3fMap();

        const Eigen::Vector3f plane_coeffs = (point_2 - point_1).cross(point_3 - point_1).normalized();

        float coeff_d = -(plane_coeffs.dot(point_1));

        // Measure distance between every point and fitted line
        for (int idx = 0; idx < cloud->points.size(); ++idx) {
            if (inliers.count(idx) > 0)
                continue;

            const auto &point = cloud->points[idx].getVector3fMap();
            // denominator always 1.0 as vector was normalized
            float d = fabs(plane_coeffs.dot(point) + coeff_d);
            // If distance is smaller than threshold count it as inlier
            if (d <= distanceTol)
                inliers.insert(idx);
        }
        if (inliers.size() > inliersResult.size())
            inliersResult = inliers;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RANSAC took " << elapsedTime.count() << " milliseconds" << std::endl;
    // Return indicies of inliers from fitted line with most inliers
    return inliersResult;
}
