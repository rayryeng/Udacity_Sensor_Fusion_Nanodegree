/* \author Ray Phan
 * \description Header for quiz answer to 3D RANSAC for detecting the ground
 * plane
 */
#ifndef _RANSAC_3D_H_
#define _RANSAC_3D_H_

#include <pcl/common/common.h>
#include <unordered_set>

// Practice writing a templated class
template <typename PointT>
class RANSAC3D {
 public:
  static std::unordered_set<int> Ransac3D(
      typename pcl::PointCloud<PointT>::Ptr cloud,
      const int maxIterations,
      const float distanceTol) {
    std::unordered_set<int> inlier_results;
    srand(time(NULL));

    // TODO: Fill in this function
    std::unordered_set<int> proposed_inliers;

    // Number of points
    const int num_points = cloud->points.size();

    // Repeat for a maximum number of iterations
    for (int i = 0; i < maxIterations; i++) {
      proposed_inliers.clear();
      // Obtain the indices to sample
      std::unordered_set<int> indices_to_sample;
      indices_to_sample.insert(std::rand() % num_points);
      indices_to_sample.insert(std::rand() % num_points);
      indices_to_sample.insert(std::rand() % num_points);

      // If we don't have them all unique, try again
      if (indices_to_sample.size() != 3) {
        i--;
        continue;
      }

      // Obtain the points
      std::unordered_set<int>::iterator it = indices_to_sample.begin();
      const auto& pt1 = cloud->points[*it++];
      const auto& pt2 = cloud->points[*it++];
      const auto& pt3 = cloud->points[*it];

      // Calculate the coefficients for the plane
      // A = (y2 - y1)(z3 - z1) - (z2 - z1)(y3 - y1)
      const double A =
          (pt2.y - pt1.y) * (pt3.z - pt1.z) - (pt2.z - pt1.z) * (pt3.y - pt1.y);
      // B = (z2 - z1)(x3 - x1) - (x2 - x1)(z3 - z1)
      const double B =
          (pt2.z - pt1.z) * (pt3.x - pt1.x) - (pt2.x - pt1.x) * (pt3.z - pt1.z);
      // C = (x2 − x1)(y3 − y1) − (y2 − y1)(x3 − x1)
      const double C =
          (pt2.x - pt1.x) * (pt3.y - pt1.y) - (pt2.y - pt1.y) * (pt3.x - pt1.x);
      // D = -(A*x1 + B*y1 + C*z1)
      const double D = -(A * pt1.x + B * pt1.y + C * pt1.z);

      // Find inliers
      for (int j = 0; j < num_points; j++) {
        const auto& pt = cloud->points[j];
        const double dist = std::abs(A * pt.x + B * pt.y + C * pt.z + D) /
                            std::sqrt(A * A + B * B + C * C);
        if (dist <= distanceTol) { proposed_inliers.insert(j); }
      }

      // Retain the best inliers
      if (proposed_inliers.size() > inlier_results.size()) {
        inlier_results = proposed_inliers;
      }
    }

    return inlier_results;
  };
};
#endif