/* \author Ray Phan
 * \description Implementation file for implementing Euclidean clustering from the quiz
 */

#include "kdtree.h"

void helperProximityFunction(const int id,
                             const std::vector<std::vector<float>>& points,
                             KdTree* tree,
                             std::vector<int>& cluster,
                             std::vector<bool>& processed,
                             const float distanceTol) {
  // Obtain the point for the current id
  const std::vector<float> point = points[id];

  // Mark point as processed
  processed[id] = true;

  // Add point to cluster
  cluster.push_back(id);

  // Find nearby ids
  const std::vector<int> nearby_ids = tree->search(point, distanceTol);

  // Iterate over each nearby id
  for (const int query_id : nearby_ids) {
    // If not processed, process it
    if (!processed[query_id]) {
      helperProximityFunction(query_id, points, tree, cluster, processed,
                              distanceTol);
    }
  }
}

std::vector<std::vector<int>> euclideanCluster(
    const std::vector<std::vector<float>>& points,  // Input points
    KdTree* tree,         // KdTree containing said points
    float distanceTol) {  // distance tolerance

  // TODO: Fill out this function to return list of indices for each cluster
  std::vector<std::vector<int>> clusters;

  // Status vector to tell us which points have been processed
  std::vector<bool> processed(points.size(), false);

  // Iterate through each point
  for (size_t i = 0; i < points.size(); i++) {
    if (processed[i]) { continue; }

    std::vector<int> cluster;  // Create cluster

    // Run proximity function
    helperProximityFunction(i, points, tree, cluster, processed, distanceTol);

    // Add to clusters vector
    clusters.push_back(cluster);
  }

  return clusters;
}
