/* \author Ray Phan
 * \description Header for implementing Euclidean clustering from the quiz
 */

#ifndef __EUCLIDEAN_CLUSTER_H__
#define __EUCLIDEAN_CLUSTER_H__
#include "kdtree.h"

void helperProximityFunction(const int id,
                             const std::vector<std::vector<float>>& points,
                             KdTree* tree,
                             std::vector<int>& cluster,
                             std::vector<bool>& processed,
                             const float distanceTol);

std::vector<std::vector<int>> euclideanCluster(
    const std::vector<std::vector<float>>& points,  // Input points
    KdTree* tree,        // KdTree containing said points
    float distanceTol);  // distance tolerance
#endif