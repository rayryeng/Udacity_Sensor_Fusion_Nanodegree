// PCL lib Functions for processing point clouds

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/common/common.h>
#include <pcl/common/pca.h>         // For PCA
#include <pcl/common/transforms.h>  // For PCA
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <chrono>
#include <ctime>
#include <iostream>
#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include "render/box.h"

template <typename PointT>
class ProcessPointClouds {
 public:
  // constructor
  ProcessPointClouds() = default;
  // deconstructor
  ~ProcessPointClouds() = default;

  void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

  typename pcl::PointCloud<PointT>::Ptr FilterCloud(
      typename pcl::PointCloud<PointT>::Ptr cloud,
      float filterRes,
      Eigen::Vector4f minPoint,
      Eigen::Vector4f maxPoint);

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
      SeparateClouds(pcl::PointIndices::Ptr inliers,
                     typename pcl::PointCloud<PointT>::Ptr cloud);

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
      SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud,
                   int maxIterations,
                   float distanceThreshold,
                   const bool useCustom = false);

  std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(
      typename pcl::PointCloud<PointT>::Ptr cloud,
      float clusterTolerance,
      int minSize,
      int maxSize,
      const bool useCustom = false);

  Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

  // Added for rotated bounding boxes
  BoxQ RotatedBoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

  void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

  typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

  std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
};
#endif /* PROCESSPOINTCLOUDS_H_ */
