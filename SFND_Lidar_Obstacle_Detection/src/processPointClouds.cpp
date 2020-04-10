// PCL lib Functions for processing point clouds

#include "processPointClouds.h"

#include <unordered_set>

#include "quiz/cluster/euclidean_cluster.h"
#include "quiz/ransac/ransac3d.h"

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(
    typename pcl::PointCloud<PointT>::Ptr cloud) {
  std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud,
    float filterRes,
    Eigen::Vector4f minPoint,
    Eigen::Vector4f maxPoint) {
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  // TODO:: Fill in the function to do voxel grid point reduction and region
  // based filtering
  typename pcl::PointCloud<PointT>::Ptr voxel_grid_filtered{
      new pcl::PointCloud<PointT>};
  // Step #1 - VoxelGrid quantization
  typename pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(filterRes, filterRes, filterRes);  // Units in metres
  vg.filter(*voxel_grid_filtered);

  // Step #2 - Crop to the region of interest
  typename pcl::PointCloud<PointT>::Ptr voxel_grid_crop{
      new pcl::PointCloud<PointT>};
  typename pcl::CropBox<PointT> crop(true);
  crop.setMin(minPoint);
  crop.setMax(maxPoint);
  crop.setInputCloud(voxel_grid_filtered);
  crop.filter(*voxel_grid_crop);

  // Optional - remove the roof pixels
  // Make a new crop object and obtain the indices of where to remove
  std::vector<int> indices_to_remove_roof;
  const Eigen::Vector4f min_point_roof(-2, -2, -1, 1);
  const Eigen::Vector4f max_point_roof(3, 2, 1, 1);
  typename pcl::CropBox<PointT> crop_roof(true);
  crop_roof.setMin(min_point_roof);
  crop_roof.setMax(max_point_roof);
  crop_roof.setInputCloud(voxel_grid_crop);
  crop_roof.filter(indices_to_remove_roof);

  // Remove the roof points
  // First create an indices object that contains the indices we want to remove
  pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
  for (const int idx : indices_to_remove_roof) {
    inliers->indices.push_back(idx);
  }

  // Next, create an extraction tool that will remove the indices
  // Set the negative flag to true to remove these indices
  typename pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(voxel_grid_crop);
  extract.setIndices(inliers);
  extract.setNegative(true);  // Inliers will now remove the points instead
  extract.filter(*voxel_grid_crop);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds"
            << std::endl;

  return voxel_grid_crop;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
    ProcessPointClouds<PointT>::SeparateClouds(
        pcl::PointIndices::Ptr inliers,
        typename pcl::PointCloud<PointT>::Ptr cloud) {
  // TODO: Create two new point clouds, one cloud with obstacles and other with
  // segmented plane

  // From the PCL tutorial
  // Create the filtering object
  pcl::ExtractIndices<PointT> extract;

  // Two point clouds - Obstacles (cars) and the road
  typename pcl::PointCloud<PointT>::Ptr obstacle_point_cloud{
      new pcl::PointCloud<PointT>};
  typename pcl::PointCloud<PointT>::Ptr road_point_cloud{
      new pcl::PointCloud<PointT>};

  // Set up the point cloud for the filtering object
  extract.setInputCloud(cloud);

  // Set up the indices
  extract.setIndices(inliers);

  // Extract the inliers
  extract.setNegative(false);  // False means points belonging to inliers remain
                               // inliers are the points belonging to the road
  extract.filter(*road_point_cloud);

  // Extract the outliers
  extract.setNegative(true);  // True means points belonging to outliers remain
      // outliers are the points not belonging to the road
  extract.filter(*obstacle_point_cloud);

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
      segResult(obstacle_point_cloud, road_point_cloud);
  return segResult;
}

// Modified to include an optional flag so that we can switch between
// using PCL RANSAC implementation or custom implementation from the quiz
template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
    ProcessPointClouds<PointT>::SegmentPlane(
        typename pcl::PointCloud<PointT>::Ptr cloud,
        int maxIterations,
        float distanceThreshold,
        const bool useCustom) {
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  // TODO:: Fill in this function to find inliers for the cloud.

  // From the PCL tutorial
  // http://pointclouds.org/documentation/tutorials/extract_indices.php#extract-indices
  pcl::ModelCoefficients::Ptr coefficients{new pcl::ModelCoefficients};
  pcl::PointIndices::Ptr inliers{new pcl::PointIndices};

  if (!useCustom) {
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;

    // Set up the options
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);  // Set up the point cloud

    // Perform RANSAC to find the plane for the point cloud (i.e. the road)
    seg.segment(*inliers, *coefficients);
  } else {  // Using custom implementation made for the quiz
    std::unordered_set<int> inliers_set =
        RANSAC3D<PointT>::Ransac3D(cloud, maxIterations, distanceThreshold);
    for (const int index : inliers_set) {
      inliers->indices.push_back(index);
    }
  }

  // Report if there are no inliers
  if (inliers->indices.size() == 0) {
    std::cerr << "There were no inliers with this point cloud - plane "
                 "extraction is not possible"
              << std::endl;
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count()
            << " milliseconds" << std::endl;

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
      segResult = SeparateClouds(inliers, cloud);
  return segResult;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
    ProcessPointClouds<PointT>::Clustering(
        typename pcl::PointCloud<PointT>::Ptr cloud,
        float clusterTolerance,
        int minSize,
        int maxSize,
        const bool useCustom) {
  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  // TODO:: Fill in the function to perform euclidean clustering to group
  // detected obstacles

  if (!useCustom) {  // Use PCL method
    // From PCL tutorial:
    // http://pointclouds.org/documentation/tutorials/cluster_extraction.php
    // Creating the KdTree object for the search method of the extraction

    typename pcl::search::KdTree<PointT>::Ptr tree(
        new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    // Contains cluster indices
    std::vector<pcl::PointIndices> cluster_indices;

    // Euclidean distance object to perform clustering
    pcl::EuclideanClusterExtraction<PointT> ec;

    // Set up parameters then cluster
    ec.setClusterTolerance(clusterTolerance);  // in cm
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // Go through each set of indices...
    for (const auto& clust : cluster_indices) {
      // Create new pointer to point cloud object of type PointT
      typename pcl::PointCloud<PointT>::Ptr cloud_cluster{
          new pcl::PointCloud<PointT>};

      // For each index, access the point in the point cloud and
      // and add to the list of points
      for (const auto index : clust.indices) {
        cloud_cluster->points.push_back(cloud->points[index]);
      }
      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      // Add to the final output
      clusters.push_back(cloud_cluster);
    }
  } else {  // Use custom implementation
    // Create KdTree object
    KdTree* tree = new KdTree;

    // Create storage to be compatible with custom Euclidean
    // clustering method
    std::vector<std::vector<float>> points_as_vec;

    // Insert points into KdTree - also retain in custom storage
    for (int i = 0; i < cloud->points.size(); i++) {
      const std::vector<float> pt = {cloud->points[i].x, cloud->points[i].y,
                                     cloud->points[i].z};
      tree->insert(pt, i);
      points_as_vec.push_back(pt);
    }

    // Get the cluster IDs
    const std::vector<std::vector<int>> clusters_from_kdtree =
        euclideanCluster(points_as_vec, tree, clusterTolerance);

    // Free the tree as we don't need it anymore
    delete tree;
    tree = nullptr;

    // Now we must additionally loop through each cluster and ensure that the size of the clusters
    // is within the min-max range - if it is, then finally add these points in
    for (const auto& clust : clusters_from_kdtree) {
      if (clust.size() >= minSize && clust.size() <= maxSize) {
        // Create new pointer to point cloud object of type PointT
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster{
            new pcl::PointCloud<PointT>};

        // For each index, access the point in the point cloud and
        // and add to the list of points
        for (const auto index : clust) {
          cloud_cluster->points.push_back(cloud->points[index]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // Add to the final output
        clusters.push_back(cloud_cluster);
      }
    }
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "clustering took " << elapsedTime.count()
            << " milliseconds and found " << clusters.size() << " clusters"
            << std::endl;

  return clusters;
}

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(
    typename pcl::PointCloud<PointT>::Ptr cluster) {
  // Find bounding box for one of the clusters
  PointT minPoint, maxPoint;
  pcl::getMinMax3D(*cluster, minPoint, maxPoint);

  Box box;
  box.x_min = minPoint.x;
  box.y_min = minPoint.y;
  box.z_min = minPoint.z;
  box.x_max = maxPoint.x;
  box.y_max = maxPoint.y;
  box.z_max = maxPoint.z;

  return box;
}

// Added - Return a BoxQ object so that it contains the
// rotated version of the bounding box to tightly wrap around the
// point cloud
template <typename PointT>
BoxQ ProcessPointClouds<PointT>::RotatedBoundingBox(
    typename pcl::PointCloud<PointT>::Ptr cluster) {
  // Process is from:
  // http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html
  BoxQ box;

  // Step #1 - Use PCA and get the components
  typename pcl::PointCloud<PointT>::Ptr cloudPCAprojection(
      new pcl::PointCloud<PointT>);
  pcl::PCA<PointT> pca;
  pca.setInputCloud(cluster);
  pca.project(*cluster, *cloudPCAprojection);

  // Step #2
  // Transform the original cloud to the origin where the principal components correspond to the axes.
  Eigen::Vector4f pcaCentroid;
  pcl::compute3DCentroid(*cluster, pcaCentroid);
  Eigen::Matrix3f eigenVectorsPCA = pca.getEigenVectors();
  Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
  projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
  projectionTransform.block<3, 1>(0, 3) =
      -1.f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
  typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected(
      new pcl::PointCloud<PointT>);
  pcl::transformPointCloud(*cluster, *cloudPointsProjected,
                           projectionTransform);
  // Get the minimum and maximum points of the transformed cloud.
  PointT minPoint, maxPoint;
  pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
  const Eigen::Vector3f meanDiagonal =
      0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());

  // Step #3 - Final Transform
  const Eigen::Quaternionf bboxQuaternion(
      eigenVectorsPCA);  //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
  const Eigen::Vector3f bboxTransform =
      eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

  // Set parameters and return
  box.bboxTransform = bboxTransform;
  box.bboxQuaternion = bboxQuaternion;
  box.cube_length = maxPoint.x - minPoint.x;
  box.cube_width = maxPoint.y - minPoint.y;
  box.cube_height = maxPoint.z - minPoint.z;
  return box;
}

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(
    typename pcl::PointCloud<PointT>::Ptr cloud, std::string file) {
  pcl::io::savePCDFileASCII(file, *cloud);
  std::cerr << "Saved " << cloud->points.size() << " data points to " + file
            << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(
    std::string file) {
  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1)  //* load the file
  {
    PCL_ERROR("Couldn't read file \n");
  }
  std::cerr << "Loaded " << cloud->points.size() << " data points from " + file
            << std::endl;

  return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(
    std::string dataPath) {
  std::vector<boost::filesystem::path> paths(
      boost::filesystem::directory_iterator{dataPath},
      boost::filesystem::directory_iterator{});

  // sort files in accending order so playback is chronological
  sort(paths.begin(), paths.end());

  return paths;
}