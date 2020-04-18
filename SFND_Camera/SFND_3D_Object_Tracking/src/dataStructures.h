
#ifndef dataStructures_h
#define dataStructures_h

#include <map>
#include <opencv2/core.hpp>
#include <vector>

struct LidarPoint {   // single lidar point in space
  double x, y, z, r;  // x,y,z in [m], r is point reflectivity
};

struct
    BoundingBox {  // bounding box around a classified object (contains both 2D and 3D data)

  int boxID;  // unique identifier for this bounding box
  int trackID;  // unique identifier for the track to which this bounding box belongs

  cv::Rect roi;       // 2D region-of-interest in image coordinates
  int classID;        // ID based on class file provided to YOLO framework
  double confidence;  // classification trust

  // Lidar 3D points which project into 2D image roi
  std::vector<LidarPoint> lidarPoints;
  std::vector<cv::KeyPoint> keypoints;  // keypoints enclosed by 2D roi
  std::vector<cv::DMatch> kptMatches;   // keypoint matches enclosed by 2D roi
};

// represents the available sensor information at the same time instance
struct DataFrame {

  cv::Mat cameraImg;  // camera image

  std::vector<cv::KeyPoint> keypoints;  // 2D keypoints within camera image
  cv::Mat descriptors;                  // keypoint descriptors
  // keypoint matches between previous and current frame
  std::vector<cv::DMatch> kptMatches;
  std::vector<LidarPoint> lidarPoints;

  // ROI around detected objects in 2D image coordinates
  std::vector<BoundingBox> boundingBoxes;
  // bounding box matches between previous and current frame
  std::map<int, int> bbMatches;
};

#endif /* dataStructures_h */
