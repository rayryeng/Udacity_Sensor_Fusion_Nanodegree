
#include <algorithm>
#include <iostream>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <set>  // Added by Ray

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;

namespace {
// Computes the median of a double vector
template <typename T>
T median(const std::vector<T>& vec) {
  if (vec.empty()) { return 0.0; }
  std::vector<T> vec_copy = vec;
  std::sort(vec_copy.begin(), vec_copy.end());
  size_t index = vec_copy.size() / 2;
  T med = vec_copy[index];
  if (vec.size() % 2 == 0) { med = (med + vec_copy[index - 1]) / 2; }
  return med;
}

// Calculates outlier interquartile range (IQR) thresholds
// Returns a pair where the first element is the lower
// threshold and the second element is the upper threshold
// The IQR is the difference between the 75th (Q3) and 25th (Q1)
// percentile.  An outlier is when a value is less than Q1 - 1.5*IQR
// and greater than Q3 + 1.5*IQR
template <typename T>
std::pair<T, T> get_iqr_thresholds(const std::vector<T>& vec) {
  // 0 1 2 3 4 5 6
  //       | <-- odd
  // 7 / 2 = 3 --> [0,3), [4,7)
  // 0 1 2 3 4 5
  // 6 / 3 = 2 -- [0,3), [3,6)
  // Determine where split points are
  const size_t left_split_point = vec.size() / 2;
  const size_t right_split_point =
      vec.size() % 2 == 0 ? left_split_point : left_split_point + 1;

  // Sort the vector
  std::vector<T> vec_copy = vec;
  std::sort(vec_copy.begin(), vec_copy.end());

  // Split into left and right halves
  std::vector<T> lower_half(vec_copy.begin(),
                            vec_copy.begin() + left_split_point);
  std::vector<T> upper_half(vec_copy.begin() + right_split_point,
                            vec_copy.end());

  // Get medians for both to get Q1 and Q3
  const T q1 = median(lower_half);
  const T q3 = median(upper_half);

  // Calculate IQR and return thresholds
  const T iqr = q3 - q1;
  return make_pair(q1 - 1.5 * iqr, q3 + 1.5 * iqr);
}

// Check for outliers
template <typename T>
bool check_for_outlier(const std::vector<T>& vec, const T val) {
  pair<T, T> thresholds = get_iqr_thresholds(vec);
  return val < thresholds.first || val > thresholds.second;
}

// Check for outliers after the thresholds are computed
template <typename T>
bool check_for_outlier(const pair<T, T>& thresholds, const T val) {
  return val < thresholds.first || val > thresholds.second;
}

// Remove outliers
template <typename T>
void remove_outliers(std::vector<T>& vec) {
  pair<T, T> thresholds = get_iqr_thresholds(vec);
  vec.erase(remove_if(vec.begin(), vec.end(),
                      [&thresholds](const T& val) {
                        return val < thresholds.first ||
                               val > thresholds.second;
                      }),
            vec.end());
}
}  // namespace

// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox>& boundingBoxes,
                         std::vector<LidarPoint>& lidarPoints,
                         float shrinkFactor,
                         cv::Mat& P_rect_xx,
                         cv::Mat& R_rect_xx,
                         cv::Mat& RT) {
  // loop over all Lidar points and associate them to a 2D bounding box
  cv::Mat X(4, 1, cv::DataType<double>::type);
  cv::Mat Y(3, 1, cv::DataType<double>::type);

  for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1) {
    // assemble vector for matrix-vector-multiplication
    X.at<double>(0, 0) = it1->x;
    X.at<double>(1, 0) = it1->y;
    X.at<double>(2, 0) = it1->z;
    X.at<double>(3, 0) = 1;

    // project Lidar point into camera
    Y = P_rect_xx * R_rect_xx * RT * X;
    cv::Point pt;
    pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2);  // pixel coordinates
    pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

    vector<vector<BoundingBox>::iterator>
        enclosingBoxes;  // pointers to all bounding boxes which enclose the current Lidar point
    for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin();
         it2 != boundingBoxes.end(); ++it2) {
      // shrink current bounding box slightly to avoid having too many outlier points around the edges
      cv::Rect smallerBox;
      smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
      smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
      smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
      smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

      // check wether point is within current bounding box
      if (smallerBox.contains(pt)) { enclosingBoxes.push_back(it2); }

    }  // eof loop over all bounding boxes

    // check wether point has been enclosed by one or by multiple boxes
    if (enclosingBoxes.size() == 1) {
      // add Lidar point to bounding box
      enclosingBoxes[0]->lidarPoints.push_back(*it1);
    }

  }  // eof loop over all Lidar points
}

void show3DObjects(std::vector<BoundingBox>& boundingBoxes,
                   cv::Size worldSize,
                   cv::Size imageSize,
                   bool bWait) {
  // create topview image
  cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

  for (auto it1 = boundingBoxes.begin(); it1 != boundingBoxes.end(); ++it1) {
    // create randomized color for current 3D object
    cv::RNG rng(it1->boxID);
    cv::Scalar currColor = cv::Scalar(rng.uniform(0, 150), rng.uniform(0, 150),
                                      rng.uniform(0, 150));

    // plot Lidar points into top view image
    int top = 1e8, left = 1e8, bottom = 0.0, right = 0.0;
    float xwmin = 1e8, ywmin = 1e8, ywmax = -1e8;
    for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end();
         ++it2) {
      // world coordinates
      // world position in m with x facing forward from sensor
      float xw = (*it2).x;
      // world position in m with y facing left from sensor
      float yw = (*it2).y;
      xwmin = xwmin < xw ? xwmin : xw;
      ywmin = ywmin < yw ? ywmin : yw;
      ywmax = ywmax > yw ? ywmax : yw;

      // top-view coordinates
      int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
      int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

      // find enclosing rectangle
      top = top < y ? top : y;
      left = left < x ? left : x;
      bottom = bottom > y ? bottom : y;
      right = right > x ? right : x;

      // draw individual point
      cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
    }

    // draw enclosing rectangle
    cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),
                  cv::Scalar(0, 0, 0), 2);

    // augment object with some key data
    char str1[200], str2[200];
    sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
    putText(topviewImg, str1, cv::Point2f(left - 250, bottom + 50),
            cv::FONT_ITALIC, 2, currColor);
    sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax - ywmin);
    putText(topviewImg, str2, cv::Point2f(left - 250, bottom + 125),
            cv::FONT_ITALIC, 2, currColor);
  }

  // plot distance markers
  float lineSpacing = 2.0;  // gap between distance markers
  int nMarkers = floor(worldSize.height / lineSpacing);
  for (size_t i = 0; i < nMarkers; ++i) {
    int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) +
            imageSize.height;
    cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y),
             cv::Scalar(255, 0, 0));
  }

  // display image
  string windowName = "3D Objects";
  cv::namedWindow(windowName, 1);
  cv::imshow(windowName, topviewImg);

  if (bWait) {
    cv::waitKey(0);  // wait for key to be pressed
  }
}

// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox& boundingBox,
                              std::vector<cv::KeyPoint>& kptsPrev,
                              std::vector<cv::KeyPoint>& kptsCurr,
                              std::vector<cv::DMatch>& kptMatches) {
  // Given the input bounding box, we need to make sure that
  // the matches between the previous and current frames
  // are within the bounding box
  boundingBox.keypoints.clear();
  boundingBox.kptMatches.clear();

  // trainIdx --> current frame
  // queryIdx --> previous frame
  std::vector<cv::KeyPoint> temp_keypoints_prev;
  std::vector<cv::KeyPoint> temp_keypoints_curr;
  std::vector<cv::DMatch> temp_matches;

  // First stage - just check to see if it's within ROI
  for (const auto& dm : kptMatches) {
    const int curr_frame_index = dm.trainIdx;
    const int prev_frame_index = dm.queryIdx;
    const cv::Point2f curr_pt = kptsCurr[curr_frame_index].pt;
    const cv::Point2f prev_pt = kptsPrev[prev_frame_index].pt;
    if (boundingBox.roi.contains(curr_pt) &&
        boundingBox.roi.contains(prev_pt)) {
      temp_keypoints_prev.push_back(kptsCurr[curr_frame_index]);
      temp_keypoints_curr.push_back(kptsPrev[curr_frame_index]);
      temp_matches.push_back(dm);
    }
  }

  // Calculate the average point for both
  cv::Point2f prev_mean_pt(0.0f, 0.0f), curr_mean_pt(0.0f, 0.0f);
  for (size_t i = 0; i < temp_keypoints_prev.size(); i++) {
    prev_mean_pt.x += temp_keypoints_prev[i].pt.x;
    prev_mean_pt.y += temp_keypoints_prev[i].pt.y;
    curr_mean_pt.x += temp_keypoints_curr[i].pt.x;
    curr_mean_pt.y += temp_keypoints_curr[i].pt.y;
  }
  prev_mean_pt.x /= temp_keypoints_prev.size();
  prev_mean_pt.y /= temp_keypoints_prev.size();
  curr_mean_pt.x /= temp_keypoints_curr.size();
  curr_mean_pt.y /= temp_keypoints_curr.size();

  // Find distances from all keypoints to the mean
  std::vector<float> prev_distances;
  std::vector<float> curr_distances;

  for (size_t i = 0; i < temp_keypoints_prev.size(); i++) {
    prev_distances.push_back(
        cv::norm(temp_keypoints_prev[i].pt - prev_mean_pt));
    curr_distances.push_back(
        cv::norm(temp_keypoints_curr[i].pt - curr_mean_pt));
  }

  // Finally, go through each and make sure that they aren't outliers
  // If both of them aren't, finally update the output structure
  const pair<float, float> prev_thresholds = get_iqr_thresholds(prev_distances);
  const pair<float, float> curr_thresholds = get_iqr_thresholds(curr_distances);
  for (size_t i = 0; i < temp_keypoints_prev.size(); i++) {
    if (check_for_outlier(prev_thresholds, prev_distances[i])) { continue; }
    if (check_for_outlier(curr_thresholds, curr_distances[i])) { continue; }
    boundingBox.keypoints.push_back(temp_keypoints_prev[i]);
    boundingBox.kptMatches.push_back(temp_matches[i]);
  }
}

// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint>& kptsPrev,
                      std::vector<cv::KeyPoint>& kptsCurr,
                      std::vector<cv::DMatch> kptMatches,
                      double frameRate,
                      double& TTC,
                      cv::Mat* visImg) {
  // Taken from TTC_camera project
  // DMatch struct
  // --> trainIdx: index of matched keypoint in curr. frame
  // --> queryIdx: index of matched keypoint in prev. frame
  // compute distance ratios between all matched keypoints
  // stores the distance ratios for all keypoints between curr. and prev. frame
  vector<double> distRatios;
  // outer kpt. loop
  for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1) {

    // get current keypoint and its matched partner in the prev. frame
    cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
    cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

    // inner kpt.-loop
    for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2) {
      // get next keypoint and its matched partner in the prev. frame
      cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
      cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

      // compute distances and distance ratios
      double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
      double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

      // avoid division by zero
      if (distPrev > std::numeric_limits<double>::epsilon()) {
        double distRatio = distCurr / distPrev;
        distRatios.push_back(distRatio);
      }
    }  // eof inner loop over all matched kpts
  }    // eof outer loop over all matched kpts

  // only continue if list of distance ratios is not empty
  if (distRatios.size() == 0) { TTC = NAN; }

  // Remove all outliers
  remove_outliers(distRatios);

  // Check again for not empty
  if (distRatios.size() == 0) { TTC = NAN; }

  // After removing outliers, find the median ratio
  const float median_dist_ratio = median(distRatios);

  // Now calculate TTC
  const double dT = 1 / frameRate;
  TTC = -dT / (1.0f - median_dist_ratio);
}

void computeTTCLidar(std::vector<LidarPoint>& lidarPointsPrev,
                     std::vector<LidarPoint>& lidarPointsCurr,
                     double frameRate,
                     double& TTC) {
  // auxiliary variables
  // time between two measurements in seconds
  const double dT = 1.0 / frameRate;
  const double laneWidth = 4.0;  // assumed width of the ego lane

  // find closest distance to Lidar points within ego lane
  double minXPrev, minXCurr;

  // Make copies of the LiDAR points so we can remove outliers
  // if necessary
  std::vector<LidarPoint> lidar_points_prev = lidarPointsPrev;
  std::vector<LidarPoint> lidar_points_curr = lidarPointsCurr;

  // x - moves forward and backwards - length
  // y - moves side-to-side - width
  // Only consider points that are within the lane
  // Keep looping until we know for sure that points are not outliers
  while (true) {
    if (lidar_points_prev.size() == 0 || lidar_points_curr.size() == 0) {
      break;  // Safeguard
    }
    // Records x coordinate closest to the car
    minXPrev = 1e9;
    minXCurr = 1e9;

    // Helps compute the average xyz coordinate for the point cloud
    double mean_x = 0.0, mean_y = 0.0, mean_z = 0.0;

    // Remembers which index to the point that is the closest
    int prev_index = -1, curr_index = -1;

    // Previous frame
    // Cycle through and figure out which is the closest point laterally
    // to the ego vehicle - also sum up all of the coordinates to compute mean
    int i = 0;
    for (auto it = lidar_points_prev.begin(); it != lidar_points_prev.end();
         ++it) {
      if (std::abs(it->y) > laneWidth / 2.0) { continue; }
      if (it->x < minXPrev) {
        minXPrev = it->x;
        prev_index = i;
      }
      mean_x += it->x;
      mean_y += it->y;
      mean_z += it->z;
      i++;
    }
    mean_x /= lidar_points_prev.size();
    mean_y /= lidar_points_prev.size();
    mean_z /= lidar_points_prev.size();

    // Find distances to the centroid
    std::vector<double> distances;
    for (const auto& it : lidar_points_prev) {
      distances.push_back(std::sqrt((it.x - mean_x) * (it.x - mean_x) +
                                    (it.y - mean_y) * (it.y - mean_y) +
                                    (it.z - mean_z) * (it.z - mean_z)));
    }

    // Check to see if current point is an outlier
    if (check_for_outlier(distances, distances[prev_index])) {
      // If it is, remove this point from the point cloud and try again
      lidar_points_prev.erase(lidar_points_prev.begin() + prev_index);
      continue;
    }

    // Repeat for the current frame
    i = 0;
    mean_x = 0.0;
    mean_y = 0.0;
    mean_z = 0.0;
    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it) {
      if (std::abs(it->y) > laneWidth / 2.0) { continue; }
      if (it->x < minXCurr) {
        minXCurr = it->x;
        curr_index = i;
      }
      mean_x += it->x;
      mean_y += it->y;
      mean_z += it->z;
    }
    mean_x /= lidar_points_curr.size();
    mean_y /= lidar_points_curr.size();
    mean_z /= lidar_points_curr.size();

    // Find distances to the centroid
    distances.clear();
    for (const auto& it : lidar_points_curr) {
      distances.push_back(std::sqrt((it.x - mean_x) * (it.x - mean_x) +
                                    (it.y - mean_y) * (it.y - mean_y) +
                                    (it.z - mean_z) * (it.z - mean_z)));
    }

    // Check to see if current point is an outlier
    if (check_for_outlier(distances, distances[curr_index])) {
      // If it is, remove this point from the point cloud and try again
      lidar_points_curr.erase(lidar_points_curr.begin() + curr_index);
      continue;
    }
    break;
  }
  // compute TTC from both measurements
  if (lidar_points_prev.size() == 0 || lidar_points_curr.size() == 0) {
    TTC = NAN;
  } else {
    TTC = minXCurr * dT / (minXPrev - minXCurr);
  }
}

void matchBoundingBoxes(std::vector<cv::DMatch>& matches,
                        std::map<int, int>& bbBestMatches,
                        DataFrame& prevFrame,
                        DataFrame& currFrame) {
  // trainIdx --> current frame
  // queryIdx --> previous frame
  // Goal - Figure out which bounding box ID from the previous frame matches the current frame
  //        given that most of the keypoints overlap between the two bounding boxes

  // Algorithm
  // Store a map where the key is a pair of indices (boxID_prev, boxID_curr)
  // and the value is the number of times we see this pair existing
  // For each match
  //    Grab trainIdx (current frame index) and queryIdx (previous frame index)
  //    Use trainIdx to access currFrame.keypoints[trainIdx].pt keypoint
  //    Use queryIdx to access prevFrame.keypoints[queryIdx].pt keypoint
  //    Figure out which bounding box each keypoint belongs to
  //    for the current frame keypoint using currFrame and the previous
  //    frame keypoint using prevFrame
  //    Log the previous frame and current frame bounding box ID and increment
  //    into the map.

  // Stores counts of overlapping matches between previous and current frame
  std::map<pair<int, int>, int> counts;

  for (const auto& dm : matches) {
    // Obtain feature points for previous and current frame
    const int curr_idx = dm.trainIdx;
    const int prev_idx = dm.queryIdx;
    const cv::Point2f curr_pt = currFrame.keypoints[curr_idx].pt;
    const cv::Point2f prev_pt = prevFrame.keypoints[prev_idx].pt;

    // For each type of point (previous and current), figure out
    // which bounding boxes from their respective frames contains
    // this point
    for (const BoundingBox& prev_bbox : prevFrame.boundingBoxes) {
      for (const BoundingBox& curr_bbox : currFrame.boundingBoxes) {
        // Did we find which point each bounding box is contained in?
        if (prev_bbox.roi.contains(prev_pt) &&
            curr_bbox.roi.contains(curr_pt)) {
          const int prev_bbox_id = prev_bbox.boxID;
          const int curr_bbox_id = curr_bbox.boxID;
          // See if we've encountered this combination before
          std::map<std::pair<int, int>, int>::iterator it =
              counts.find(std::make_pair(prev_bbox_id, curr_bbox_id));
          if (it == counts.end()) {
            // If we haven't, make a new entry to start counting
            counts.insert(std::pair<std::pair<int, int>, int>(
                std::make_pair(prev_bbox_id, curr_bbox_id), 1));
          } else {
            // If yes, update the entry
            const int count = it->second;
            it->second = count + 1;
          }
        }
      }
    }
  }

  // Collect all unique bounding box IDs from the previous frame
  set<int> prev_ids;
  for (const auto& it : counts) {
    prev_ids.insert(it.first.first);
  }

  // Now loop over all possible bounding box IDs from the previous frame,
  // see how many combinations of matches to the current frame we have
  // then choose the largest one
  for (const int box_id : prev_ids) {
    // Records largest amount of matches for a previous box ID
    // and current box ID pair
    int max_count = -1;

    // Stores the current bounding box ID with the largest matches
    int curr_box_id = -1;

    // Go through each pair of bounding boxes with
    // matches - make sure we concentrate on the one
    // where the previous bounding box ID matches what
    // we're currently iterating over
    for (const auto& it : counts) {
      // If this isn't the previous bounding box ID
      // we're concentrating on, skip
      if (it.first.first != box_id) { continue; }

      // If this prev., curr. pair has a count
      // that exceeds the max...
      if (it.second > max_count) {
        // Record it and the current bounding box
        // ID that it's connected to
        max_count = it.second;
        curr_box_id = it.first.second;
      }
    }

    // If there are no matches from the previous frame
    // to any boxes to the current frame, skip
    if (curr_box_id == -1) { continue; }
    bbBestMatches.insert(std::pair<int, int>(box_id, curr_box_id));
  }
}