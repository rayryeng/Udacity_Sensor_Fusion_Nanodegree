
#include <algorithm>
#include <iostream>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;

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
    cv::Point2f diff = temp_keypoints_prev[i].pt - prev_mean_pt;
    prev_distances.push_back(cv::sqrt(diff.x * diff.x + diff.y * diff.y));
    diff = temp_keypoints_curr[i].pt - curr_mean_pt;
    curr_distances.push_back(cv::sqrt(diff.x * diff.x + diff.y * diff.y));
  }

  // Find the mean and standard deviation of these distances
  float mean_prev_diff =
      std::accumulate(prev_distances.begin(), prev_distances.end(), 0.0f);
  float mean_curr_diff =
      std::accumulate(curr_distances.begin(), curr_distances.end(), 0.0f);
  float stddev_prev_diff = 0.0, stddev_curr_diff = 0.0;

  for (size_t i = 0; i < prev_distances.size(); i++) {
    stddev_prev_diff += (prev_distances[i] - mean_prev_diff) *
                        (prev_distances[i] - mean_prev_diff);
    stddev_curr_diff += (curr_distances[i] - mean_curr_diff) *
                        (curr_distances[i] - mean_curr_diff);
  }
  stddev_prev_diff = std::sqrt(stddev_prev_diff / (prev_distances.size() - 1));
  stddev_curr_diff = std::sqrt(stddev_curr_diff / (curr_distances.size() - 1));

  // Finally, go through each and make sure that the corresponding distance
  // away from the mean is at least within 2*sigma.  If both of them are,
  // finally update the output structure
  for (size_t i = 0; i < temp_keypoints_prev.size(); i++) {
    if (std::abs(prev_distances[i] - mean_prev_diff) >
        2.0f * stddev_prev_diff) {
      continue;
    }
    if (std::abs(curr_distances[i] - mean_curr_diff) >
        2.0f * stddev_curr_diff) {
      continue;
    }
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
  const double minDist = 20.0;  // min. required distance
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

      if (distPrev > std::numeric_limits<double>::epsilon() &&
          distCurr >= minDist) {  // avoid division by zero
        double distRatio = distCurr / distPrev;
        distRatios.push_back(distRatio);
      }
    }  // eof inner loop over all matched kpts
  }    // eof outer loop over all matched kpts

  // only continue if list of distance ratios is not empty
  if (distRatios.size() == 0) {
    TTC = NAN;
  }

  // Find the median ratio to handle outliers
  // Sort the distances
  std::sort(distRatios.begin(), distRatios.end());

  // Obtain the middle index of the vector
  const size_t index = distRatios.size() / 2;

  // Get the median
  double medianDistRatio = distRatios[index];

  // If the size of the vector is even, access the
  // vector at the index previous to it and average
  if (distRatios.size() % 2 == 0) {
    medianDistRatio = (medianDistRatio + distRatios[index - 1]) / 2.0;
  }

  // Now calculate TTC
  const double dT = 1 / frameRate;
  TTC = -dT / (1 - medianDistRatio);
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

  // This variable ensures that the most up front point is indeed representative
  // of the cluster of points where it comes from
  // Point should be within 2*sigma of the centroid
  double dist_to_cluster = 2;

  // Make copies of the LiDAR points so we can remove outliers
  // if necessary
  std::vector<LidarPoint> lidar_points_prev = lidarPointsPrev;
  std::vector<LidarPoint> lidar_points_curr = lidarPointsCurr;

  // x - moves forward and backwards - length
  // y - moves side-to-side - width
  // Only consider points that are within the lane
  // Keep looping until we know for sure that points are not outliers
  while (true) {
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

    // Calculate mean distance
    double mean_dist =
        std::accumulate(distances.begin(), distances.end(), 0.0) /
        distances.size();
    // Calculate std. dev.
    double stddev_dist = 0.0;
    for (const double val : distances) {
      stddev_dist += ((val - mean_dist) * (val - mean_dist));
    }
    stddev_dist = std::sqrt(stddev_dist / (distances.size() - 1));

    // If this distance is larger than 2*sigma of the distances,
    // delete and try again
    if (std::abs(distances[prev_index] - mean_dist) >
        dist_to_cluster * stddev_dist) {
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

    // Calculate mean distance
    mean_dist = std::accumulate(distances.begin(), distances.end(), 0.0) /
                distances.size();
    // Calculate std.dev
    stddev_dist = 0.0;
    for (const double val : distances) {
      stddev_dist += ((val - mean_dist) * (val - mean_dist));
    }
    stddev_dist = std::sqrt(stddev_dist / (distances.size() - 1));

    // If distance of this point to the centroid is larger than 2*sigma
    // delete and try again
    if (std::abs(distances[curr_index] - mean_dist) >
        dist_to_cluster * stddev_dist) {
      lidar_points_curr.erase(lidar_points_curr.begin() + curr_index);
      continue;
    }
    break;
  }
  // compute TTC from both measurements
  TTC = minXCurr * dT / (minXPrev - minXCurr);
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
  // We only increment if the keypoint found in the bounding box of the
  // previous frame overlaps with the bounding box of the next frame by
  // some threshold, say 50%.
  // For each match
  //    Grab trainIdx (current frame index) and queryIdx (previous frame index)
  //    Use trainIdx to access currFrame.keypoints[trainIdx].pt keypoint
  //    Use queryIdx to access prevFrame.keypoints[queryIdx].pt keypoint
  //    Figure out which bounding box each keypoint belongs to
  //    for the current frame keypoint using currFrame and the previous
  //    frame keypoint using prevFrame
  //    Determine IoU between the bounding boxes
  //    If IoU is larger than a threshold, access the boxID
  //    for the prevFrame and the currFrame and log into map

  // Define IoU function
  auto iou = [](const cv::Rect& bbox_a, const cv::Rect& bbox_b) -> double {
    const double x_a = std::max(bbox_a.x, bbox_b.x);
    const double y_a = std::max(bbox_a.y, bbox_b.y);
    const double x_b =
        std::min(bbox_a.x + bbox_a.width, bbox_b.x + bbox_b.width);
    const double y_b =
        std::min(bbox_a.y + bbox_a.height, bbox_b.y + bbox_b.height);

    const double inter_area =
        std::max(0.0, x_b - x_a + 1) * std::max(0.0, y_b - y_a + 1);
    const double bbox_a_area = (bbox_a.width + 1) * (bbox_a.height + 1);
    const double bbox_b_area = (bbox_b.width + 1) * (bbox_b.height + 1);
    return inter_area / (bbox_a_area + bbox_b_area - inter_area);
  };

  // Stores counts of overlapping matches between previous and current frame
  std::map<pair<int, int>, int> counts;

  // Thresholds for determining if we have valid tracked bounding boxes
  const int count_threshold = 10;
  const double min_iou_overlap = 0.25;
  for (const auto& dm : matches) {
    // Obtain feature points for previous and current frame
    const int curr_idx = dm.trainIdx;
    const int prev_idx = dm.queryIdx;
    const cv::Point2f curr_pt = currFrame.keypoints[curr_idx].pt;
    const cv::Point2f prev_pt = prevFrame.keypoints[prev_idx].pt;

    // Count the number of times each point is contained
    // in a bounding box over all bounding boxes for their
    // respective frames
    int count_prev = 0, count_curr = 0;
    BoundingBox prev_bbox, curr_bbox;
    for (const BoundingBox& box : prevFrame.boundingBoxes) {
      if (box.roi.contains(prev_pt)) {
        count_prev++;
        prev_bbox = box;
      }
    }

    for (const BoundingBox& box : currFrame.boundingBoxes) {
      if (box.roi.contains(curr_pt)) {
        count_curr++;
        curr_bbox = box;
      }
    }

    // We need to see if both counts are just 1 meaning
    // that the point only appeared within one object
    // bounding box.
    if (count_curr == 1 && count_prev == 1) {
      // Great, so we see that each point corresponds to just
      // one bounding box.  Now do both of these bounding boxes
      // overlap in some way?  The distance between successive
      // frames is quite small so we'd expect that a bounding box
      // moving in between frames will have a relatively small
      // displacement so there should be a large overlap
      // If there's a large overlap, then let's count up
      // how many times we see this combination
      if (iou(prev_bbox.roi, curr_bbox.roi) >= min_iou_overlap) {
        // Get previous and current box ID
        const int prev_bbox_id = prev_bbox.boxID;
        const int curr_bbox_id = curr_bbox.boxID;

        // See if we've encountered this combination before
        std::map<std::pair<int, int>, int>::iterator it =
            counts.find(std::make_pair(prev_bbox_id, curr_bbox_id));
        if (it == counts.end()) {
          // If we haven't, make a new entry to start counting
          counts.insert(std::pair<std::pair<int, int>, int>(
              std::make_pair(prev_bbox_id, curr_bbox_id), 0));
        } else {
          // If yes, update the entry
          const int count = it->second;
          it->second = count + 1;
        }
      }
    }
  }

  // Now go through all box ID pairs to see if there are a significant
  // number of keypoints between previous and current that would merit
  // that the bounding boxes correspond to each other
  // If so, log these
  for (const auto& it : counts) {
    if (it.second >= count_threshold) {
      bbBestMatches.insert(
          std::pair<int, int>(it.first.first, it.first.second));
    }
  }
}
