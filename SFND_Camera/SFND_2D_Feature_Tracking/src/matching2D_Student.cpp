#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint>& kPtsSource,
                      std::vector<cv::KeyPoint>& kPtsRef,
                      cv::Mat& descSource,
                      cv::Mat& descRef,
                      std::vector<cv::DMatch>& matches,
                      std::string descriptorType,
                      std::string matcherType,
                      std::string selectorType) {
  // configure matcher
  bool crossCheck = false;
  cv::Ptr<cv::DescriptorMatcher> matcher;

  if (matcherType.compare("MAT_BF") == 0) {
    int normType = cv::NORM_HAMMING;
    matcher = cv::BFMatcher::create(normType, crossCheck);
  } else if (matcherType.compare("MAT_FLANN") == 0) {
    // ...
  }

  // perform matching task
  if (selectorType.compare("SEL_NN") == 0) {  // nearest neighbor (best match)
    // Finds the best match for each descriptor in desc1
    matcher->match(descSource, descRef, matches);
  }  // k nearest neighbors (k=2)
  else if (selectorType.compare("SEL_KNN") == 0) {

    // ...
  }
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint>& keypoints,
                   cv::Mat& img,
                   cv::Mat& descriptors,
                   string descriptorType) {
  // select appropriate descriptor
  cv::Ptr<cv::DescriptorExtractor> extractor;
  if (descriptorType.compare("BRISK") == 0) {

    int threshold = 30;  // FAST/AGAST detection threshold score.
    int octaves = 3;     // detection octaves (use 0 to do single scale)
    // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.
    float patternScale = 1.0f;

    extractor = cv::BRISK::create(threshold, octaves, patternScale);
  } else {

    //...
  }

  // perform feature description
  double t = (double)cv::getTickCount();
  extractor->compute(img, keypoints, descriptors);
  t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
  cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0
       << " ms" << endl;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint>& keypoints,
                           cv::Mat& img,
                           bool bVis) {
  // compute detector parameters based on image size
  //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
  int blockSize = 4;
  // max. permissible overlap between two features in %
  double maxOverlap = 0.0;
  double minDistance = (1.0 - maxOverlap) * blockSize;
  // max. num. of keypoints
  int maxCorners = img.rows * img.cols / max(1.0, minDistance);

  double qualityLevel = 0.01;  // minimal accepted quality of image corners
  double k = 0.04;

  // Apply corner detection
  double t = (double)cv::getTickCount();
  vector<cv::Point2f> corners;
  cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance,
                          cv::Mat(), blockSize, false, k);

  // add corners to result vector
  for (auto it = corners.begin(); it != corners.end(); ++it) {
    cv::KeyPoint newKeyPoint;
    newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
    newKeyPoint.size = blockSize;
    keypoints.push_back(newKeyPoint);
  }
  t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
  cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in "
       << 1000 * t / 1.0 << " ms" << endl;

  // visualize results
  if (bVis) {
    cv::Mat visImage = img.clone();
    cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    string windowName = "Shi-Tomasi Corner Detector Results";
    cv::namedWindow(windowName, 6);
    imshow(windowName, visImage);
    cout << "Press any key to continue\n";
    cv::waitKey(0);
  }
}

// Added in by Ray from the previous exercises
void detKeypointsHarris(vector<cv::KeyPoint>& keypoints,
                        cv::Mat& img,
                        bool bVis) {

  // Detector parameters
  // for every pixel, a blockSize × blockSize neighborhood is considered
  int blockSize = 2;
  // aperture parameter for Sobel operator (must be odd)
  int apertureSize = 3;
  // minimum value for a corner in the 8bit scaled response matrix
  int minResponse = 100;
  double k = 0.04;  // Harris parameter (see equation for details)

  // Detect Harris corners and normalize output
  cv::Mat dst, dst_norm, dst_norm_scaled;
  dst = cv::Mat::zeros(img.size(), CV_32FC1);
  double t = (double)cv::getTickCount();
  cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
  cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1);
  cv::convertScaleAbs(dst_norm, dst_norm_scaled);

  // Perform non-maximum suppression
  const int nms_window_size = 2 * apertureSize + 1;  // 7 x 7
  const int rows = dst_norm.rows;
  const int cols = dst_norm.cols;

  // Store the resulting points in a vector of cv::KeyPoints
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      int centre_r = -1;
      int centre_c = -1;

      // The max value is set to the minimum response
      // We should have keypoints that exceed this first
      unsigned char max_val = static_cast<unsigned char>(minResponse);
      for (int x = -nms_window_size; x <= nms_window_size; x++) {
        for (int y = -nms_window_size; y <= nms_window_size; y++) {
          if ((i + x) < 0 || (i + x) >= rows) { continue; }
          if ((j + y) < 0 || (j + y) >= cols) { continue; }
          const unsigned char val =
              dst_norm_scaled.at<unsigned char>(i + x, j + y);
          if (val > max_val) {
            max_val = val;
            centre_r = i + x;
            centre_c = j + y;
          }
        }
      }

      // If the largest value was at the centre, remember this keypoint
      if (centre_r == i && centre_c == j) {
        keypoints.emplace_back(j, i, 2 * apertureSize, -1, max_val);
      }
    }
  }

  t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
  cout << "Harris detection with n=" << keypoints.size() << " keypoints in "
       << 1000 * t / 1.0 << " ms" << endl;

  // visualize results
  if (bVis) {
    cv::Mat visImage = img.clone();
    cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    string windowName = "Harris Corner Detector Results";
    cv::namedWindow(windowName, 6);
    imshow(windowName, visImage);
    cout << "Press any key to continue\n";
    cv::waitKey(0);
  }
}

// For the remaining detector types: FAST, BRISK, ORB, AKAZE, SIFT
void detKeypointsModern(std::vector<cv::KeyPoint>& keypoints,
                        cv::Mat& img,
                        std::string detectorType,
                        bool bVis) {

  // Stores created detector here
  cv::Ptr<cv::FeatureDetector> detector;

  // Figure out which keypoint we want
  if (detectorType == "FAST") {
    // difference between intensity of the central pixel and pixels of a circle around this pixel
    const int FAST_threshold = 30;
    // perform non-maxima suppression on keypoints
    const bool bNMS = true;
    // TYPE_9_16, TYPE_7_12, TYPE_5_8
    cv::FastFeatureDetector::DetectorType type =
        cv::FastFeatureDetector::TYPE_9_16;
    detector = cv::FastFeatureDetector::create(FAST_threshold, bNMS, type);
  } else if (detectorType == "BRISK") {
    detector = cv::BRISK::create();
  } else if (detectorType == "ORB") {
    detector = cv::ORB::create();
  } else if (detectorType == "AKAZE") {
    detector = cv::AKAZE::create();
  } else if (detectorType == "SIFT") {
    detector = cv::xfeatures2d::SIFT::create();
  } else {
    return;
  }

  // Detect the keypoints
  double t = (double)cv::getTickCount();
  detector->detect(img, keypoints);
  t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
  cout << detectorType << " detection with n=" << keypoints.size()
       << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

  // visualize results
  if (bVis) {
    cv::Mat visImage = img.clone();
    cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    string windowName = detectorType + " Detector Results";
    cv::namedWindow(windowName, 6);
    imshow(windowName, visImage);
    cout << "Press any key to continue\n";
    cv::waitKey(0);
  }
}