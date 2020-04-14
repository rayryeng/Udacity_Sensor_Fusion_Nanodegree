/* INCLUDES FOR THIS PROJECT */
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <sstream>
#include <vector>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/* MAIN PROGRAM */
// Edit by Ray
// We will take in command-line args that will help us change between
// the detectors and descriptors to prevent constant recompilation
// argv[0] --> Program
// argv[1] --> detectorType: SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
// argv[2] --> descriptorType: BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
// argv[3] --> matcherType (matching algorithms - brute-force, FLANN): MAT_BF, MAT_FLANN
// argv[4] --> selectorType (choosing the best match vs. k best matches): SEL_NN, SEL_KNN
// argv[5] --> bVis (visualising results): 0/1
// argv[6] --> bLimitKpts (for limiting keypoint results): 0/1
// argv[7] --> bFocusOnVehicle (to concentrate on vehicle from KITTI dataset): 0/1
// Defaults if you don't specify any command-line args
// SHITOMASI, BRISK, MAT_BF, DES_BINARY, bVis=true, bLimitKpts=true, bFocusOnVehicle=true

int main(int argc, const char* argv[]) {

  /* INIT VARIABLES AND DATA STRUCTURES */

  ////////// Command-line args
  // SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
  string detectorType = "SHITOMASI";
  if (argc >= 2) { detectorType = string(argv[1]); }

  // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
  string descriptorType = "BRISK";
  if (argc >= 3) { descriptorType = string(argv[2]); }

  // Descriptor and matcher strings for configuration
  // MAT_BF, MAT_FLANN
  string matcherType = "MAT_BF";
  if (argc >= 4) { matcherType = string(argv[3]); }

  // DES_BINARY, DES_HOG
  string descriptorContentType =
      descriptorType == "SIFT" ? "DES_HOG" : "DES_BINARY";

  // SEL_NN, SEL_KNN
  string selectorType = "SEL_NN";
  if (argc >= 5) { selectorType = string(argv[4]); }

  // For visualization / debugging
  string strToParse = "1";
  if (argc >= 6) { strToParse = string(argv[5]); }
  const bool bVis = stoi(strToParse) == 1;

  // For limiting keypoints visualization
  strToParse = "1";
  if (argc >= 7) { strToParse = string(argv[6]); }
  const bool bLimitKpts = stoi(strToParse) == 1;

  strToParse = "1";
  if (argc >= 8) { strToParse = string(argv[7]); }
  const bool bFocusOnVehicle = stoi(strToParse) == 1;

  cout << "=== Options ===\n";
  cout << "Detector Type: " << detectorType;
  cout << "\nDescriptor Type: " << descriptorType;
  cout << "\nMatcher Type: " << matcherType;
  cout << "\nDescriptor Content Type: " << descriptorContentType;
  cout << "\nSelector Type: " << selectorType;
  cout << "\nVisualization: " << bVis;
  cout << "\nLimit keypoints visualization: " << bLimitKpts;
  cout << "\nFocus on vehicle: " << bFocusOnVehicle << "\n\n";

  // data location
  string dataPath = "../";

  // camera
  string imgBasePath = dataPath + "images/";
  // left camera, color
  string imgPrefix = "KITTI/2011_09_26/image_00/data/000000";
  string imgFileType = ".png";
  // first file index to load (assumes Lidar and camera names have identical naming convention)
  int imgStartIndex = 0;
  int imgEndIndex = 9;  // last file index to load
  // no. of digits which make up the file index (e.g. img-0001.png)
  int imgFillWidth = 4;

  // misc
  // no. of images which are held in memory (ring buffer) at the same time
  int dataBufferSize = 2;
  // list of data frames which are held in memory at the same time
  vector<DataFrame> dataBuffer;

  // Timing measures to keep track of how long on average it took
  // for the detector and computing the descriptors
  double detector_time = 0.0;
  double descriptor_time = 0.0;
  int num_images = 0;
  /* MAIN LOOP OVER ALL IMAGES */

  for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex;
       imgIndex++) {
    /* LOAD IMAGE INTO BUFFER */

    // assemble filenames for current index
    ostringstream imgNumber;
    imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
    string imgFullFilename =
        imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

    // load image from file and convert to grayscale
    cv::Mat img, imgGray;
    img = cv::imread(imgFullFilename);
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

    //// STUDENT ASSIGNMENT
    //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

    // push image into data frame buffer
    DataFrame frame;
    frame.cameraImg = imgGray;

    // If we're at our limit, delete the first one
    if (dataBuffer.size() == dataBufferSize) {
      dataBuffer.erase(dataBuffer.begin());
    }
    // Now push the new one to the end
    dataBuffer.push_back(frame);

    //// EOF STUDENT ASSIGNMENT
    cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

    /* DETECT IMAGE KEYPOINTS */

    // extract 2D keypoints from current image
    // create empty feature list for current image
    vector<cv::KeyPoint> keypoints;

    //// STUDENT ASSIGNMENT
    //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
    //// -> SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
    if (detectorType.compare("SHITOMASI") == 0) {
      detector_time += detKeypointsShiTomasi(keypoints, imgGray, bVis);
    } else if (detectorType == "HARRIS") {
      detector_time += detKeypointsHarris(keypoints, imgGray, bVis);
    } else {
      detector_time +=
          detKeypointsModern(keypoints, imgGray, detectorType, bVis);
    }
    //// EOF STUDENT ASSIGNMENT

    //// STUDENT ASSIGNMENT
    //// TASK MP.3 -> only keep keypoints on the preceding vehicle

    // only keep keypoints on the preceding vehicle
    cv::Rect vehicleRect(535, 180, 180, 150);
    if (bFocusOnVehicle) {
      // Erase-Remove idiom on a std::vector
      // Remove keypoints that are outside the boundaries of the bounding box
      // remove_if returns an iterator such that the unwanted elements
      // are moved towards the tail end of the container, and we use erase
      // to remove these unwanted elements
      keypoints.erase(remove_if(keypoints.begin(), keypoints.end(),
                                [&vehicleRect](const cv::KeyPoint& point) {
                                  return !vehicleRect.contains(point.pt);
                                }),
                      keypoints.end());
    }

    //// EOF STUDENT ASSIGNMENT

    // optional : limit number of keypoints (helpful for debugging and learning)
    if (bLimitKpts) {
      int maxKeypoints = 50;

      // there is no response info, so keep the first 50 as they are sorted in descending quality order
      if (detectorType.compare("SHITOMASI") == 0) {
        keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
      }
      cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
      cout << " NOTE: Keypoints have been limited!" << endl;
    }

    // push keypoints and descriptor for current frame to end of data buffer
    (dataBuffer.end() - 1)->keypoints = keypoints;
    cout << "#2 : DETECT KEYPOINTS done" << endl;

    /* EXTRACT KEYPOINT DESCRIPTORS */

    //// STUDENT ASSIGNMENT
    //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
    //// -> BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
    cv::Mat descriptors;
    descriptor_time += descKeypoints((dataBuffer.end() - 1)->keypoints,
                                     (dataBuffer.end() - 1)->cameraImg,
                                     descriptors, descriptorType);
    //// EOF STUDENT ASSIGNMENT

    // push descriptors for current frame to end of data buffer
    (dataBuffer.end() - 1)->descriptors = descriptors;

    cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

    // wait until at least two images have been processed
    if (dataBuffer.size() > 1) {

      /* MATCH KEYPOINT DESCRIPTORS */

      vector<cv::DMatch> matches;
      //// STUDENT ASSIGNMENT
      //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
      //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

      matchDescriptors((dataBuffer.end() - 2)->keypoints,
                       (dataBuffer.end() - 1)->keypoints,
                       (dataBuffer.end() - 2)->descriptors,
                       (dataBuffer.end() - 1)->descriptors, matches,
                       descriptorContentType, matcherType, selectorType);

      //// EOF STUDENT ASSIGNMENT

      // store matches in current data frame
      (dataBuffer.end() - 1)->kptMatches = matches;

      cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

      // visualize matches between current and previous image
      if (bVis) {
        cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
        cv::drawMatches((dataBuffer.end() - 2)->cameraImg,
                        (dataBuffer.end() - 2)->keypoints,
                        (dataBuffer.end() - 1)->cameraImg,
                        (dataBuffer.end() - 1)->keypoints, matches, matchImg,
                        cv::Scalar::all(-1), cv::Scalar::all(-1),
                        vector<char>(),
                        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        string windowName = "Matching keypoints between two camera images";
        cv::namedWindow(windowName, 7);
        cv::imshow(windowName, matchImg);
        cout << "Press key to continue to next image" << endl;
        cv::waitKey(0);  // wait for key to be pressed
      }
    }
    num_images++;
  }  // eof loop over all images

  detector_time /= num_images;
  descriptor_time /= num_images;

  cout << "\n***** Final Aggregated Timings *****\n";
  cout << "Detector Time: " << 1000 * detector_time << " ms";
  cout << "\nDescriptor Time: " << 1000*descriptor_time << " ms\n";
  return 0;
}
