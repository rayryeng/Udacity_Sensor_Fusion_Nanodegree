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
// argv[3] --> bVis (visualising results): 0/1
// argv[4] --> bLimitKpts (for limiting keypoint reuslts): 0/1
// Defaults if you don't specify any command-line args
// SHITOMASI, BRISK, bVis=true

int main(int argc, const char* argv[]) {

  /* INIT VARIABLES AND DATA STRUCTURES */

  ////////// Command-line args
  // SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
  string detectorType = "SHITOMASI";
  if (argc >= 2) {
    detectorType = string(argv[1]);
  }

  // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
  string descriptorType = "BRISK";
  if (argc >= 3) {
    descriptorType = string(argv[2]);
  }

  // For visualization / debugging
  string bVisTemp = "1";
  if (argc >= 4) {
    bVisTemp = string(argv[3]);
  }
  const bool bVis = stoi(bVisTemp) == 1;

  // For limiting keypoints visualization
  bVisTemp = "1";
  if (argc >= 5) {
    bVisTemp = string(argv[4]);
  }
  const bool bLimitKpts = stoi(bVisTemp) == 1;

  cout << "=== Options ===\n";
  cout << "Detector Type: " << detectorType;
  cout << "\nDescriptor Type: " << descriptorType;
  cout << "\nVisualization: " << bVis;
  cout << "\nLimit keypoints visualization: " << bLimitKpts << "\n\n";

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
    //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

    if (detectorType.compare("SHITOMASI") == 0) {
      detKeypointsShiTomasi(keypoints, imgGray, bVis);
    } else if (detectorType == "HARRIS") {
      detKeypointsHarris(keypoints, imgGray, bVis);
    } else {
      detKeypointsModern(keypoints, imgGray, detectorType, bVis);
    }
    //// EOF STUDENT ASSIGNMENT

    //// STUDENT ASSIGNMENT
    //// TASK MP.3 -> only keep keypoints on the preceding vehicle

    // only keep keypoints on the preceding vehicle
    bool bFocusOnVehicle = true;
    cv::Rect vehicleRect(535, 180, 180, 150);
    if (bFocusOnVehicle) {
      // ...
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
    //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

    cv::Mat descriptors;
    descKeypoints((dataBuffer.end() - 1)->keypoints,
                  (dataBuffer.end() - 1)->cameraImg, descriptors,
                  descriptorType);
    //// EOF STUDENT ASSIGNMENT

    // push descriptors for current frame to end of data buffer
    (dataBuffer.end() - 1)->descriptors = descriptors;

    cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

    // wait until at least two images have been processed
    if (dataBuffer.size() > 1) {

      /* MATCH KEYPOINT DESCRIPTORS */

      vector<cv::DMatch> matches;
      string matcherType = "MAT_BF";         // MAT_BF, MAT_FLANN
      string descriptorType = "DES_BINARY";  // DES_BINARY, DES_HOG
      string selectorType = "SEL_NN";        // SEL_NN, SEL_KNN

      //// STUDENT ASSIGNMENT
      //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
      //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

      matchDescriptors((dataBuffer.end() - 2)->keypoints,
                       (dataBuffer.end() - 1)->keypoints,
                       (dataBuffer.end() - 2)->descriptors,
                       (dataBuffer.end() - 1)->descriptors, matches,
                       descriptorType, matcherType, selectorType);

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

  }  // eof loop over all images

  return 0;
}
