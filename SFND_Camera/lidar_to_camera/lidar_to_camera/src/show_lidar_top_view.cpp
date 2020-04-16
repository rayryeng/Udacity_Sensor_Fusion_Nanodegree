#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "structIO.hpp"

using namespace std;

void showLidarTopview() {
  std::vector<LidarPoint> lidarPoints;
  readLidarPts("../dat/C51_LidarPts_0000.dat", lidarPoints);

  cv::Size worldSize(10.0, 20.0);  // width and height of sensor field in m
  cv::Size imageSize(1000, 2000);  // corresponding top view image in pixel

  // create topview image
  cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(0, 0, 0));

  // plot Lidar points into image
  for (auto it = lidarPoints.begin(); it != lidarPoints.end(); ++it) {
    float xw =
        (*it).x;  // world position in m with x facing forward from sensor
    float yw = (*it).y;  // world position in m with y facing left from sensor

    // Origin of the car is "middle" and bottom
    // Column coordinate - -xw rotates the axis so that instead of pointing up
    // we point down.  imageSize.height / worldSize.height gives us
    // the scaling factor to quantify how much real world distance x maps
    // to pixel coordinates.  Dividing by the world size height, gives us normalised
    // coordinates.  We then scale by the image height, then offset by the
    // image height to move us to the top left corner
    // Row coordinate - -yw rotates the axis so that instead of pointing to
    // the left, we point to the right.  We find the scaling factor
    // just like the column coordinate, then offset by the image width / 2
    // to again move us to the left column
    int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
    int x = (-yw * imageSize.height / worldSize.height) + imageSize.width / 2;

    //cv::circle(topviewImg, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);

    // TODO:
    // 1. Change the color of the Lidar points such that
    // X=0.0m corresponds to red while X=20.0m is shown as green.
    // 2. Remove all Lidar points on the road surface while preserving
    // measurements on the obstacles in the scene.

    // Trial and error - the lidar is mounted on top of the car and the
    // height from the top of the car to the road is about 1.4 m
    // +z is pointing upwards hence -1.4m is moving downwards from
    // the origin of the LIDAR
    if (it->z <= -1.4) { continue; }

    // Calculate the colour required
    // If x = 0, ratio = 1, so colour_val = 255.  If x = 20, ratio = 0, so colour_val = 0
    const double ratio = (worldSize.height - xw) / worldSize.height;
    const int colour_val = static_cast<int>(255 * ratio);
    cv::circle(topviewImg, cv::Point(x, y), 5,
               cv::Scalar(0, 255 - colour_val, colour_val));
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
  string windowName = "Top-View Perspective of LiDAR data";
  cv::namedWindow(windowName, 2);
  cv::imshow(windowName, topviewImg);
  cv::waitKey(0);  // wait for key to be pressed
}

int main() {
  showLidarTopview();
}