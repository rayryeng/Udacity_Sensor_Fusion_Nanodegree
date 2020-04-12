#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

using namespace std;

void cornernessHarris()
{
    // load image from file
    cv::Mat img;
    img = cv::imread("../images/img1.png");
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY); // convert to grayscale

    // Detector parameters
    int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04;       // Harris parameter (see equation for details)

    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1); //, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    // visualize results
    string windowName = "Harris Corner Detector Response Matrix";
    cv::namedWindow(windowName, 1); // Edit to scale of 1 instead of 4
    cv::imshow(windowName, dst_norm_scaled);
    cv::waitKey(0);
    cv::destroyWindow(windowName);

    // TODO: Your task is to locate local maxima in the Harris response matrix 
    // and perform a non-maximum suppression (NMS) in a local neighborhood around 
    // each maximum. The resulting coordinates shall be stored in a list of keypoints 
    // of the type `vector<cv::KeyPoint>`.
    const int nms_window_size = 2 * apertureSize + 1; // 7 x 7
    const int rows = dst_norm.rows;
    const int cols = dst_norm.cols;

    // Store the resulting points in a vector of cv::KeyPoints
    vector<cv::KeyPoint> keypoints;
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            int centre_r = -1;
            int centre_c = -1;

            // The max value is set to the minimum response
            // We should have keypoints that exceed this first
            unsigned char max_val = static_cast<unsigned char>(minResponse);
            for (int x = -nms_window_size; x <= nms_window_size; x++) {
                for (int y = -nms_window_size; y <= nms_window_size; y++) {
                    if ((i + x) < 0 || (i + x) >= rows) {
                        continue;
                    }
                    if ((j + y) < 0 || (j + y) >= cols) {
                        continue;
                    }
                    const unsigned char val = dst_norm_scaled.at<unsigned char>(i + x, j + y);
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
  
    cv::Mat output;
    cv::drawKeypoints(dst_norm_scaled, keypoints, output);
    string windowNameNMS = "Output Image with Keypoints (NMS)";
    cv::imshow(windowNameNMS, output);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

int main()
{
    cornernessHarris();
}