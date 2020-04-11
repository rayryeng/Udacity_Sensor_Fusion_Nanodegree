#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>


using namespace std;

void createMatrix1()
{
    // create matrix
    int nrows = 480, ncols = 640;
    cv::Mat m1_8u;
    m1_8u.create(nrows, ncols, CV_8UC1); // two-channel matrix with 8bit unsigned elements
    m1_8u.setTo(cv::Scalar(255));        // white

    // STUDENT TASK :
    // Create a variable of type cv::Mat* named m3_8u which has three channels with a
    // depth of 8bit per channel. Then, set the first channel to 255 and display the result.
    cv::Mat m3_8u = cv::Mat::zeros(nrows, ncols, CV_8UC3);

    // Split the channels, set the first one to all 255 then merge back
    vector<cv::Mat> chans;
    cv::split(m3_8u, chans);
    chans[0].setTo(cv::Scalar(255));
    cv::merge(chans, m3_8u);


    // show result
    string windowName = "First steps in OpenCV (m1_8u)";
    cv::namedWindow(windowName, 1); // create window
    cv::imshow(windowName, m1_8u);
    cv::waitKey(0); // wait for keyboard input before continuing
    cv::destroyWindow(windowName); // Added by Ray

    // STUDENT TASK :
    // Display the results from the STUDENT TASK above
    string windowName2 = "Create CV_8UC3 image with blue channel set to 0";
    cv::namedWindow(windowName2, 1);
    cv::imshow(windowName2, m3_8u);
    cv::waitKey(0);
    cv::destroyWindow(windowName2);
}


int main()
{
    createMatrix1();
    return 0;
}