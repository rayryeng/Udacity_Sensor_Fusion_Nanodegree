#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

void gradientSobel()
{
    // TODO: Based on the image gradients in both x and y, compute an image 
    // which contains the gradient magnitude according to the equation at the 
    // beginning of this section for every pixel position. Also, apply different 
    // levels of Gaussian blurring before applying the Sobel operator and compare the results.

    // Step #1 - Load in an image
    cv::Mat img;
    img = cv::imread("../images/img1gray.png", cv::IMREAD_GRAYSCALE);
    cv::Mat output;

    // Step #2 - Calculate Gaussian Blur
    // 7 x 7 kernel, sigma = 2.0;
    cv::GaussianBlur(img, output, cv::Size(7, 7), 2.0);

    // Step #3 - Now calculate the gradients
    // Sobel - x direction
    cv::Mat sobelx, sobely;
    cv::Sobel(output, sobelx, CV_64F, 1, 0);
    cv::Sobel(output, sobely, CV_64F, 0, 1);

    // Step #4 - Calculate magnitude
    cv::Mat gradient = sobelx.mul(sobelx) + sobely.mul(sobely);
    cv::sqrt(gradient, gradient);

    double min, max;
    cv::minMaxIdx(gradient, &min, &max);
    cv::Mat gradient_uchar;

    // Scale gradient to the 0-255 range
    // (x - min) / (max - min) --> [0 - 1]
    // 255 * (x - min) / (max - min)
    // 255 * x / (max - min) - (255 * min) / (max - min)
    // alpha = 255 / (max - min)
    // beta = -(255 * min) / (max - min)
    gradient.convertTo(gradient_uchar, CV_8U, (255 / (max - min)), -255 * min / (max - min));

    // Show the image
    cv::imshow("Gradient", gradient_uchar);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

int main()
{
    gradientSobel();
}