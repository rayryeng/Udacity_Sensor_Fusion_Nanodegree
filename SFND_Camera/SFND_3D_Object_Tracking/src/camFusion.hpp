
#ifndef camFusion_hpp
#define camFusion_hpp

#include <stdio.h>
#include <opencv2/core.hpp>
#include <vector>
#include "dataStructures.h"

void clusterLidarWithROI(std::vector<BoundingBox>& boundingBoxes,
                         std::vector<LidarPoint>& lidarPoints,
                         float shrinkFactor,
                         cv::Mat& P_rect_xx,
                         cv::Mat& R_rect_xx,
                         cv::Mat& RT);
void clusterKptMatchesWithROI(BoundingBox& boundingBox,
                              std::vector<cv::KeyPoint>& kptsPrev,
                              std::vector<cv::KeyPoint>& kptsCurr,
                              std::vector<cv::DMatch>& kptMatches);
void matchBoundingBoxes(std::vector<cv::DMatch>& matches,
                        std::map<int, int>& bbBestMatches,
                        DataFrame& prevFrame,
                        DataFrame& currFrame);

void show3DObjects(std::vector<BoundingBox>& boundingBoxes,
                   cv::Size worldSize,
                   cv::Size imageSize,
                   bool bWait = true);

// Changed to input an additional image to show matches
// between two frames - also got rid of the pointer declarations
void computeTTCCamera(std::vector<cv::KeyPoint>& kptsPrev,
                      std::vector<cv::KeyPoint>& kptsCurr,
                      std::vector<cv::DMatch> kptMatches,
                      double frameRate,
                      double& TTC,
                      const cv::Mat& visImgPrev = cv::Mat(),
                      const cv::Mat& visImgCurr = cv::Mat());
void computeTTCLidar(std::vector<LidarPoint>& lidarPointsPrev,
                     std::vector<LidarPoint>& lidarPointsCurr,
                     double frameRate,
                     double& TTC);
#endif /* camFusion_hpp */
