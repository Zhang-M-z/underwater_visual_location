#ifndef BASE_HPP_
#define BASE_HPP_

#include <Eigen/Core>
#include <array>
#include <opencv2/opencv.hpp>
#include <vector>


std::vector<cv::Point3f> objectPoints = {
        cv::Point3f(0.0f, 0.0f, 0.0f),
        cv::Point3f(90.0f, 0.0f, 0.0f),
        cv::Point3f(0.0f, 90.0f, 0.0f),
        cv::Point3f(90.0f, 90.0f, 1.0f)
    };
cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1078.5710352, 0, 960.42804705,
                                                    0, 1070.62559673, 524.7678925,
                                                    0, 0, 1);
cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << -0.06820831, 0.14711315, 0.00048244, 0.00042337, 0.11601874);




#endif