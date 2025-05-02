#ifndef DETECTOR_HPP_
#define DETECTOR_HPP_

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <cmath>
#include <vector>
#include <iostream>
#include <algorithm>
#include "ros/ros.h"



extern std::vector<cv::Point3f> objectPoints;
extern cv::Mat cameraMatrix;
extern cv::Mat distCoeffs;

cv::Point2f center_cal(const std::vector<cv::Point>& contour);
std::vector<cv::Point2f> find_QRcode(cv::Mat& src);

class CamSet {
public:
    // 单目相机参数
    //cv::Mat cameraMatrix;
    //cv::Mat distCoeffs;

    CamSet() {
        // 初始化相机内参矩阵
        cameraMatrix = (cv::Mat_<double>(3, 3) << 1078.5710352, 0, 960.42804705,
                                                 0, 1070.62559673, 524.7678925,
                                                 0, 0, 1);

        // 初始化相机畸变系数矩阵 (k1, k2, p1, p2, k3)
        distCoeffs = (cv::Mat_<double>(5, 1) << -0.06820831, 0.14711315, 0.00048244, 0.00042337, 0.11601874);
    }

    cv::Mat CorrectImage(const cv::Mat& img);
};











#endif