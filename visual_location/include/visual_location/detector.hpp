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

//cv::Point2f center_cal(const std::vector<cv::Point>& contour);
//void find_QRcode(cv::Mat& src,std::vector<cv::Point2f>& pos,bool* rec);
//cv::Mat CorrectImage(const cv::Mat& img,cv::Mat cameraMatrix,cv::Mat distCoeffs);
static double angleBetween(const cv::Point2f& p, const cv::Point2f& q, const cv::Point2f& r);


/*
std::vector<cv::Point3f> objectPoints = {
        cv::Point3f(0.0f, 0.0f, 0.0f),
        cv::Point3f(48.0f, 0.0f, 0.0f),
        cv::Point3f(0.0f, 48.0f, 0.0f),
        cv::Point3f(48.0f, 48.0f, 1.0f)
    };
cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1078.5710352, 0, 960.42804705,
                                                 0, 1070.62559673, 524.7678925,
                                                 0, 0, 1);
cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << -0.06820831, 0.14711315, 0.00048244, 0.00042337, 0.11601874);
*/


/*
class CamSet {
public:
    // 单目相机参数
    //cv::Mat cameraMatrix;
    //cv::Mat distCoeffs;

    CamSet() {
        // 初始化相机内参矩阵
        //cameraMatrix = (cv::Mat_<double>(3, 3) << 1078.5710352, 0, 960.42804705,
         //                                        0, 1070.62559673, 524.7678925,
          //                                       0, 0, 1);

        // 初始化相机畸变系数矩阵 (k1, k2, p1, p2, k3)
        //distCoeffs = (cv::Mat_<double>(5, 1) << -0.06820831, 0.14711315, 0.00048244, 0.00042337, 0.11601874);
    }

    cv::Mat CorrectImage(const cv::Mat& img);
};


*/

class CamSet {
public:
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    CamSet() {
        // 单目相机参数
        cameraMatrix = (cv::Mat_<double>(3, 3) <<
            1487.431388, 0.0,         1241.60040811,
            0.0,         1478.4943856, 687.64527377,
            0.0,         0.0,         1.0);
        distCoeffs = (cv::Mat_<double>(1, 5) <<
            -0.24776359, 0.19104283, 0.0029431, 0.00005472, -0.10815356);
    }

    cv::Mat CorrectImage(const cv::Mat& img);
};


void find_QRcode(const cv::Mat& src, const CamSet& camset, double *p,double *y,double *r,double *d);



#endif