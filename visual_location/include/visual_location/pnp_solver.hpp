#ifndef PNP_SOLVER_HPP_
#define PNP_SOLVER_HPP_


#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

class OptimizedPnP {
public:
    struct Params {
        cv::Mat camera_matrix=(cv::Mat_<double>(3, 3) << 1078.5710352, 0, 960.42804705,
                                                 0, 1070.62559673, 524.7678925,
                                                 0, 0, 1); // 相机内参矩阵
        cv::Mat dist_coeffs= (cv::Mat_<double>(5, 1) << -0.06820831, 0.14711315, 0.00048244, 0.00042337, 0.11601874);    // 畸变系数
        int max_iterations = 30; // 最大迭代次数
        double eps = 1e-6;       // 收敛阈值
    };

    bool solve(const std::vector<cv::Point3f> object_points,
            const std::vector<cv::Point2f> image_points,
               cv::Mat& rvec, cv::Mat& tvec,
               const Params& params);

    static double computeReprojectionError(
        const std::vector<cv::Point3f>& object_points,
        const std::vector<cv::Point2f>& image_points,
        const cv::Mat& rvec, const cv::Mat& tvec,
        const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs);

};








#endif