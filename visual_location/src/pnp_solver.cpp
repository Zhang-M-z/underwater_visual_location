#include <visual_location/pnp_solver.hpp>


bool OptimizedPnP::solve(const std::vector<cv::Point3f> object_points,
                const std::vector<cv::Point2f> image_points,
               cv::Mat& rvec, cv::Mat& tvec,
               const Params& params = Params()) {
        // 初始解：使用EPnP快速估计位姿
        if (!cv::solvePnP(object_points, image_points,
                          params.camera_matrix, params.dist_coeffs,
                          rvec, tvec, false, cv::SOLVEPNP_EPNP)) {
            return false;
        }

        // LM算法优化：最小化重投影误差
        cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
                                  params.max_iterations, params.eps);
        cv::solvePnPRefineLM(object_points, image_points,
                             params.camera_matrix, params.dist_coeffs,
                             rvec, tvec, criteria);

        return true;
    };

double OptimizedPnP::computeReprojectionError(
        const std::vector<cv::Point3f>& object_points,
        const std::vector<cv::Point2f>& image_points,
        const cv::Mat& rvec, const cv::Mat& tvec,
        const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs) {
        std::vector<cv::Point2f> projected_points;
        cv::projectPoints(object_points, rvec, tvec,
                          camera_matrix, dist_coeffs, projected_points);

        double total_error = 0.0;
        for (size_t i = 0; i < image_points.size(); ++i) {
            total_error += cv::norm(image_points[i] - projected_points[i]);
        }
        return total_error / image_points.size();
    };
    
