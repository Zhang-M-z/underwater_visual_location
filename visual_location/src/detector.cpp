#include <visual_location/detector.hpp>
#include <visual_location/base.hpp>


/*
cv::Mat CamSet::CorrectImage(const cv::Mat& img) {
        cv::Mat newCameraMatrix, roi;
        cv::Size imageSize = img.size();
        newCameraMatrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0);//原本为&roi
        
        cv::Mat mapx, mapy;
        cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), newCameraMatrix, imageSize, CV_16SC2, mapx, mapy);
        
        cv::Mat correctedImg;
        cv::remap(img, correctedImg, mapx, mapy, cv::INTER_LINEAR);
        
        return correctedImg;
    }

*/
/*
cv::Mat CorrectImage(const cv::Mat& img,cv::Mat cameraMatrix,cv::Mat distCoeffs) {
        cv::Mat newCameraMatrix, roi;
        cv::Size imageSize = img.size();
        newCameraMatrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0);//原本为&roi
        
        cv::Mat mapx, mapy;
        cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), newCameraMatrix, imageSize, CV_16SC2, mapx, mapy);
        
        cv::Mat correctedImg;
        cv::remap(img, correctedImg, mapx, mapy, cv::INTER_LINEAR);
        
        return correctedImg;
    
    }

cv::Point2f center_cal(const std::vector<cv::Point>& contour) {
    cv::Moments m = cv::moments(contour);
    if (m.m00 != 0) {
        return cv::Point2f(m.m10 / m.m00, m.m01 / m.m00);
    }
    return cv::Point2f(0, 0);
}

void find_QRcode(cv::Mat& src,std::vector<cv::Point2f>& pos,bool *rec) {
    *rec = 1;
    cv::Mat src_all = src.clone();
    cv::Mat src_gray;
    cv::cvtColor(src, src_gray, cv::COLOR_BGR2GRAY);
    cv::blur(src_gray, src_gray, cv::Size(3, 3));
    cv::equalizeHist(src_gray, src_gray);
    
    cv::Mat threshold_output;
    cv::threshold(src_gray, threshold_output, 112, 255, cv::THRESH_BINARY);
    
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(threshold_output, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    
    std::vector<std::tuple<int, int, int>> vin;
    
    // 筛选出有两个子轮廓的轮廓
    for (int i = 0; i < contours.size(); i++) {
        if (hierarchy[i][2] != -1) {
            int tempindex1 = hierarchy[i][2];
            if (hierarchy[tempindex1][2] != -1) {
                int tempindex2 = hierarchy[tempindex1][2];
                vin.push_back(std::make_tuple(i, tempindex1, tempindex2));
            }
        }
    }

    // 根据轮廓面积比例筛选
    std::vector<std::tuple<int, int, int>> vin_filtered;
    for (auto& item : vin) {
        std::vector<cv::Point> out1Contours = contours[std::get<0>(item)];
        std::vector<cv::Point> out2Contours = contours[std::get<1>(item)];
        double lenth1 = cv::arcLength(out1Contours, true);
        double lenth2 = cv::arcLength(out2Contours, true);
        if (std::abs(lenth1 / lenth2 - 2) <= 1) {
            vin_filtered.push_back(item);
        }
    }

    std::vector<cv::Point2f> pointthree;
    for (auto& item : vin_filtered) {
        pointthree.push_back(center_cal(contours[std::get<0>(item)]));
    }

    if (pointthree.size() < 3) {
        cv::imshow("video", src);
        *rec = 0;
        return ;
    }

    // 计算角度
    auto calculate_angle = [](cv::Point2f p1, cv::Point2f p2, cv::Point2f p3) {
        cv::Point2f ca = p2 - p1;
        cv::Point2f cb = p3 - p1;
        double dot_product = ca.x * cb.x + ca.y * cb.y;
        double norm_ca = std::sqrt(ca.x * ca.x + ca.y * ca.y);
        double norm_cb = std::sqrt(cb.x * cb.x + cb.y * cb.y);
        double angle = std::acos(dot_product / (norm_ca * norm_cb)) * 180.0 / CV_PI;
        double ccw = ca.x * cb.y - ca.y * cb.x;
        return std::make_tuple(angle, ccw);
    };

    std::vector<double> angles;
    std::vector<double> ccws;
    for (int i = 0; i < 3; i++) {
        auto [angle, ccw] = calculate_angle(pointthree[i], pointthree[(i + 1) % 3], pointthree[(i + 2) % 3]);
        angles.push_back(angle);
        ccws.push_back(ccw);
    }

    // 找到最大角度的点，确定透视变换
    int max_angle_idx = std::max_element(angles.begin(), angles.end()) - angles.begin();
    double max_angle = angles[max_angle_idx];

    std::vector<cv::Point2f> poly;
    if (max_angle_idx == 0) {
        poly = { pointthree[0], pointthree[1], pointthree[2], cv::Point2f(-pointthree[0].x + pointthree[1].x + pointthree[2].x, -pointthree[0].y + pointthree[1].y + pointthree[2].y) };
    } else if (max_angle_idx == 1) {
        poly = { pointthree[1], pointthree[0], pointthree[2], cv::Point2f(-pointthree[1].x + pointthree[2].x + pointthree[0].x, -pointthree[1].y + pointthree[2].y + pointthree[0].y) };
    } else {
        poly = { pointthree[2], pointthree[0], pointthree[1], cv::Point2f(-pointthree[2].x + pointthree[0].x + pointthree[1].x, -pointthree[2].y + pointthree[0].y + pointthree[1].y) };
    }

    // 旋转两向量以确定右上角与左下角的点
    auto rotate_vector = [](cv::Point2f origin, cv::Point2f point1, cv::Point2f point2) {
        cv::Point2f vector1 = point1 - origin;
        cv::Point2f vector2 = point2 - origin;
        
        double angle = std::atan2(vector1.y, vector1.x);
        cv::Mat rotationMatrix = (cv::Mat_<double>(2, 2) << std::cos(-angle), -std::sin(-angle),
                                                            std::sin(-angle), std::cos(-angle));
        cv::Mat rotated_vector = rotationMatrix * (cv::Mat_<double>(2, 1) << vector2.x, vector2.y);
        
        return cv::Point2f(rotated_vector.at<double>(0), rotated_vector.at<double>(1));
    };

    cv::Point2f rotated_vector = rotate_vector(poly[0], poly[1], poly[2]);
    if (rotated_vector.y > 0) {
        std::swap(poly[1], poly[2]);
    }

    // Draw result for debugging
    std::vector<cv::Point> points_int;
    for (const auto& pt : poly) 
    {
        points_int.push_back(cv::Point(static_cast<int>(pt.x), static_cast<int>(pt.y)));
    }
    cv::polylines(src, {points_int}, true, cv::Scalar(0, 255, 0), 2);
    cv::imshow("video", src);
    
    
    for(int a=0;a<4;a+=1)
    {
        pos[a]= poly[a];
    };
}
*/
/*
std::vector<cv::Point2f> find_QRcode(cv::Mat& src) {
    cv::Mat src_all = src.clone();
    cv::Mat src_gray;
    cv::cvtColor(src, src_gray, cv::COLOR_BGR2GRAY);
    cv::blur(src_gray, src_gray, cv::Size(3, 3));
    cv::equalizeHist(src_gray, src_gray);
    
    cv::Mat threshold_output;
    cv::threshold(src_gray, threshold_output, 112, 255, cv::THRESH_BINARY);
    
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(threshold_output, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    
    std::vector<std::tuple<int, int, int>> vin;
    
    // 筛选出有两个子轮廓的轮廓
    for (int i = 0; i < contours.size(); i++) {
        if (hierarchy[i][2] != -1) {
            int tempindex1 = hierarchy[i][2];
            if (hierarchy[tempindex1][2] != -1) {
                int tempindex2 = hierarchy[tempindex1][2];
                vin.push_back(std::make_tuple(i, tempindex1, tempindex2));
            }
        }
    }

    // 根据轮廓面积比例筛选
    std::vector<std::tuple<int, int, int>> vin_filtered;
    for (auto& item : vin) {
        std::vector<cv::Point> out1Contours = contours[std::get<0>(item)];
        std::vector<cv::Point> out2Contours = contours[std::get<1>(item)];
        double lenth1 = cv::arcLength(out1Contours, true);
        double lenth2 = cv::arcLength(out2Contours, true);
        if (std::abs(lenth1 / lenth2 - 2) <= 1) {
            vin_filtered.push_back(item);
        }
    }

    std::vector<cv::Point2f> pointthree;
    for (auto& item : vin_filtered) {
        pointthree.push_back(center_cal(contours[std::get<0>(item)]));
    }

    if (pointthree.size() < 3) {
        cv::imshow("video", src);
        std::vector<cv::Point2f> poly_zero;
        poly_zero = {cv::Point2f(0,0),cv::Point2f(0,0),cv::Point2f(0,0),cv::Point2f(0,0)};
        return poly_zero;
    }

    // 计算角度
    auto calculate_angle = [](cv::Point2f p1, cv::Point2f p2, cv::Point2f p3) {
        cv::Point2f ca = p2 - p1;
        cv::Point2f cb = p3 - p1;
        double dot_product = ca.x * cb.x + ca.y * cb.y;
        double norm_ca = std::sqrt(ca.x * ca.x + ca.y * ca.y);
        double norm_cb = std::sqrt(cb.x * cb.x + cb.y * cb.y);
        double angle = std::acos(dot_product / (norm_ca * norm_cb)) * 180.0 / CV_PI;
        double ccw = ca.x * cb.y - ca.y * cb.x;
        return std::make_tuple(angle, ccw);
    };

    std::vector<double> angles;
    std::vector<double> ccws;
    for (int i = 0; i < 3; i++) {
        auto [angle, ccw] = calculate_angle(pointthree[i], pointthree[(i + 1) % 3], pointthree[(i + 2) % 3]);
        angles.push_back(angle);
        ccws.push_back(ccw);
    }

    // 找到最大角度的点，确定透视变换
    int max_angle_idx = std::max_element(angles.begin(), angles.end()) - angles.begin();
    double max_angle = angles[max_angle_idx];

    std::vector<cv::Point2f> poly;
    if (max_angle_idx == 0) {
        poly = { pointthree[0], pointthree[1], pointthree[2], cv::Point2f(-pointthree[0].x + pointthree[1].x + pointthree[2].x, -pointthree[0].y + pointthree[1].y + pointthree[2].y) };
    } else if (max_angle_idx == 1) {
        poly = { pointthree[1], pointthree[0], pointthree[2], cv::Point2f(-pointthree[1].x + pointthree[2].x + pointthree[0].x, -pointthree[1].y + pointthree[2].y + pointthree[0].y) };
    } else {
        poly = { pointthree[2], pointthree[0], pointthree[1], cv::Point2f(-pointthree[2].x + pointthree[0].x + pointthree[1].x, -pointthree[2].y + pointthree[0].y + pointthree[1].y) };
    }

    // 旋转两向量以确定右上角与左下角的点
    auto rotate_vector = [](cv::Point2f origin, cv::Point2f point1, cv::Point2f point2) {
        cv::Point2f vector1 = point1 - origin;
        cv::Point2f vector2 = point2 - origin;
        
        double angle = std::atan2(vector1.y, vector1.x);
        cv::Mat rotationMatrix = (cv::Mat_<double>(2, 2) << std::cos(-angle), -std::sin(-angle),
                                                            std::sin(-angle), std::cos(-angle));
        cv::Mat rotated_vector = rotationMatrix * (cv::Mat_<double>(2, 1) << vector2.x, vector2.y);
        
        return cv::Point2f(rotated_vector.at<double>(0), rotated_vector.at<double>(1));
    };

    cv::Point2f rotated_vector = rotate_vector(poly[0], poly[1], poly[2]);
    if (rotated_vector.y > 0) {
        std::swap(poly[1], poly[2]);
    }

    // Draw result for debugging
    std::vector<cv::Point> points_int;
    for (const auto& pt : poly) 
    {
        points_int.push_back(cv::Point(static_cast<int>(pt.x), static_cast<int>(pt.y)));
    }
    cv::polylines(src, {points_int}, true, cv::Scalar(0, 255, 0), 2);
    cv::imshow("video", src);
    
    return poly;
}

*/

/*
int main() {
    cv::VideoCapture capture(0,cv::CAP_V4L2);
    if (!capture.isOpened()) {
        std::cerr << "Error opening video stream or file" << std::endl;
        return -1;
    }
    
    capture.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
    
    CamSet config;
    

    while (true) {
        cv::Mat frame;
        capture >> frame;
        if (frame.empty()) break;
        
        cv::Mat correctedImg = config.CorrectImage(frame);
        std::vector<cv::Point2f> poly = find_QRcode(correctedImg);
        
        cv::Mat rvec;  // 输出的旋转向量
        cv::Mat tvec;  // 输出的平移向量
        
        bool success = cv::solvePnP(objectPoints,poly, cameraMatrix, distCoeffs, rvec, tvec);

        if (success) {
        //计算相机距离被测物的实际距离
            
            ROS_INFO("图像坐标：(%f,%f),(%f,%f),(%f,%f),(%f,%f)",poly[0].x,poly[0].y,poly[1].x,poly[1].y,poly[2].x,poly[2].y,poly[3].x,poly[3].y);
            

            ROS_INFO("旋转向量：%d,%d,%d",rvec.data[0],rvec.data[1],rvec.data[2]);
            ROS_INFO("平移向量：%d,%d,%d",tvec.data[0],tvec.data[1],tvec.data[2]);
            float distance = sqrt(tvec.at<double>(0,0) * tvec.at<double>(0,0) + tvec.at<double>(1,0) * tvec.at<double>(1,0) + tvec.at<double>(2,0) * tvec.at<double>(2,0)) / 10; 
            ROS_INFO("%f",distance);
        }


        int c = cv::waitKey(1) & 0xFF;
        if (c == 27) break;

    }

    capture.release();
    cv::destroyAllWindows();
    return 0;

}
*/
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>

using namespace cv;
using namespace std;

cv::Mat CamSet::CorrectImage(const cv::Mat& img) {
        int w = img.cols;
        int h = img.rows;
        cv::Mat newCamMtx = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, cv::Size(w, h), 1, cv::Size(w, h), nullptr);
        cv::Mat mapx, mapy;
        cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), newCamMtx, cv::Size(w, h), CV_16SC2, mapx, mapy);
        cv::Mat undist;
        cv::remap(img, undist, mapx, mapy, INTER_LINEAR);
        return undist;
    }


static double angleBetween(const Point2f& p, const Point2f& q, const Point2f& r) {
    Point2f v1 = q - p;
    Point2f v2 = r - p;
    double dot = v1.dot(v2);
    double mag = norm(v1) * norm(v2);
    double cosang = dot / mag;
    cosang = std::max(-1.0, std::min(1.0, cosang));
    return acos(cosang) * 180.0 / CV_PI;
}

void find_QRcode(const Mat& src, const CamSet& cam, double *p,double *y,double *r,double *d) {
    Mat gray;
    cvtColor(src, gray, COLOR_BGR2GRAY);
    blur(gray, gray, Size(3, 3));
    equalizeHist(gray, gray);
    Mat thresh;
    threshold(gray, thresh, 112, 255, THRESH_BINARY);
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(thresh, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

    // 找到嵌套的方形Marker
    vector<Point2f> markers;
    for (size_t i = 0; i < contours.size(); ++i) {
        int child = hierarchy[i][2];
        if (child >= 0 && hierarchy[child][2] >= 0) {
            double peri = arcLength(contours[i], true);
            vector<Point> approx;
            approxPolyDP(contours[i], approx, 0.05 * peri, true);
            if (approx.size() == 4) {
                Moments M = moments(contours[i]);
                if (M.m00 != 0) {
                    float cx = static_cast<float>(M.m10 / M.m00);
                    float cy = static_cast<float>(M.m01 / M.m00);
                    markers.emplace_back(cx, cy);
                }
            }
        }
    }
    if (markers.size() < 3) {
        imshow("video", src);
        return;
    }

    // 提取前三个marker点：P, A, B
    vector<Point2f> pts(markers.begin(), markers.begin() + 3);

    // 寻找直角点P
    int right_idx = -1;
    for (int i = 0; i < 3; ++i) {
        int j = (i + 1) % 3;
        int k = (i + 2) % 3;
        if (fabs(angleBetween(pts[i], pts[j], pts[k]) - 90) < 20) {
            right_idx = i;
            break;
        }
    }
    if (right_idx < 0) {
        vector<double> errs;
        for (int i = 0; i < 3; ++i) {
            int j = (i + 1) % 3;
            int k = (i + 2) % 3;
            errs.push_back(fabs(angleBetween(pts[i], pts[j], pts[k]) - 90));
        }
        right_idx = static_cast<int>(min_element(errs.begin(), errs.end()) - errs.begin());
    }
    Point2f P = pts[right_idx];
    vector<int> idxs;
    for (int i = 0; i < 3; ++i) if (i != right_idx) idxs.push_back(i);
    Point2f A = pts[idxs[0]];
    Point2f B = pts[idxs[1]];

    // 计算第四点 D = A + B - P
    Point2f D = A + B - P;
    vector<Point2f> quad = {P, A, D, B};

    // 排序角点
    vector<float> s(4), diff(4);
    for (int i = 0; i < 4; ++i) {
        s[i] = quad[i].x + quad[i].y;
        diff[i] = quad[i].x - quad[i].y;
    }
    vector<Point2f> ordered = {
        quad[min_element(s.begin(), s.end()) - s.begin()],
        quad[min_element(diff.begin(), diff.end()) - diff.begin()],
        quad[max_element(s.begin(), s.end()) - s.begin()],
        quad[max_element(diff.begin(), diff.end()) - diff.begin()]
    };

    // 绘制三点连线
    vector<Point> draw_pts = {Point(cvRound(P.x), cvRound(P.y)),
                              Point(cvRound(A.x), cvRound(A.y)),
                              Point(cvRound(B.x), cvRound(B.y))};
    for (int i = 0; i < 3; ++i) {
        line(const_cast<Mat&>(src), draw_pts[i], draw_pts[(i+1)%3], Scalar(0,255,0), 4);
    }

    // 3D-2D 对应点：真实世界坐标(米)
    float real_size = 0.045f;
    vector<Point3f> obj_pts = {
        {0, 0, 0}, {real_size, 0, 0},
        {real_size, real_size, 0}, {0, real_size, 0}
    };
    vector<Point2f> img_pts = ordered;

    Mat rvec, tvec;
    bool success = solvePnP(obj_pts, img_pts, cam.cameraMatrix, cam.distCoeffs, rvec, tvec);
    if (!success) {
        imshow("video", src);
        return;
    }

    // 计算欧拉角
    Mat R;
    Rodrigues(rvec, R);
    double sy = sqrt(R.at<double>(0,0)*R.at<double>(0,0) + R.at<double>(1,0)*R.at<double>(1,0));
    double pitch = atan2(R.at<double>(2,1), R.at<double>(2,2)) * 180.0 / CV_PI;
    double yaw   = atan2(-R.at<double>(2,0), sy) * 180.0 / CV_PI;
    double roll  = atan2(R.at<double>(1,0), R.at<double>(0,0)) * 180.0 / CV_PI;

    double distance = tvec.at<double>(2);

    *p = pitch;
    *y = yaw;
    *r = roll;
    *d = distance;
    imshow("video", src);
}
