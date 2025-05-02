#include <visual_location/detector_node.hpp>
#include <visual_location/detector.hpp>
#include <visual_location/pnp_solver.hpp>

OptimizedPnP::Params params;
OptimizedPnP pnp_solver;


void imageCallback(const sensor_msgs::ImageConstPtr& msg)  
{  
//sensor_msgs::Image ROS中image传递的消息形式
  try  
  {  
      //收集图像消息，cv_bridge将ros图像格式转换成cv图像格式，编码格式为bgr8
    
      
    CamSet config;
      

    
    cv::Mat frame;
    frame = cv_bridge::toCvShare(msg, "bgr8")->image ;

    
    
    cv::Mat correctedImg = config.CorrectImage(frame);
    std::vector<cv::Point2f> poly = find_QRcode(correctedImg);
    if (poly.size() != 4) {
    ROS_WARN("检测到的二维码点数不足 4 个！");
    return;  // 直接返回，避免后续操作
}
    cv::Mat rvec;  // 输出的旋转向量
    cv::Mat tvec;  // 输出的平移向量
    
    //bool success = pnp_solver.solve(objectPoints,poly,rvec,tvec,params);
    bool success = solvePnP(objectPoints,poly,cameraMatrix,distCoeffs,rvec,tvec);
    if (success) {
    //计算相机距离被测物的实际距离
        
        ROS_INFO("图像坐标：(%f,%f),(%f,%f),(%f,%f),(%f,%f)",poly[0].x,poly[0].y,poly[1].x,poly[1].y,poly[2].x,poly[2].y,poly[3].x,poly[3].y);
        

        ROS_INFO("旋转向量：%d,%d,%d",rvec.data[0],rvec.data[1],rvec.data[2]);
        ROS_INFO("平移向量：%d,%d,%d",tvec.data[0],tvec.data[1],tvec.data[2]);
        float distance = sqrt(tvec.at<double>(0,0) * tvec.at<double>(0,0) + tvec.at<double>(1,0) * tvec.at<double>(1,0) + tvec.at<double>(2,0) * tvec.at<double>(2,0)) / 10; 
        ROS_INFO("%f",distance);
    } 
    
  }  
  catch (cv_bridge::Exception& e)  //如果发生转换错误，如编码格式不匹配。则捕获异常
  {  
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());  
  }  
}  
 
int main(int argc, char **argv)  
{  
  ros::init(argc, argv, "image_sub_node");  
  ros::NodeHandle nh;  
    
  cv::startWindowThread();  
  image_transport::ImageTransport it(nh);  
  image_transport::Subscriber sub = it.subscribe("camera", 1, imageCallback);  
  ros::spin();  
  
  cv::destroyAllWindows();  //窗口
}  