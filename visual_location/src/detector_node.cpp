#include <visual_location/detector_node.hpp>
#include <visual_location/detector.hpp>
#include <visual_location/base.hpp>
extern cv::Mat cameraMatrix;        
extern cv::Mat distCoeffs;
extern std::vector<cv::Point3f> objectPoints;
std::vector<cv::Point2f> pos(4);
using namespace cv;
using namespace std;
double pitch;
double row;
double yaw;
double dis;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)  
{  
//sensor_msgs::Image ROS中image传递的消息形式
  try  
  {  
      //收集图像消息，cv_bridge将ros图像格式转换成cv图像格式，编码格式为bgr8
    bool rec = 0;
      
    
      

    
    cv::Mat frame;
    frame = cv_bridge::toCvShare(msg, "bgr8")->image ;

    
    /*
    cv::Mat correctedImg = CorrectImage(frame,cameraMatrix,distCoeffs);
    //std::vector<cv::Point2f> pos(4);
    find_QRcode(correctedImg,pos,&rec);
    
    if(rec==0) {
      ROS_INFO("没有目标");
      return;
    }
     


  
    cv::Mat rvec;  // 输出的旋转向量
    cv::Mat tvec;  // 输出的平移向量
    
    //bool success = pnp_solver.solve(objectPoints,poly,rvec,tvec,params);
    bool success = solvePnP(objectPoints,pos,cameraMatrix,distCoeffs,rvec,tvec,false,
            cv::SOLVEPNP_ITERATIVE);
    cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 100, 1e-6);
    
    // 调用优化函数
    cv::solvePnPRefineVVS(
        objectPoints, 
        pos, 
        cameraMatrix, 
        distCoeffs, 
        rvec, 
        tvec, 
        criteria,
        50.0 // 默认VVSlambda为1，可根据需要调整
    );
    cv::Mat rotM = cv::Mat::eye(3,3,CV_64F);
    cv::Mat rotT = cv::Mat::eye(3,3,CV_64F);
    Rodrigues(rvec, rotM);  //将旋转向量变换成旋转矩阵
    Rodrigues(tvec, rotT);

    double theta_x, theta_y,theta_z;
    double PI = 3.14;
    theta_x = atan2(rotM.at<double>(2, 1), rotM.at<double>(2, 2));
    theta_y = atan2(-rotM.at<double>(2, 0),
    sqrt(rotM.at<double>(2, 1)*rotM.at<double>(2, 1) + rotM.at<double>(2, 2)*rotM.at<double>(2, 2)));
    theta_z = atan2(rotM.at<double>(1, 0), rotM.at<double>(0, 0));
    theta_x = theta_x * (180 / PI);
    theta_y = theta_y * (180 / PI);
    theta_z = theta_z * (180 / PI);

    cv::Mat P;
    P = (rotM.t()) * tvec;
    if (success) {
    //计算相机距离被测物的实际距离
        
        //ROS_INFO("图像坐标：(%f,%f),(%f,%f),(%f,%f),(%f,%f)",pos[0].x,pos[0].y,pos[1].x,pos[1].y,pos[2].x,pos[2].y,pos[3].x,pos[3].y);
        

        //ROS_INFO("旋转向量：%d,%d,%d",rvec.data[0],rvec.data[1],rvec.data[2]);
        ROS_INFO("平移向量：%d,%d,%d",tvec.data[0],tvec.data[1],tvec.data[2]);
        ROS_INFO("欧拉角：%f,%f,%f",theta_x,theta_y,theta_z);
       


        float distance = sqrt(tvec.data[0] * tvec.data[0] + tvec.data[1] * tvec.data[1] + tvec.data[2] * tvec.data[2]) / 10; 
        ROS_INFO("%f",distance);
        */
      CamSet cam;
      

      namedWindow("video", WINDOW_AUTOSIZE);
      
      
      while (true) {
        
        Mat undist = cam.CorrectImage(frame);
        find_QRcode(undist,cam,&pitch,&yaw,&row,&dis);
        ROS_INFO("欧拉角：%f,%f,%f",pitch,yaw,row);
        if ((waitKey(1) & 0xFF) == 27) break;
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