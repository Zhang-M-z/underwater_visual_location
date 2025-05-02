#include <visual_location/camera_capture.hpp>
 
std::string  source = "rtsp://192.168.1.64:554";
int main(int argc,char **argv)
{
    ros::init(argc,argv,"image_publisher");
    ros::NodeHandle n;
    
      // Open camera with CAMERA_INDEX (webcam is typically #0).
  //const int CAMERA_INDEX = 0;
  //cv::VideoCapture capture( CAMERA_INDEX, cv::CAP_V4L2); //摄像头视频的读操作
  cv::VideoCapture capture( source, cv::CAP_ANY);
  if( not capture.isOpened() )
  {
    ROS_ERROR_STREAM(
      "Failed to open camera with index " << source << "!"
    );
    ros::shutdown();//打开失败则关闭节点
  }
  
 
  // 创建ROS中图像的发布者
  image_transport::ImageTransport it( n ); 
  image_transport::Publisher pub_image = it.advertise( "camera", 1 );//将图像发布到话题，最多缓存一帧图像
 
 
  //cv_bridge功能包提供了ROS图像和OpenCV图像转换的接口，建立了一座桥梁
  cv_bridge::CvImagePtr frame = boost::make_shared< cv_bridge::CvImage >();//创建一个智能指针存储图像数据
  frame->encoding = sensor_msgs::image_encodings::BGR8;//设置图像编码格式为bgr8
 
  while( ros::ok() ) {
    capture >> frame->image; //流的转换
    if( frame->image.empty() )
    {
      ROS_ERROR_STREAM( "Failed to capture frame!" );
      ros::shutdown();
    }
    //打成ROS数据包
    frame->header.stamp = ros::Time::now();//设置ROS消息的时间戳为当前时间
    pub_image.publish( frame->toImageMsg() );//将图像cv格式转换成ros格式
    cv::waitKey( 3 );//opencv刷新图像 3ms
    ros::spinOnce();
    
  }
 
  capture.release();  //退出程序则释放流
  return EXIT_SUCCESS;
}
    
