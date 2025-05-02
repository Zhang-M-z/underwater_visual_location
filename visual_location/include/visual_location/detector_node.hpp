#ifndef DETECTOR_NODE_HPP_
#define DETECTOR_NODE_HPP_


#include "ros/ros.h"  
#include <image_transport/image_transport.h>  
#include <opencv2/highgui/highgui.hpp>  
#include <cv_bridge/cv_bridge.h>  

void imageCallback(const sensor_msgs::ImageConstPtr& msg);

#endif