/**
* @brief: ROS Node for reading images
* @details: ROS node to read frames from a ROS topic /image_raw.
* @author: Darshan Shah
* @date: March 4th 2019.
*
*/

#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace std;
using namespace cv;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
	try {
		Mat rec_img = cv_bridge::toCvShare(msg, "bgr8")->image;
    	imshow("view", rec_img);
    	waitKey(5);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  	}

}


int main(int argc, char** argv) {
	ros::init(argc, argv, "image_listener");
	
	ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/image_raw", 1, imageCallback);
    ros::spin();
    destroyAllWindows();
}