/**
* @brief: ROS Node for reading a Video
* @details: ROS node to read frames from a video and publish image frames to a topic /image_raw.
* @author: Darshan Shah
* @date: March 4th 2019.
*
*/

#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "tag_detector/set_blur_window_size.h"


using namespace std;
using namespace cv;

int GaussWindowWidth = 9;
int GaussWindowHeight = 9;

bool getWindowSize(tag_detector::set_blur_window_size::Request &req, 
         tag_detector::set_blur_window_size::Response &res)
{
  GaussWindowWidth = req.i;
  GaussWindowHeight = req.j;
  res.status = true;
  ROS_INFO("Gaussian Window Size Set to [%d,%d] successfully!", GaussWindowWidth, GaussWindowHeight);
  return true;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "video_reader");
  string path = ros::package::getPath("tag_detector");
	string filePath = "/data/1.mp4";

  if(argc == 2){
    string fileNumber = argv[1];
    if (fileNumber == "1"){
      cout << "Normal Lighting video Selected. To Select Night lighting video, set argv = 2"<<endl;
      filePath = "/data/1.mp4";
    } else if (fileNumber == "2"){
      cout << "Night light video Selected. To select Normal lighting video, set argv = 1"<<endl;
      filePath = "/data/2.mp4";
    }
  }
	ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/image_raw", 1);
    ros::ServiceServer service = nh.advertiseService("set_blur_window_size", getWindowSize);


    VideoCapture cap(path + filePath);
    ROS_INFO("Total Frames: %lf", cap.get(CV_CAP_PROP_FRAME_COUNT));
    ros::Rate loop_rate(5);

    cv::Mat frame, blurFrame;
  	sensor_msgs::ImagePtr msg;

  	while(nh.ok()) {  //Run the loop while ROS node is running.
  		cap>>frame;
      GaussianBlur(frame, blurFrame, Size(GaussWindowWidth, GaussWindowHeight), 0, 0 );
  		if(!frame.empty()){
  			msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", blurFrame).toImageMsg();
  			pub.publish(msg);
  			// imshow("Video Frame", frame);  //Uncomment to see the images being read.
  			waitKey(8);
  		} else {
  			ROS_WARN("No Image frames to Read :P");
  		}
  		ros::spinOnce();
  		loop_rate.sleep();

  	}
  // destroyAllWindows();
 }

