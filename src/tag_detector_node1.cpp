/**
* @brief: ROS Node for reading a Video
* @details: ROS node to read frames from a video and publish image frames to a topic /image_raw.
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



int main(int argc, char** argv) {
	ros::init(argc, argv, "video_reader");
	
	ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/image_raw", 1);

    VideoCapture cap("/home/darshan/catkin_ws/src/tag_detector/data/1.mp4");
    ROS_INFO("Total Frames: %lf", cap.get(CV_CAP_PROP_FRAME_COUNT));
    ros::Rate loop_rate(5);

    cv::Mat frame;
  	sensor_msgs::ImagePtr msg;

  	while(nh.ok()) {  //Run the loop while ROS node is running.
  		cap>>frame;
  		if(!frame.empty()){
  			msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
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

