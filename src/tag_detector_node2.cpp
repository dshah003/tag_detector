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

void detectRect(Mat& image);
static double angle(Point pt1, Point pt2, Point pt0);
static void drawSquares(Mat& image, const vector<vector<Point> >& squares);

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
	try {
		Mat tagDetected;
		Mat rec_img = cv_bridge::toCvShare(msg, "bgr8")->image;
    	// imshow("view", rec_img);
    	// waitKey(5);
    	detectRect(rec_img);
    	imshow("Tag Detected", rec_img);
    	waitKey(3);
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

void detectRect(Mat& ColorImage)
{   
    Mat greyImg, canny_output, binImg, Sqr_img;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    vector<vector<Point> > squares;
    vector<Point> approx;

    int thresh = 100;

    cvtColor(ColorImage, greyImg, cv::COLOR_RGB2GRAY);
    equalizeHist(greyImg, greyImg);  //Makes the image a bit invariant to illumination.
    
    threshold(greyImg, binImg, 127, 255, 0);
    Canny(binImg, canny_output, 100, thresh*2 );
    dilate(canny_output, canny_output, Mat(), Point(-1,-1));
    
    findContours(canny_output, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE );
    for( size_t i = 0; i< contours.size(); i++ ) {
        approxPolyDP(contours[i], approx, arcLength(contours[i], true)*0.02, true);
        if( approx.size() == 4 &&
                    fabs(contourArea(approx)) > 1000 && 
                    isContourConvex(approx))
                {
                    double maxCosine = 0;
                    for( int j = 2; j < 5; j++ )
                    {
                        // find the maximum cosine of the angle between joint edges
                        double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }

                    if( maxCosine < 0.3 )
                        squares.push_back(approx);
                }

        drawSquares(ColorImage, squares);
    }
}

static double angle(Point pt1, Point pt2, Point pt0) {
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

static void drawSquares( Mat& image, const vector<vector<Point> >& squares )
{
    for( size_t i = 0; i < squares.size(); i++ )
    {
        const Point* p = &squares[i][0];
        int n = (int)squares[i].size();
        polylines(image, &p, &n, 1, true, Scalar(0,255,0), 3, LINE_AA);
    }
    // imshow("Squares", image);
}