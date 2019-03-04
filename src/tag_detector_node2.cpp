/**
* @brief: ROS Node for reading Images and Detecting April Tags.
* @details: ROS node to read frames from a ROS topic /image_raw.
* @author: Darshan Shah
* @date: March 4th 2019.
*
*/

#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <iostream>
#include <image_transport/image_transport.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

using namespace std;
using namespace cv;

void detectRect(Mat& image);
static double angle(Point pt1, Point pt2, Point pt0);
static void drawSquares(Mat& image, const vector<vector<Point> >& squares);
double evaluateImg(Mat image);
double calcMean(Mat& img);
void deblur(Mat& blurredImg);

int qualityThreshold = 2500;


void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
	int64 t0, t1;
    double Sharpness;
	vector<double> timeRecord;
  	double sec;
	try {
		Mat rec_img = cv_bridge::toCvShare(msg, "bgr8")->image;
    	// imshow("Orignal Image", rec_img);
    	// waitKey(500);
    	t0 = getTickCount();
        Sharpness = evaluateImg(rec_img); //Evaluate Image Quality.
        if (Sharpness > qualityThreshold) {
            ROS_INFO("Quality Measure = %lf | Image is Excessively BLURRED. Applying Deblur Filter.", Sharpness);
            deblur(rec_img);
        } else {
            ROS_INFO("Quality Measure = %lf | Image is Sharp enough!", Sharpness);
        }
    	detectRect(rec_img);
    	t1 = getTickCount();
    	imshow("Tag Detected", rec_img);
    	waitKey(3);
    	sec = (t1-t0)/getTickFrequency();
    	timeRecord.push_back(sec);
    	ROS_INFO("Computation time for Detection Tags: %lf", sec);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  	}
  	// double averageTime = accumulate( timeRecord.begin(), timeRecord.end(), 0.00)/timeRecord.size();
  	// ROS_INFO("Average COmputation time was %lf per image", averageTime); 

}


int main(int argc, char** argv) {
	ros::init(argc, argv, "image_listener");
	
	ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/image_raw", 1, imageCallback);
    ros::spin();
    // destroyAllWindows();
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
}

double evaluateImg(Mat img){
    cvtColor(img, img, cv::COLOR_RGB2GRAY);
    int M = getOptimalDFTSize( img.rows ); //Get Row and column sizes
    int N = getOptimalDFTSize( img.cols );
    Mat padded; 
    copyMakeBorder(img, padded, 0, M - img.rows, 0, N - img.cols, BORDER_CONSTANT, Scalar::all(0)); //Padd the image

    Mat planes[] = {Mat_<float>(padded), Mat::zeros(padded.size(), CV_32F)};
    Mat complexImg;
    merge(planes, 2, complexImg);

    dft(complexImg, complexImg);

    split(complexImg, planes);
    magnitude(planes[0], planes[1], planes[0]);
    Mat mag = planes[0];
    double FFT_mean = calcMean(mag); //If the image is blurry, it has high amount of Low frequency 
                                   //components. Hence the mean is high. This is used as a measure of Sharpness.
    return FFT_mean; 
}

double calcMean(Mat& img){
    int v_bins = 50;
    int histSize[] = { v_bins };
    Mat hist;
    float v_ranges[] = { 0, 255 };
    const float* ranges[] = { v_ranges };
    calcHist(&img, 1, 0, cv::Mat(), hist, 1, histSize, ranges, true, false); //histogram calculation
    Scalar mean, stddev;
    meanStdDev(hist, mean, stddev);
    double hist_mean = mean[0];
    return hist_mean;
    // std::cout << "Mean: " << mean[0] << "   StdDev: " << stddev[0] << std::endl;
}

void deblur(Mat& img){
    int c = 4;
    int p = 8*c+1;
    Mat kernel = (Mat_<int>(3,3)<< -c,-c,-c,-c,p,-c,-c,-c,-c);
    filter2D(img, img, -1 , kernel, Point(-1,-1), 0, BORDER_DEFAULT );
}