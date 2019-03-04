/**
 * @brief      : ROS Node for reading Images and Detecting April Tags.
 * @author     : Darshan Shah
 * @date       : March 4th 2019.
 *
 * @section DESCRIPTION This file is a ROS node file to implement node 2 of the package.
 * Includes features like image subscriber, image evaluation, deblur and Tag detection
 */

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <vector>
#include <iostream>

using namespace std;
using namespace cv;

void detectRect(Mat& image);
static double angle(Point pt1, Point pt2, Point pt0);
static void drawSquares(Mat& image, const vector<vector<Point> >& squares);
double evaluateImg(Mat image);
double calcMean(Mat& img);
void deblur(Mat& blurredImg);

int qualityThreshold = 2500;


/**
 * @brief      Callback funtion for the Image Subscriber. Converts ROS image to
 *             OpenCV readable image format. and controls the flow of the rest
 *             of the program.
 *
 * @param[in]  msg   Pointer to the image received by the subscriber.
 */
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
	int64 t0, t1;
	double Sharpness;
	double sec;
	try {
	Mat rec_img = cv_bridge::toCvShare(msg, "bgr8")->image;  // Convert images from ROS to OpenCV readable format.
    // imshow("Orignal Image", rec_img);
    // waitKey(500);
    t0 = getTickCount();  // Start timer to compute time of execution.
    Sharpness = evaluateImg(rec_img);  // Evaluate Image Quality.
    if (Sharpness > qualityThreshold) {
    	ROS_INFO("Quality Measure = %lf | Image is Excessively BLURRED. Applying Deblur Filter.", Sharpness);
        deblur(rec_img); // Deblur the image if it's too blur
    } else {
        ROS_INFO("Quality Measure = %lf | Image is Sharp enough!", Sharpness);
    }
	detectRect(rec_img);
	t1 = getTickCount(); // Stop timer
	imshow("Tag Detected", rec_img);
	waitKey(3);
	sec = (t1-t0)/getTickFrequency(); // Calculate the computation time
	ROS_INFO("Computation time for Detection Tags: %lf", sec);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  	}

}


/**
 * @brief      Detects Rectangle/Square 2D Tags.
 *
 * @param      ColorImage  The color image
 */
void detectRect(Mat& ColorImage)
{   
    Mat greyImg, canny_output, binImg, Sqr_img;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    vector<vector<Point> > squares;
    vector<Point> approx;

    int thresh = 100;

    cvtColor(ColorImage, greyImg, cv::COLOR_RGB2GRAY);
    equalizeHist(greyImg, greyImg);  // Makes the image a bit invariant to illumination.
    
    threshold(greyImg, binImg, 127, 255, 0); // Thresholding to Create a Binary Image.
    Canny(binImg, canny_output, 100, thresh*2 ); // Canny Edge Detector
    dilate(canny_output, canny_output, Mat(), Point(-1,-1)); // Dilate the detected edges to remove any discontinuities.
    
    findContours(canny_output, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE );
    for( size_t i = 0; i< contours.size(); i++ ) {
        approxPolyDP(contours[i], approx, arcLength(contours[i], true)*0.02, true); // Approximating a curve into set of lines.
        if( approx.size() == 4 &&
                    fabs(contourArea(approx)) > 700 && 
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

/**
 * @brief      Finds the Maximum Cosine angle betwwen the joint edges given.
 *
 * @param[in]  pt1   The point 1 of Joint Edge
 * @param[in]  pt2   The point 2 of Joint Edge
 * @param[in]  pt0   The point 0 of Joint Edge.
 *
 * @return     { Maxumum Cosine angle }
 */
static double angle(Point pt1, Point pt2, Point pt0) {
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

/**
 * @brief      Draws square Boundary around the detected 2D tags..
 *
 * @param      image    The color image
 * @param[in]  squares  The vector consisting of square coordinates
 */
static void drawSquares( Mat& image, const vector<vector<Point> >& squares )
{
    for( size_t i = 0; i < squares.size(); i++ )
    {
        const Point* p = &squares[i][0];
        int n = (int)squares[i].size();
        polylines(image, &p, &n, 1, true, Scalar(0,255,0), 3, LINE_AA);  // Draw files connecting the set of points
    }
}


/**
 * @brief      Applies DFT to evaluate Sharpness of an Image.
 *
 * @param[in]  img   The image
 *
 * @return     { Returns the mean of the Histogram of FFT which is corelated to
 *             the sharpness of the image. }
 */
double evaluateImg(Mat img){
    cvtColor(img, img, cv::COLOR_RGB2GRAY);
    int M = getOptimalDFTSize( img.rows ); //Get Row and column sizes
    int N = getOptimalDFTSize( img.cols );
    Mat padded; 
    copyMakeBorder(img, padded, 0, M - img.rows, 0, N - img.cols, BORDER_CONSTANT, Scalar::all(0)); // Padd the image

    Mat planes[] = {Mat_<float>(padded), Mat::zeros(padded.size(), CV_32F)};
    Mat complexImg;
    merge(planes, 2, complexImg);

    dft(complexImg, complexImg);

    split(complexImg, planes);
    magnitude(planes[0], planes[1], planes[0]);
    Mat mag = planes[0];
    double FFT_mean = calcMean(mag); // If the image is blurry, it has high amount of Low frequency 
                                     // components. Hence the mean is high. This is used as a measure of Sharpness.
    return FFT_mean; 
}


/**
 * @brief      Computes the histogram of the magnnitude matrix and estimates
 *             Mean and Std deviation from the histogram.
 *
 * @param      img   The Magnitude component of the FFT
 *
 * @return     The mean.
 */
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

/**
 * @brief      Deblurs the image using High Boost filtering technique. 
 *
 * @param      img   The image to deblur.
 */
void deblur(Mat& img){
    int c = 4;
    int p = 8*c+1;
    Mat kernel = (Mat_<int>(3,3)<< -c,-c,-c,-c,p,-c,-c,-c,-c);
    filter2D(img, img, -1 , kernel, Point(-1,-1), 0, BORDER_DEFAULT );
}

/**
 * @brief      Main function. Initializes the ROS Node handle, Subscriber and
 *             consists of ROS spin.
 */
int main(int argc, char** argv) {
	ros::init(argc, argv, "image_listener");
	
	ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/image_raw", 1, imageCallback);
    ros::spin();
    // destroyAllWindows();
}