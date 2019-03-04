# Tag Detector
ROS Package in C++ to detect AprilTags
___________________________________________


### Functions

A dataset of 2 videos with a Mosaic of AprilTags in provided. This ROS package consists of 2 nodes:  

**Node 1**  
 - Reads from the video file and publishes to the ROS Topic _/image_raw_.  
 - Provides the functionality of adjusting the blur size and hence the intensit of Blur that could be applied to the image via a ROS service server named _/set_blur_window_size_.  

 **Node 2**  
 - Subscribes to the topic _/image_raw_ and reads the image being published by the first node.
 - After reading, the image's sharpness is evaluated. This is done so by computing a Fourier Transform of the image and then indirectly calculating the number of high frequency components from the histogram of the image's FFT. This number acts as a metric to evaluate sharpness of the image. It was experimentally found that for unblurred orignal image, the value is very low, 500< whereas, for blurred images, this value drastically increases to about >2000
 - If the image is classified to be blurry by an arbitrary threshold, a DeBlur filter is applied. The deblur filter is implemented based on "High Boost filtering".  The blur image is made to pass through High pass filter and the output is blended with the blurred image. This improves the image's sharpness. This process is done by convolving the image with a custom kernel which is equivalent to the process of first high pass filtering and then blending.
 - The Square blocks of April Tags are detected by the following process:
  	- Median Blurring
  	- Histogram Equalization.
  	- Image thresholding and binarization
  	- Canny Edge detection
  	- Dilation
  	- Contour Detection and Polynomial Approximation
  	- Fitting rectangle and filtering the generated rectangles based on their area and number of corners.
  	- Draw Squares.
  - The computation time of each frame is tracked by using the getTickCount() function of std library.
  - All the highlights are displayed using ROS_INFO.
  - All the nodes are wrapped using a single Launch file.



### Instructions

Clone the package in your catkin workspace. In a terminal window, go to your workspace and do a catkin_make. 
Assuming the name of your catkin workspace is _catkin_ws_,

```sh
$ cd ~/catkin_ws
$ catkin_make
```

This should build the ROS package on your system. The Data foleady has 2 videder alro files. The ROS package must be able to run out of the box.  
Source the setup.bash files.
```sh
$ source ~/catkin_ws/devel/setup.bash
```

To run the launch file, 
```sh
$ roslaunch tag_detector tag_detector.launch  video:=1
```

To run the second video, change the argument to 2.
```sh
$ roslaunch tag_detector tag_detector.launch  video:=2
```

In order to introduce Gaussian Blur in the input video, this can be done by calling the ROS service _/set_blur_window_size_. For ex, To set window size to (3, 3), open a new terminal, (make sure the setup.bash is sourced in every terminal you open) and send the following command:
```sh
$ rosservice call /set_blur_window_size 3 3
```
The terminal running the ROS node must show an acknowledgment about setting of parameters along with other information such as Computation time per frame, Image quality info and if deblurring is applied to the orignal image.

