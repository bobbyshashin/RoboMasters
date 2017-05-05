#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Vector3.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/gpu/gpu.hpp"

using namespace cv;
using namespace std;

ros::Publisher centerPub;
int wtf = 0;

int main( int argc, char** argv ) {

    ros::init(argc, argv, "circle_detector");
    ros::NodeHandle nh("~");
    
    centerPub = nh.advertise<geometry_msgs::Vector3>("/target_position", 5);
    VideoCapture cap(0); //capture the video from web cam

    if ( !cap.isOpened() ) { // if not success, exit program

        cout << "Cannot open the web cam" << endl;
        return -1;
    }

    Mat imgOriginal;
    Mat imgHSV;
    Mat imgThresholded_cpu;

    Mat mask1, mask2;
    ros::Time begin; // Timer
    ros::Rate loop_rate(10);

    while(ros::ok()) {
	
	begin = ros::Time::now();
        bool bSuccess = cap.read(imgOriginal); // read a new frame from video

        if (!bSuccess) { //if not success, break loop

             cout << "Cannot read a frame from video stream" << endl;
             break;
        }
	
	// 640 * 480 for now
	//cout << "Height: " << imgOriginal.rows << endl;
	//cout << "Width: " << imgOriginal.cols << endl;

	cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);

 	inRange(imgHSV, Scalar(0, 50, 70), Scalar(15, 255, 255), mask1);
	inRange(imgHSV, Scalar(165, 50, 70), Scalar(180, 255, 255), mask2);

	imgThresholded_cpu = mask1 | mask2;

	gpu::GpuMat d_src(imgThresholded_cpu);
        gpu::GpuMat d_dst;

	gpu::erode(d_src, d_dst, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
	gpu::dilate(d_dst, d_src, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

	gpu::dilate(d_src, d_dst, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)) );
	gpu::erode(d_dst, d_src, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)) );
	
	Mat imgThresholded(d_src);

	vector<Vec3f> circles;
	HoughCircles( imgThresholded, circles, CV_HOUGH_GRADIENT, 1, imgThresholded.rows/4, 200, 32, 0, 0 );

	cout << "Number of circles detected: " << circles.size() << endl;
	for( size_t i = 0; i < circles.size(); i++ ) {

	    geometry_msgs::Vector3 c;
	    c.x = circles[i][0];
  	    c.y = circles[i][1];
	    centerPub.publish(c);

   	    Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
   	    int radius = cvRound(circles[i][2]);
   	    // circle center
   	    circle( imgOriginal, center, 3, Scalar(0,255,0), -1, 8, 0 );
   	    // circle outline
   	    circle( imgOriginal, center, radius, Scalar(0,0,255), 3, 8, 0 );
 	 }
 	 
         imshow("Thresholded Image", imgThresholded_cpu); //show the thresholded image
         imshow("Original", imgOriginal); //show the original image

         if (waitKey(1) == 27) { //wait for 'esc' key press for 1ms. If 'esc' key is pressed, break loop
        	 cout << "esc key is pressed by user" << endl;
        	 break;
         }
	 // loop_rate.sleep();
	 ros::spinOnce();
	 cout << "Loop time: " << ros::Time::now() - begin << endl;
    }

    return 0;

}
