#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

 int main( int argc, char** argv )
 {
    VideoCapture cap(0); //capture the video from web cam

    if ( !cap.isOpened() ) { // if not success, exit program

         cout << "Cannot open the web cam" << endl;
         return -1;
    }

    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

    int iLowH = 160;
    int iHighH = 179;

    int iLowS = 50;
    int iHighS = 255;

    int iLowV = 0;
    int iHighV = 255;

    int iBlurSize = 30;

    // Create trackbars in "Control" window
    cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control", &iHighH, 179);

    cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control", &iHighS, 255);

    cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Control", &iHighV, 255);

    cvCreateTrackbar("BlurSize", "Control", &iBlurSize, 100);

    while (true) {

        Mat imgOriginal;

        bool bSuccess = cap.read(imgOriginal); // read a new frame from video

         if (!bSuccess) { //if not success, break loop

             cout << "Cannot read a frame from video stream" << endl;
             break;
        }

         //flip and blur the raw data
         Mat imgBlur, imgFlip;
         Size ksize; ksize.width = max(iBlurSize, 1); ksize.height = max(iBlurSize, 1);
         flip(imgOriginal, imgFlip, 1);
         blur(imgFlip, imgBlur, ksize);

         //convert to HSV
		 Mat imgHSV;
         cvtColor(imgBlur, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

         // thresholding
         Mat imgThresholded;
         inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

         //morphological opening (remove small objects from the foreground)
         erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
         dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

         //morphological closing (fill small holes in the foreground)
         dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
         erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		/*
         Moments oMoments = moments(imgThresholded);
         double dM01 = oMoments.m01;
         double dM10 = oMoments.m10;
         double dArea = oMoments.m00;
         if (dArea > 10000) {
        	 int posX = dM10 / dArea;
        	 int posY = dM01 / dArea;
        	 circle(imgThresholded, Point(posX, posY), 5, Scalar(0, 0, 255), -1);
        	 circle(imgFlip, Point(posX, posY), 5, Scalar(0, 0, 255), -1);
         }
		*/
         // find boundary
         vector< vector< Point > > contours;
         Mat contourOutput = imgThresholded.clone();
         findContours(contourOutput, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

         Mat imgContour(imgThresholded.size(), CV_8UC3, Scalar(0, 0, 0));
         Scalar colors[3];
         colors[0] = cv::Scalar(255, 0, 0);
         colors[1] = cv::Scalar(0, 255, 0);
         colors[2] = cv::Scalar(0, 0, 255);
         for (size_t idx = 0; idx < contours.size(); idx++) {
        	 if (contourArea(contours[idx]) > 300)
        		 cv::drawContours(imgContour, contours, idx, colors[idx % 3]);
         }

         imshow("Contour Image", imgContour); //show the boundary of all the large white area
         imshow("Thresholded Image", imgThresholded); //show the thresholded image
         imshow("Original", imgFlip); //show the original image

         if (waitKey(30) == 27) { //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
        	 cout << "esc key is pressed by user" << endl;
        	 break;
         }
    }

    return 0;

}
