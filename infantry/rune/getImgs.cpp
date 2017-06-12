#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;
using namespace cv;

int offset_x = 0;
int offset_y = 0;
int thresh = 182;
int max_thresh = 255;
RNG rng(12345); // random number generator

void getImgs(Mat original_img, Mat& img, vector<Mat>*& single_imgs) {

    img = original_img;
 
    GaussianBlur(img, img, Size(9,9) ,0);
    cvtColor(img, img, CV_BGR2GRAY);  

    Canny(img, img, thresh, thresh*2, 3);

    Mat structuringElement = getStructuringElement(MORPH_RECT, Size(8, 8));
    dilate(img, img, structuringElement);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    findContours( img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    
    vector<vector<Point> > contours_polys( contours.size() );
    vector<Rect> boundRects( contours.size() );
	vector<Rect> croppedBoundRects( contours.size());

    cout << "Contours: " << contours.size() << endl;

    for( int i = 0; i < contours.size(); i++ ) {  
        approxPolyDP( contours[i], contours_polys[i], 3, true );
        boundRects[i] = boundingRect( Mat(contours_polys[i]) );
		croppedBoundRects[i] = boundRects[i] + Size(offset_x, offset_y);	
     }

    /* Check Rect Height-Width Ratio */
    for(int i = 0; i < boundRects.size(); i++) {
		int height = boundRects[i].height;
		int width = boundRects[i].width;
		cout << "Rect " << i << "Ratio = " << (float)height / (float)width << endl;
		cout << "Centre " << i << " (" << boundRects[i].x << "," << boundRects[i].y << ")" << endl;
    }


    //fillPoly(img, boundRect, color);
    for(int i = 0; i < contours.size(); i++) {
		rectangle( img, croppedBoundRects[i], Scalar(255, 255, 255), -1, 8, 0 );
    }

    cvtColor(img, img, CV_GRAY2BGR);
    //cvtColor(original_img, original_img, CV_BGR2GRAY);
    bitwise_and(original_img, img, img);
	
	/* Convert one whole image to 9 same image */
	single_imgs = new vector<Mat>(croppedBoundRects.size());
	for(int i = 0; i < single_imgs->size(); i++) {
		img(croppedBoundRects[i]).copyTo(single_imgs->at(i));
		resize(single_imgs->at(i), single_imgs->at(i), Size(28*4, 28*4));
		cout << "Size "<< i <<": "<< single_imgs->at(i).size() << boundRects[i].size()  <<endl;
	}
}

int main(int argc, char** argv )
{
    
    Mat original_img, img;
    original_img = imread("1.bmp");
    if ( !original_img.data )
    {
        printf("No orignal image data \n");
        return -1;
    }
    resize(original_img, original_img, Size(480, 640));
	vector<Mat>* single_imgs;	
	getImgs(original_img, img,  single_imgs);	
    namedWindow("Original Image", WINDOW_AUTOSIZE );
    imshow("Original Image", original_img);
    namedWindow("Pre-processed Image", WINDOW_AUTOSIZE );
    imshow("Pre-processed Image", img);

	for(int i = 0; i < single_imgs->size(); i++) {
		if ( !single_imgs->at(i).data) {
			printf("No single image data \n");
			return -1;
		}
		char num = i - '0';
		namedWindow("single_img "+num, WINDOW_AUTOSIZE );
		imshow("single_img "+num, single_imgs->at(i));
	}

    waitKey(0);

    return 0;
}
