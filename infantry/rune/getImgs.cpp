#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;
using namespace cv;

int marginPixel = 4;
int thresh = 182;
int max_thresh = 255;
RNG rng(12345); // random number generator

Rect cropRect(Rect rect, Point tl, Point br, int x_offset_tl, int y_offset_tl, int x_offset_br, int y_offset_br) {

	return Rect( Point(tl.x + x_offset_tl, tl.y + y_offset_tl), Point(br.x + x_offset_br, br.y + y_offset_br) );

}

Rect cropRect(Rect rect, Point tl, Point br) {
	return Rect( tl, br
}

void getImgs(Mat original_img, vector<Mat>* single_imgs) {

    Mat img = original_img;
 
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

    cout << "Contours: " << contours.size() << endl;

    for( int i = 0; i < contours.size(); i++ ) {  
        approxPolyDP( contours[i], contours_polys[i], 3, true );
        boundRects[i] = boundingRect( Mat(contours_polys[i]) );
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
	rectangle( img, Point(boundRect[i].tl().x + marginPixel, boundRect[i].tl().y + marginPixel), Point(boundRect[i].br().x - marginPixel, boundRect[i].br().y - marginPixel), Scalar(255, 255, 255), -1, 8, 0 );
	rectangle(img, cropRect(boundRect[i], 
    }

    cvtColor(img, img, CV_GRAY2BGR);
    //cvtColor(original_img, original_img, CV_BGR2GRAY);
    bitwise_and(original_img, img, img);
	
	/* Convert one whole image to 9 same image */
	vector<Mat> single_imgs(boundRect.size());

    ofstream myFile;
	myFile.open("rects.txt");

	for(int i = 0; i < single_imgs.size(); i++) {
		Rect square( Point(boundRect[i].tl().x + marginPixel, boundRect[i].tl().y + marginPixel), Point(boundRect[i].br().x - marginPixel, boundRect[i].br().y - marginPixel) );
		//square.height = square.width;
		cout << "w:"<< square.width << " h:"<< square.height << " s:" << square.size();
		//single_imgs[i] = Mat(square.size(), img.type(), Scalar::all(0));
		img(square).copyTo(single_imgs[i]);
		resize(single_imgs[i], single_imgs[i], Size(28*4, 28*4));
		for(int j=0; j<single_imgs[i].cols * single_imgs[i].rows; j++) {
			myFile << (short)single_imgs[i].data[j];
		}
		myFile << "\n";
		cout << "Size "<< i <<": "<< single_imgs[i].size() << boundRect[i].size()  <<endl;
	}
    myFile.close();
    //bitwise_not(img, img);  
}

int main(int argc, char** argv )
{
    
    Mat original_img;
    original_img = imread("1.bmp");
    resize(original_img, original_img, Size(480, 640));
    Mat img = original_img;
 
    Size size(9,9);  
    GaussianBlur(img,img,size,0);
    cvtColor(img, img, CV_BGR2GRAY);  

    Canny(img, img, thresh, thresh*2, 3);

    Mat structuringElement = getStructuringElement(MORPH_RECT, Size(8, 8));
    dilate(img, img, structuringElement);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    findContours( img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    
    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );

    cout << "Contours: " << contours.size() << endl;

    for( int i = 0; i < contours.size(); i++ ) {  
        approxPolyDP( contours[i], contours_poly[i], 3, true );
        boundRect[i] = boundingRect( Mat(contours_poly[i]) );
     }

    /* Check Rect Height-Width Ratio */
    for(int i = 0; i < boundRect.size(); i++) {
		int height = boundRect[i].height;
		int width = boundRect[i].width;
		cout << "Rect " << i << "Ratio = " << (float)height / (float)width << endl;
		cout << "Centre " << i << " (" << boundRect[i].x << "," << boundRect[i].y << ")" << endl;
    }

    //Scalar color = Scalar( rng.uniform(255, 255), rng.uniform(255,255), rng.uniform(255,255) );
    Scalar color = Scalar( 255, 255, 255 );
    //fillPoly(img, boundRect, color);
    for(int i = 0; i < contours.size(); i++) {
		rectangle( img, Point(boundRect[i].tl().x + marginPixel, boundRect[i].tl().y + marginPixel), Point(boundRect[i].br().x - marginPixel, boundRect[i].br().y - marginPixel), color, -1, 8, 0 );
        //drawContours( img, contours, i, color, 2, 8, hierarchy, 0, Point() );
    }

    cvtColor(img, img, CV_GRAY2BGR);
    //cvtColor(original_img, original_img, CV_BGR2GRAY);
    bitwise_and(original_img, img, img);
	
	/* Convert one whole image to 9 same image */
	vector<Mat> single_imgs(boundRect.size());

    ofstream myFile;
	myFile.open("rects.txt");

	for(int i = 0; i < single_imgs.size(); i++) {
		Rect square( Point(boundRect[i].tl().x + marginPixel, boundRect[i].tl().y + marginPixel), Point(boundRect[i].br().x - marginPixel, boundRect[i].br().y - marginPixel) );
		//square.height = square.width;
		cout << "w:"<< square.width << " h:"<< square.height << " s:" << square.size();
		//single_imgs[i] = Mat(square.size(), img.type(), Scalar::all(0));
		img(square).copyTo(single_imgs[i]);
		resize(single_imgs[i], single_imgs[i], Size(28*4, 28*4));
		for(int j=0; j<single_imgs[i].cols * single_imgs[i].rows; j++) {
			myFile << (short)single_imgs[i].data[j];
		}
		myFile << "\n";
		cout << "Size "<< i <<": "<< single_imgs[i].size() << boundRect[i].size()  <<endl;
	}
    myFile.close();
    //bitwise_not(img, img);  
    if ( !img.data )
    {
        printf("No image data \n");
        return -1;
    }
    namedWindow("Original Image", WINDOW_AUTOSIZE );
    imshow("Original Image", original_img);
    namedWindow("Display Image", WINDOW_AUTOSIZE );
    imshow("Display Image", img);

	
	for(int i = 0; i < single_imgs.size(); i++) {
		char num = i - '0';
		namedWindow("single_img "+num, WINDOW_AUTOSIZE );
		imshow("single_img "+num, single_imgs[i]);
	}

    waitKey(0);

    return 0;
}
