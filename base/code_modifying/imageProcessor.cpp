#include "imageProcessor.hpp"

//#define USE_VIDEO

#define VIDEO_WIDTH 640
#define VIDEO_HEIGHT 480
#define BUFFER_SIZE 1

Mat imgData[BUFFER_SIZE];
Mat imgTemp;

using namespace cv;
using namespace std;

/* HSV Parameters for Red/Blue Armor Detection */
Scalar redLowerRange(80, 100, 30);
Scalar redUpperRange(140, 255, 255);

Scalar blueLowerRange(0, 100, 30);
Scalar blueUpperRange(30, 255, 255);


/* Member Functions */
void imageProcessor::imageProducer(){

#ifdef USE_VIDEO


#else // use video stream from camera
	VideoCapture cap(0);
	if(!cap.isOpened()) {
		cout << "Default camera cannot be opened" << endl;
		return;
	}

#endif

	while(1) {


		cap >> imgTemp;


	}








}


void imageProcessor::imageConsumer(){

	ArmorDetector armorDetector(_settings);


	while(1) { // Main loop for armor detection






	}








}