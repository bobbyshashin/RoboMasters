#include <iostream>
#include <opencv2/opencv.hpp>
//#include <opencv2/gpu/gpu.hpp>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#include <ctime>
#include <time.h>
#include <sys/time.h>
#include <thread>

#include "manifoldGPIO.h"

using namespace cv;
using namespace std;
//using namespace cv::gpu;

using std::cout;
using std::endl;

#define FALSE -1
#define TRUE   0

#ifndef SHOW_IMG
//#define SHOW_IMG
#endif

short CVSerialDataX = -888;
short CVSerialDataY = 999;
int fd = 0;

Mat cameraFrame;
unsigned int detectRedShield = low;
bool lastShieldValue = detectRedShield;

struct timeval tv_begin;
struct timeval tv_end;

int UART0_Open(int fd, char *port) {
    fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (FALSE == fd) return (FALSE);
    if (fcntl(fd, F_SETFL, 0) < 0) return (FALSE);
    if (0 == isatty(STDIN_FILENO)) return (FALSE);
    return fd;
}

void UART0_Close(int fd) {
    close(fd);
}

int UART0_Init(int fd, int speed, int flow_ctrl, int databits, int stopbits, int parity) {
    int i;
    int status;
    int speed_arr[] = {
            B115200,
            B19200,
            B9600,
            B4800,
            B2400,
            B1200,
            B300
    };
    int name_arr[] = {
            115200,
            19200,
            9600,
            4800,
            2400,
            1200,
            300
    };
    struct termios options;

    if (tcgetattr(fd, &options) != 0) return (FALSE);

    for (i = 0; i < sizeof(speed_arr) / sizeof(int); i++) {
        if (speed == name_arr[i]) {
            cfsetispeed(&options, speed_arr[i]);
            cfsetospeed(&options, speed_arr[i]);
        }
    }

    options.c_cflag |= CLOCAL;

    options.c_cflag |= CREAD;

    switch (flow_ctrl) {

        case 0:
            options.c_cflag &= ~CRTSCTS;
            break;

        case 1:
            options.c_cflag |= CRTSCTS;
            break;
        case 2:
            options.c_cflag |= IXON | IXOFF | IXANY;
            break;
    }

    options.c_cflag &= ~CSIZE;
    switch (databits) {
        case 5:
            options.c_cflag |= CS5;
            break;
        case 6:
            options.c_cflag |= CS6;
            break;
        case 7:
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag |= CS8;
            break;
        default:
            return (FALSE);
    }

    switch (parity) {
        case 'n':
        case 'N':
            options.c_cflag &= ~PARENB;
            options.c_iflag &= ~INPCK;
            break;
        case 'o':
        case 'O':
            options.c_cflag |= (PARODD | PARENB);
            options.c_iflag |= INPCK;
            break;
        case 'e':
        case 'E':
            options.c_cflag |= PARENB;
            options.c_cflag &= ~PARODD;
            options.c_iflag |= INPCK;
            break;
        case 's':
        case 'S':
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;
            break;
        default:
            return (FALSE);
    }

    switch (stopbits) {
        case 1:
            options.c_cflag &= ~CSTOPB;
            break;
        case 2:
            options.c_cflag |= CSTOPB;
            break;
        default:
            return (FALSE);
    }

    options.c_oflag &= ~OPOST;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    options.c_cc[VTIME] = 1;
    options.c_cc[VMIN] = 1;

    tcflush(fd, TCIFLUSH);

    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        perror("com set error!\n");
        return (FALSE);
    }
    return (TRUE);
}

int UART0_Send(int fd, char *send_buf, int data_len) {
    int len = 0;

    len = write(fd, send_buf, data_len);
    if (len == data_len)
        return len;
    else {
        tcflush(fd, TCOFLUSH);
        return FALSE;
    }
}

void serialSetup() {
    int err;
    char port[] = "/dev/ttyTHS1";

    fd = UART0_Open(fd, port);
    do {
        err = UART0_Init(fd, 115200, 0, 8, 1, 'N');
    } while (FALSE == err || FALSE == fd);
}

void serialSend() {
    
	char real_output[4];

	real_output[0] = CVSerialDataX >>8;
	real_output[1] = (char)CVSerialDataX;
	real_output[2] = CVSerialDataY >>8;
	real_output[3] = (char)CVSerialDataY;
	


 //   char output[10] = "";
  //  sprintf(output, "%02X%04X%04X", 0, (unsigned short)CVSerialDataX, (unsigned short)CVSerialDataY);
    
    int len = UART0_Send(fd, real_output, 4);
    
    char printData0x[15] = "";
    char printDataDe[15] = "";
    if (len > 0) {
        //sprintf(printData0x, "0x: %04X, %04X", (unsigned short)CVSerialDataX, (unsigned short)CVSerialDataY);
        //sprintf(printDataDe, "De: %04d, %04d", CVSerialDataX, CVSerialDataY);
        
        printData0x[14] = '\0';
        printDataDe[14] = '\0';
        
        //printf("%s\t\t", printData0x);
        //printf("%s\n", printDataDe);
    }
}


void handle(union sigval v) {
    time_t t;
    char p[32];

    time(&t);
    strftime(p, sizeof(p), "%T", localtime(&t));

//    printf("%s\t", p);

    serialSend();
}

void serialStart() {
    struct sigevent evp;
    struct itimerspec ts;
    timer_t timer;
    int ret;

    memset(&evp, 0, sizeof(evp));
    evp.sigev_value.sival_ptr = &timer;
    evp.sigev_notify = SIGEV_THREAD;
    evp.sigev_notify_function = handle;
    evp.sigev_value.sival_int = 3; //as a parameter for handle()

    ret = timer_create(CLOCK_REALTIME, &evp, &timer);

    ts.it_interval.tv_sec = 0;
    ts.it_interval.tv_nsec = 10000000; //transmitting time interval in nanosecond
    ts.it_value.tv_sec = 3;
    ts.it_value.tv_nsec = 0;

    ret = timer_settime(timer, TIMER_ABSTIME, &ts, NULL);
}

std::vector<KeyPoint> keypoints;


const string trackbars[6] = {"LH", "LS", "LV", "UH", "US", "UV"};
const string windowTitle = "Robomasters";

Scalar redLowerRange(80, 100, 30);
Scalar redUpperRange(140, 255, 255);

Scalar blueLowerRange(0, 100, 30);
Scalar blueUpperRange(30, 255, 255);

int thresh = 0;

class TrackedEllipse {
public:
    RotatedRect shape;
    double area;

    TrackedEllipse(double area, RotatedRect shape) : shape(shape), area(area) {}
};

void syncTrackbars() {
    for (int i = 0; i < 6; i++)
        if (detectRedShield)
            if (i < 3) redLowerRange[i % 3] = getTrackbarPos(trackbars[i], windowTitle);
            else redUpperRange[i % 3] = getTrackbarPos(trackbars[i], windowTitle);
        else
            if (i < 3) blueLowerRange[i % 3] = getTrackbarPos(trackbars[i], windowTitle);
            else blueUpperRange[i % 3] = getTrackbarPos(trackbars[i], windowTitle);
}

void processImg(){

    int keycode;

    GPIO toggleButton = gpio157;
    gpioExport(toggleButton);
    gpioSetDirection(toggleButton, input);
    gpioGetValue(toggleButton, &detectRedShield);

    gpioGetValue(toggleButton, &detectRedShield);
    while((keycode = waitKey(10)) != 27){

        gettimeofday(&tv_begin,NULL);
        if (lastShieldValue != detectRedShield) {
            for (int i = 0; i < 6; i++) {
                setTrackbarPos(trackbars[i], windowTitle, i < 3 ? detectRedShield ? redLowerRange.val[i % 3] : blueLowerRange.val[i % 3] : detectRedShield ? redUpperRange.val[i % 3] : blueUpperRange.val[i % 3]);
            }
            lastShieldValue = detectRedShield;
        }

        keycode = keycode & 0xFF;

        Mat hsvFrame;
    
        cvtColor(cameraFrame, hsvFrame, COLOR_RGB2HSV);

        inRange(hsvFrame, detectRedShield ? redLowerRange : blueLowerRange, detectRedShield ? redUpperRange : blueUpperRange,
                hsvFrame);
        cvtColor(hsvFrame, hsvFrame, COLOR_GRAY2RGB);
        bitwise_and(cameraFrame, hsvFrame, hsvFrame);

        Mat mask;
        cvtColor(hsvFrame, mask, COLOR_RGB2GRAY);

        syncTrackbars();

        Mat structuringElement = getStructuringElement(MORPH_RECT, Size(2, 2));
        morphologyEx(mask, mask, MORPH_OPEN, structuringElement, Point(-1, -1), 2);
        morphologyEx(mask, mask, MORPH_CLOSE, structuringElement, Point(-1, -1), 2);

        std::vector<vector<Point> > contours;
        findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

        drawContours(mask, contours, -1, Scalar(255), CV_FILLED);

        vector<double> areas;
        vector<TrackedEllipse> ellipses;

        for (int i = 0; i < contours.size(); i++) {
            vector<Point> contour = contours[i];
            if (contour.size() >= 5) {
                areas.push_back(contourArea(Mat(contour)));
                ellipses.push_back(TrackedEllipse(contourArea(Mat(contour)),
                                                  fitEllipse(Mat(contour))));
            }
        }

        Mat thresholdedFrame = Mat(cameraFrame.rows, cameraFrame.cols, CV_8UC3, Scalar(0, 0, 0));

        if (ellipses.size() >= 2) {

            double maxArea;
            int maxAreaIndex;

            minMaxIdx(Mat(areas), 0, &maxArea, 0, &maxAreaIndex);

            vector<TrackedEllipse> candidateEllipses;

            // Get possible candidate ellipses who have an angle difference of <10
            for (int i = 0; i < ellipses.size(); i++) {
                double angleDifference =
                        180.0 -
                        fabs(fmod(fabs(ellipses[maxAreaIndex].shape.angle - ellipses[i].shape.angle), 360.0) - 180.0);
                if (angleDifference < 15 || angleDifference > 170) {
                    candidateEllipses.push_back(ellipses[i]);
                }
            }

            if (candidateEllipses.size() >= 2) {
                // Sort list based on distance and area and angle difference.

                sort(candidateEllipses, [&ellipses, &maxAreaIndex](TrackedEllipse a, TrackedEllipse b) {
                    double angleDifferenceA =
                            180.0 - fabs(fmod(fabs(ellipses[maxAreaIndex].shape.angle - a.shape.angle), 360.0) - 180.0);
                    double angleDifferenceB =
                            180.0 - fabs(fmod(fabs(ellipses[maxAreaIndex].shape.angle - b.shape.angle), 360.0) - 180.0);
                    return angleDifferenceA < angleDifferenceB;
                });

        sort(candidateEllipses, [&ellipses, &maxAreaIndex](TrackedEllipse a, TrackedEllipse b) {
            return a.area > b.area;
        });

                Point midpoint = (candidateEllipses[0].shape.center + candidateEllipses[1].shape.center) * 0.5;
                ellipse(thresholdedFrame, candidateEllipses[0].shape, Scalar(255, 255, 255), CV_FILLED);
                ellipse(thresholdedFrame, candidateEllipses[1].shape, Scalar(255, 255, 255), CV_FILLED);

                CVSerialDataX = midpoint.x - 320;
                CVSerialDataY = 240 - midpoint.y;
            } else {
                CVSerialDataX = -899;
                CVSerialDataY = 999;
            }
        } else {
            CVSerialDataX = -899;
            CVSerialDataY = 999;
        }

        bitwise_and(cameraFrame, thresholdedFrame, thresholdedFrame);
        Scalar meanColor = mean(thresholdedFrame);

        // If there is a candidate shield target available but the color is wrong, stop aiming.
       cout << (detectRedShield ? "DETECTING RED" : "DETECTING BLUE") << " " << "[" << CVSerialDataX << ", " << CVSerialDataY << "]" << endl;

        circle(thresholdedFrame, Point(CVSerialDataX + 320, 240 - CVSerialDataY), 5, Scalar(255, 255, 255));
#ifdef SHOW_IMG
        imshow("HSV", hsvFrame);
        moveWindow("HSV",0,0);
        imshow("Webcam", cameraFrame);
        moveWindow("Webcam",0,600);
        imshow("Filter", mask);
        moveWindow("Filter",400,0);
#endif
        imshow("Final", thresholdedFrame);
        moveWindow("Final",400,600);
        moveWindow(windowTitle,1000,0);
    
        gettimeofday(&tv_end,NULL);
    
        double time_diff = (tv_end.tv_usec-tv_begin.tv_usec)/1000;
        cout << "Loop time: " << time_diff << endl;
    }
}

void grabImg(){

    VideoCapture stream(0);

    if (!stream.isOpened()) {
        cout << "Failed to open webcam." << endl;
    }
    stream.read(cameraFrame);


}

int main() {
    setUseOptimized(true);
    cout << checkHardwareSupport(CV_CPU_SSE2);
    namedWindow(windowTitle);

    for (int i = 0; i < 6; i++) {
        createTrackbar(trackbars[i], windowTitle, 0, 255);
        setTrackbarPos(trackbars[i], windowTitle, i < 3 ? detectRedShield ? redLowerRange.val[i % 3] : blueLowerRange.val[i % 3] : detectRedShield ? redUpperRange.val[i % 3] : blueUpperRange.val[i % 3]);
    }

    serialSetup();
    serialStart();
    std::vector<std::thread> threads; // A vector storing all threads
    threads.push_back(std::thread(processImg));
    threads.push_back(std::thread(grabImg));
    
    for(auto& thread : threads)
        thread.join();
        
    gpioUnexport(toggleButton);
    UART0_Close(fd);

    return 0;
}

