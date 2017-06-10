#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#include <ctime>

#include "manifoldGPIO.h"

using namespace cv;
using namespace cv::gpu;

using namespace std;
using namespace std::chrono;

#define FALSE -1
#define TRUE   0

long delayTime = 1 * 1000;

short CVSerialDataX = -888;
short CVSerialDataY = 999;
int fd = 0;
;
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

class TrackedEllipse {
public:
    RotatedRect shape;
    double area;
    TrackedEllipse(double area, RotatedRect shape) : shape(shape), area(area) {}
};

long currentTime() {
	return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

int main() {
    VideoCapture stream(0);
    if (!stream.isOpened()) {
        cout << "Failed to open webcam." << endl;
    }
    serialSetup();
    serialStart();

    //namedWindow("test");

    GPIO toggleButton = gpio157;
    gpioExport(toggleButton);
    gpioSetDirection(toggleButton, input);

    int keycode;
    unsigned int detectRedShield = low;

    long now = currentTime(), timeElapsed;
    cout << "Letting system initiate for 5 seconds..." << endl;
    while ((timeElapsed = currentTime() - now) <= delayTime) if (timeElapsed % 500 == 0) cout << ".";
    cout << endl;

    while ((keycode = waitKey(30)) != 27) {
	gpioGetValue(toggleButton, &detectRedShield);

        Mat cameraFrame;
        stream.read(cameraFrame);

        Mat mask;
        //Mat mask_cpu;
	cvtColor(cameraFrame, cameraFrame, COLOR_RGB2HSV);
        inRange(cameraFrame, Scalar(0, 1, 160), Scalar(49, 255, 255), mask); 
        cvtColor(cameraFrame, cameraFrame, COLOR_HSV2RGB);
        cvtColor(mask, mask, CV_GRAY2BGR); 

        bitwise_and(cameraFrame, mask, cameraFrame);

        cvtColor(mask, mask, COLOR_BGR2GRAY);

        threshold(mask, mask, 100, 255, THRESH_TOZERO); // Threshold Value: 100, Max Value: 255
        Mat structuringElement = getStructuringElement(MORPH_RECT, Size(2, 2));
        
        //gpu::GpuMat d_src(mask_cpu);
        //gpu::GpuMat d_dst;

        morphologyEx(mask, mask, MORPH_OPEN, structuringElement, Point(-1, 1), 2);
        morphologyEx(mask, mask, MORPH_CLOSE, structuringElement, Point(-1, -1), 2);

	//gpu::morphologyEx(d_src, d_dst, MORPH_OPEN, structuringElement, Point(-1, 1), 2);
        //gpu::morphologyEx(d_dst, d_src, MORPH_CLOSE, structuringElement, Point(-1, -1), 2);

        //Mat mask(d_src);

        std::vector<vector<Point> > contours;
        findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        drawContours(mask, contours, -1, Scalar(255, 255, 255), CV_FILLED);
        vector<double> areas;
        vector<TrackedEllipse> ellipses;
        for (int i = 0; i < contours.size(); i++) {
            vector<Point> contour = contours[i];
             if (contour.size() > 5) {
                RotatedRect shape = fitEllipse(Mat(contour));
               //cout << shape.size.width / shape.size.height << endl;
                if (true) {  // shape.angle <= 85
			std::ostringstream ratioT;
			ratioT << "[" << shape.size.width / shape.size.height << "]" << " [" << shape.angle << "]";
                      //putText(cameraFrame, ratioT.str(), shape.center, CV_FONT_HERSHEY_COMPLEX, 1, Scalar(0, 0, 255));
                areas.push_back(contourArea(Mat(contour)));
                ellipses.push_back(TrackedEllipse(contourArea(Mat(contour)),
                                                  shape));
                }
            }
        }
        Mat thresholdedFrame = Mat(cameraFrame.rows, cameraFrame.cols, CV_8UC3, Scalar(0, 0, 0));
        if (ellipses.size() >= 2) {
            int maxAreaIndex;
            minMaxIdx(Mat(areas), 0, 0, 0, &maxAreaIndex);

            // Get possible candidate ellipses who have an angle difference of <15.
            vector<TrackedEllipse> candidateEllipses;
            for (int i = 0; i < ellipses.size(); i++) {
                double angleDifference =
                        180.0 -
                        fabs(fmod(fabs(ellipses[maxAreaIndex].shape.angle - ellipses[i].shape.angle), 360.0) - 180.0);
		double areaDifference = abs(ellipses[maxAreaIndex].shape.size.area() - ellipses[i].shape.size.area());
		double maxEllipseRatio = ellipses[maxAreaIndex].shape.size.width / ellipses[maxAreaIndex].shape.size.height;
		double candidateEllipseRatio = ellipses[i].shape.size.width / ellipses[i].shape.size.height;
                if ((angleDifference < 15 || angleDifference > 170)) {
                    candidateEllipses.push_back(ellipses[i]);
                }
            }
            if (candidateEllipses.size() >= 2) {
                Mat testFrame;
                cvtColor(cameraFrame, testFrame, CV_BGR2GRAY);
                // Sort list based on angle difference.
                sort(candidateEllipses, [&ellipses, &maxAreaIndex](TrackedEllipse a, TrackedEllipse b) {
                    double angleDifferenceA =
                            180.0 - fabs(fmod(fabs(ellipses[maxAreaIndex].shape.angle - a.shape.angle), 360.0) - 180.0);
                    double angleDifferenceB =
                            180.0 - fabs(fmod(fabs(ellipses[maxAreaIndex].shape.angle - b.shape.angle), 360.0) - 180.0);
                    return angleDifferenceA < angleDifferenceB;
                });
                Point midpoint = (candidateEllipses[0].shape.center + candidateEllipses[1].shape.center) * 0.5;
                ellipse(thresholdedFrame, candidateEllipses[0].shape, Scalar(255, 255, 255), CV_FILLED);
                ellipse(thresholdedFrame, candidateEllipses[1].shape, Scalar(255, 255, 255), CV_FILLED);

                CVSerialDataX = midpoint.x - 320;
                CVSerialDataY = 240 - midpoint.y;
            } else {
		CVSerialDataX = -888;
		CVSerialDataY = 999;
	    }
        } else { CVSerialDataX = -888; CVSerialDataY = 999; }
        bitwise_and(cameraFrame, thresholdedFrame, thresholdedFrame);
        Scalar meanColor = mean(thresholdedFrame);

	bool shieldIsRed = meanColor[2] > meanColor[0]; // BGR format.
	// If there is a candidate shield target available but the color is wrong, stop aiming.
	if (CVSerialDataX != -888 && CVSerialDataY != 999) {
		if (shieldIsRed && !detectRedShield || !shieldIsRed && detectRedShield || (meanColor[0] == 0 && meanColor[2] == 0)) {
			CVSerialDataX = -888;
			CVSerialDataY = 999;
		}
	}
       cout << "Mean Color: " << meanColor << " " << (shieldIsRed ? "RED" : "BLUE") << " (" << (detectRedShield ? "DETECTING RED" : "DETECTING BLUE") << ") " << "[" << CVSerialDataX << ", " << CVSerialDataY << "]" << endl;
        circle(thresholdedFrame, Point(CVSerialDataX + 320, 240 - CVSerialDataY), 5, Scalar(255, 255, 255));
        imshow("Webcam", cameraFrame);
        imshow("Filter", thresholdedFrame);
    }
    gpioUnexport(toggleButton);
    UART0_Close(fd);
    return 0;
}
