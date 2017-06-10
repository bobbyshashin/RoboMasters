#include "serial.h"
#include <iostream>

int openPort(const char* devName) {

	int fd; // File descriptor
	fd = open(devName, O_RDWR | O_NOCTTY | O_NDELAY);

	if(fd == -1) // Failed to open
		std::cout << "Failed to open " << devName << endl;
	else {

		fcntl(fd, F_SETFL, 0);
		std::cout << "Port is open" << endl;
	}

	return fd;

}


int configurePort(int fd) {

	struct termios port_settings;
	cfsetispeed(&port_settings, B115200); // set baud rate
	cfsetospeed(&port_settings, B115200); // set baud rate

	port_settings.c_cflag &= ~PARENB;           // set no parity, stop bits, data bits
    port_settings.c_cflag &= ~CSTOPB;
    port_settings.c_cflag &= ~CSIZE;
    port_settings.c_cflag |= CS8;

    tcsetattr(fd, TCSANOW, &port_settings);     // apply the settings to the port

    return(fd);

} 


bool send(int fd, double* data) {







}