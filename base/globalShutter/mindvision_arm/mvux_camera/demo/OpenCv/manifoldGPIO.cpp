// GPIO.c

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include "manifoldGPIO.h"

//
// gpioExport
// Export the given gpio to userspace;
// Return: Success = 0 ; otherwise open file error
int gpioExport ( GPIO gpio )
{
    int fd, length;
    char commandBuffer[MAX_BUF];

    fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
    if (fd < 0) {

        std::cout << "Unable to export GPIO" << gpio << std::endl;
        return fd;
    }

    length = snprintf(commandBuffer, sizeof(commandBuffer), "%d", gpio);
    if (write(fd, commandBuffer, length) != length) {
        perror("gpioExport");
        return fd;

    }
    close(fd);

    return 0;
}

//
// gpioUnexport
// Unexport the given gpio from userspace
// Return: Success = 0 ; otherwise open file error
int gpioUnexport ( GPIO gpio )
{
    int fd, length;
    char commandBuffer[MAX_BUF];

    fd = open(SYSFS_GPIO_DIR "/unexport", O_WRONLY);
    if (fd < 0) {
        
	std::cout << "Unable to unexport GPIO" << gpio << std::endl;
        return fd;
    }

    length = snprintf(commandBuffer, sizeof(commandBuffer), "%d", gpio);
    if (write(fd, commandBuffer, length) != length) {
        perror("gpioUnexport");
        return fd;
    }
    close(fd);
    return 0;
}

// gpioSetDirection
// Set the direction of the GPIO pin 
// Return: Success = 0 ; otherwise open file error
int gpioSetDirection ( GPIO gpio, unsigned int out_flag )
{
    int fd;
    char commandBuffer[MAX_BUF];

    snprintf(commandBuffer, sizeof(commandBuffer), SYSFS_GPIO_DIR  "/gpio%d/direction", gpio);

    fd = open(commandBuffer, O_WRONLY);
    if (fd < 0) {

        std::cout << "Unable to set direction for GPIO" << gpio << std::endl;
        return fd;
    }

    if (out_flag) {
        if (write(fd, "out", 4) != 4) {
            perror("gpioSetDirection");
            return fd;
        }
    }
    else {
        if (write(fd, "in", 3) != 3) {
            perror("gpioSetDirection");
            return fd;
        }
    }
    close(fd);
    return 0;
}

//
// gpioSetValue
// Set the value of the GPIO pin to 1 or 0
// Return: Success = 0 ; otherwise open file error
int gpioSetValue ( GPIO gpio, unsigned int value )
{
    int fd;
    char commandBuffer[MAX_BUF];

    snprintf(commandBuffer, sizeof(commandBuffer), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

    fd = open(commandBuffer, O_WRONLY);

    if (fd < 0) {

        std::cout << "Unable to set value for GPIO" << gpio << std::endl;
        return fd;
    }

    if (value) {
        if (write(fd, "1", 2) != 2) {
            perror("gpioSetValue");
            return fd;
        }
    }
    else {
        if (write(fd, "0", 2) != 2) {
            perror("gpioSetValue");
            return fd;
        }
    }
    close(fd);
    return 0;
}

//
// gpioGetValue
// Get the value of the requested GPIO pin ; value return is 0 or 1
// Return: Success = 0 ; otherwise open file error
int gpioGetValue ( GPIO gpio, unsigned int *value)
{
    int fd; //File Descriptor
    char commandBuffer[MAX_BUF];
    char ch;

    snprintf(commandBuffer, sizeof(commandBuffer), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

    fd = open(commandBuffer, O_RDONLY);
    if (fd < 0) {

        std::cout << "Unable to get value for GPIO" << gpio << std::endl;
        return fd;
    }

    if (read(fd, &ch, 1) != 1) {
        perror("gpioGetValue");
        return fd;
     }

    if (ch != '0') {
        *value = 1;
    } else {
        *value = 0;
    }

    close(fd);
    return 0;
}


//
// gpioSetEdge
// Set the edge of the GPIO pin
// Valid edges: 'none' 'rising' 'falling' 'both'
// Return: Success = 0; otherwise open file error
int gpioSetEdge ( GPIO gpio, char *edge )
{
    int fd; // File Descriptor
    char commandBuffer[MAX_BUF];

    snprintf(commandBuffer, sizeof(commandBuffer), SYSFS_GPIO_DIR "/gpio%d/edge", gpio);

    fd = open(commandBuffer, O_WRONLY);
    if (fd < 0) {

        std::cout << "Unable to get edge for GPIO" << gpio << std::endl;
        return fd;
    }

    if (write(fd, edge, strlen(edge) + 1) != ((int)(strlen(edge) + 1))) {
        perror("gpioSetEdge");
        return fd;
    }
    close(fd);
    return 0;
}

//
// gpioOpen
// Open the given pin for reading
// Returns the file descriptor of the named pin
int gpioOpen( GPIO gpio )
{
    int fd;
    char commandBuffer[MAX_BUF];

    snprintf(commandBuffer, sizeof(commandBuffer), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

    fd = open(commandBuffer, O_RDONLY | O_NONBLOCK );
    if (fd < 0) {
        
	std::cout << "Unable to open GPIO" << gpio << std::endl;
    }
    return fd;
}

//
// gpioClose
// Close the given file descriptor 
int gpioClose ( int fd )
{
    return close(fd);
}

// gpioActiveLow
// Set the active_low attribute of the GPIO pin to 1 or 0
// Return: Success = 0 ; otherwise open file error
int gpioActiveLow ( GPIO gpio, unsigned int value )
{
    int fd; //File Descriptor
    char commandBuffer[MAX_BUF];

    snprintf(commandBuffer, sizeof(commandBuffer), SYSFS_GPIO_DIR "/gpio%d/active_low", gpio);

    fd = open(commandBuffer, O_WRONLY);
    if (fd < 0) {
        char errorBuffer[128];
        snprintf(errorBuffer,sizeof(errorBuffer), "gpioActiveLow unable to open gpio%d",gpio);
        perror(errorBuffer);
        return fd;
    }

    if (value) {
        if (write(fd, "1", 2) != 2) {
            perror("gpioActiveLow");
            return fd;
        }
    }
    else {
        if (write(fd, "0", 2) != 2) {
            perror("gpioActiveLow");
            return fd;
        }
    }
    close(fd);
    return 0;
}

