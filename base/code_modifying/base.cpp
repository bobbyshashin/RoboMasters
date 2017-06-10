#include <thread>
#include <vector>
#include <iostream>

#include "settings.hpp"
#include "ImageProcessor.hpp"


int main() {

    /* Serial Port Configuration */
    int fd = openPort("/dev/ttyTHS1"); // UART2 of DJI Manifold
    configurePort(fd);

    /* Load Configurations */
    //char* config_file = "~/RoboMasters/base/code_modifying/param_config.xml";
    //Settings setting(config_file);
    Settings setting;

    /* Start threads for vision */
    ImageProcessor imageProcessor(&setting, fd);

    std::vector<std::thread> threads; // A vector storing all threads
    threads.push_back(std::thread(&ImageProcessor::imageProducer, imageProcessor));
    threads.push_back(std::thread(&ImageProcessor::imageConsumer, imageProcessor));

    for(auto& thread : threads)
        thread.join();




    close(fd);
    return 0;


}