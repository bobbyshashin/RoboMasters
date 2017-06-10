#include "settings.hpp"

void Settings::init(){

	GPIO toggleButton = gpio157;
    gpioExport(toggleButton);
    gpioSetDirection(toggleButton, input);
    gpioGetValue(toggleButton, &armorParam.color);

    


}