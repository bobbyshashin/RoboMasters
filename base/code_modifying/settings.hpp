#include "manifoldGPIO.h"
#include "opencv2/core/core.hpp"
#include <string>

using namespace cv;

#define ARMOR_MODE 0
#define RUNE_MODE 1

class Settings {

public:
	struct RuneParam {
		int sudoku_cell_width;
		int sudoku_cell_height;
		int shoot_time_gap;
		int shoot_filter_size;
        RuneParam(){
            sudoku_cell_width = 143;
            sudoku_cell_height = 81;
            shoot_time_gap = 100;
            shoot_filter_size = 5;
        }
	} runeParam;

    struct ArmorParam {

        // Actual Armor Size (mm)
        int smallArmorWidth = 135; 
        int smallArmorHeight = 125; // Notice: The armor plates are installed slantways
        int bigArmorWidth = 230;
        int bigArmorHeight = 127; // Notice: The armor plates are installed slantways

        int color = 0; // 0 is blue and 1 is red
        /*
        ArmorParam(){

            smallArmorWidth = 135; 
            smallArmorHeight = 125; 
            bigArmorWidth = 230;
            bigArmorHeight = 127;

        }
        */
    } armorParam;


    Settings(){
        
    }

    void init();

   
};

