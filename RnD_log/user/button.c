#include "button.h"
#include "buzzer_song.h"

void button_init(void){
	
	BUTTON_init(JOY_BOTTOM);
	BUTTON_init(JOY_UP);
	BUTTON_init(JOY_RIGHT);
	BUTTON_init(JOY_LEFT);
	BUTTON_init(JOY_MIDDLE);

}

void button_check(){

	if(gpio_read_input(JOY_BOTTOM)==0){
		FAIL_MUSIC;
	}
	else if(gpio_read_input(JOY_LEFT)==0){
		FAIL_MUSIC;
	}
	else if(gpio_read_input(JOY_RIGHT)==0){
		FAIL_MUSIC;
	}
	else if(gpio_read_input(JOY_UP)==0){
		FAIL_MUSIC;
	}
	else if(gpio_read_input(JOY_MIDDLE)==0){
		FAIL_MUSIC;
	}
}