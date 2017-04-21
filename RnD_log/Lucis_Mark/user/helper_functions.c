#include "helper_functions.h"
//The buffer functions 
int32_t buffer_out(int32_t* b, int32_t length, int32_t counter){
	int32_t b_output = 0;
	for (int i = 0 ; i< length ; i++){
		b_output+= b[i];
	}
	b_output = b_output/ length;
	return b_output;
}

void buffer_in(int32_t* b , int32_t length, int32_t counter, int32_t input ){
	int16_t index = counter%length ;
	b[index] = input;
}

//The adjustment of the set points
int32_t abs(int32_t x){
	if (x < 0){
			return -x;
	}
	return x;
}

int32_t max(int32_t a, int32_t b){

	if(a > b)
		return a;
	else
		return b;

}

int32_t min(int32_t a, int32_t b){

	if(a < b)
		return a;
	else
		return b;

}

void wheel_setpoints_adjust(int32_t * sp1, int32_t* sp2, int32_t* sp3, int32_t* sp4, int32_t limit){
	int32_t max = abs(*sp1);
	if (abs(*sp2) > max) max = abs(*sp2);
	if (abs(*sp3) > max) max = abs(*sp3);
	if (abs(*sp4) > max) max = abs(*sp4);

	if (max > limit){ 
			*sp1 = *sp1 * limit  / max;
			*sp2 = *sp2 * limit  / max;
			*sp3 = *sp3 * limit  / max;
			*sp4 = *sp4 * limit  / max;			
	}
}	
