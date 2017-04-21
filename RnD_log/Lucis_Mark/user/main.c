#include "main.h"
#include "function_list.h"
#include "hero_param.h"

volatile u32 ticks_msimg = (u32) - 1;

int16_t LiftingMotorSetpoint[4] = {0};
enum State{StaticState, MovingState};
bool SetpointStatic = false;
enum State GimbalState; 	
volatile uint8_t upper_pneumatic_state = 0;
volatile bool lower_pneumatic_state = false;
bool lower_pneumatic_prev = false;
bool upper_pneumatic_prev = false;

void init(){

	SysTick_Init();  
	Dbus_init();//usart1
	//buzzer_init();	 //initialization of buzzer
	tft_init(2, WHITE, BLACK, BLACK);
	LED_master_init();
	gyro_init();
	ADC1_init();
	judging_system_init(); //usart3
	gyro_init();
	pneumatic_init();
	CAN1_Configuration();
	CAN2_Configuration();
	gyro_init();
	gyro_cal();
	TIM5_Int_Init(24, 13124);// 256hz //3.9xx ms for gyro usage
	DataMonitor_Init();
	ENCODER_Init();
	GUN_Init();
}

 
int16_t checkSetpoint(int16_t a, bool dir){

	if(dir) 
		if(a > LiftingMotorSetpointLimit - 200) a = LiftingMotorSetpointLimit - 1;
		else 																		a += 200;
	
	else 
		if(a < 200) 														a = 0;
		else 																		a -= 200;
	
	return a;
}

void windowLimit(int32_t* dst, int32_t upperLimit, int32_t lowerLimit) {

	if(*dst > upperLimit)
		*dst = upperLimit;
	else if(*dst < lowerLimit)
		*dst = lowerLimit;
}

int32_t buffer[4][BUFFER_LENGTH];

//int32_t inverse = 1;//if inv == 1, move forward, else if inv == -1, move backward

//The coorperation of gimbal and the chasis
//The direction is from 0 to 8192
//The gyro of chasis is ranged from 0 to 3600, so we need conversion
int32_t direction = 0;
int32_t upperTotal = 360 * 27;

int32_t xtotal = 0;
int32_t pre_xtotal = 0;

int main(void)
{	
	init();
	DBUS_ReceiveData.mouse.ytotal = 0;
	
	//init the buffers with zero 
	for (int i=0; i<4; i++)
		for (int j=0; j<BUFFER_LENGTH; j++)
			buffer[i][j] = 0;
		
	
	incPIDinit(&gimbalSpeedMoveState);
	incPIDinit(&pitchSpeedMoveState);

	incPIDset(&gimbalSpeedMoveState, 70, 3.7, 0);
	incPIDset(&pitchSpeedMoveState, 70, 3.7, 0);

	mouse_prev = DBUS_ReceiveData.mouse.xtotal;

	while (1){	

		if (ticks_msimg != get_ms_ticks()){
			ticks_msimg = get_ms_ticks();  //maximum 1000000	

			//filter_rate limit control
			if (cameraPositionId == 0 && DBUS_CheckPush(KEY_CTRL) == 0) 
				filter_rate_limit = 600;
			else 
				filter_rate_limit = 200;

			if (pressCameraChangePrev == 0 && DBUS_CheckPush(KEY_Q)){
				cameraPositionId++;
				if (cameraPositionId == 6)
					cameraPositionId = 0;
				cameraPositionSetpoint = cameraArray[cameraPositionId];
			}

			if (DBUS_ReceiveData.rc.switch_left == 1 || DBUS_ReceiveData.rc.switch_left == 3){ 

				/*******************************************************
				******************* DBUS Data Analyze ******************
				*******************************************************/

				//Analyse the data received from DBUS and transfer moving command					
				
				speed_limitor  = 660;
				speed_multiplier = filter_rate_limit;
				angular_speed_limitor = 200;
				forward_speed = (DBUS_ReceiveData.rc.ch1 + DBUS_CheckPush(KEY_W)*660 - DBUS_CheckPush(KEY_S)*660) * speed_multiplier/speed_limitor;
				right_speed =   (DBUS_ReceiveData.rc.ch0 + DBUS_CheckPush(KEY_D)*660 - DBUS_CheckPush(KEY_A)*660) * speed_multiplier/speed_limitor;

				//direction = -DBUS_ReceiveData.rc.ch2*2 + -DBUS_ReceiveData.mouse.xtotal*4 ;
				/*
				if(DBUS_ReceiveData.rc.switch_left == 1) {		//auto follow mode
					direction += (-DBUS_ReceiveData.rc.ch2/300 + -DBUS_ReceiveData.mouse.x);
					setpoint_angle = -direction * 3600/upperTotal;
					gimbalPositionSetpoint = direction + output_angle*upperTotal/3600;
					if (gimbalPositionSetpoint > 1500) gimbalPositionSetpoint = 1500;
					if (gimbalPositionSetpoint < -1500) gimbalPositionSetpoint = -1500;
				}
				*/
				if(DBUS_ReceiveData.rc.switch_left == 3 || DBUS_ReceiveData.rc.switch_left == 1){ //keyboard-mouse mode, chasis will turn if mouse go beyong the boundary
				
					xtotal =  DBUS_ReceiveData.mouse.xtotal;
 					//direction not move when the difference is large
					if (abs(direction + output_angle*upperTotal / 3600) <= outsideLimit) 
						direction += (-DBUS_ReceiveData.rc.ch2 / 300 + -(xtotal - pre_xtotal)*7);
					else if ((direction + output_angle*upperTotal / 3600) > outsideLimit)
						direction = outsideLimit - output_angle * upperTotal/3600;			
					else if ((direction + output_angle * upperTotal / 3600) < - outsideLimit)
						direction = -outsideLimit - output_angle * upperTotal / 3600;

					gimbalPositionSetpoint = direction +  output_angle*upperTotal/3600;

					if(DBUS_ReceiveData.mouse.press_right || abs(DBUS_ReceiveData.rc.ch2)>3){
						setpoint_angle = -direction * 3600/upperTotal;
					}

					//Used for protection				
					
					//windowLimit(&gimbalPositionSetpoint, 700, -700); //problem with overloading, write another function
					if(gimbalPositionSetpoint > 700)
						gimbalPositionSetpoint = 700;
					else if (gimbalPositionSetpoint < -700)
						gimbalPositionSetpoint = -700;
					
					// else gimbalPositionSetpoint=-DBUS_ReceiveData.mouse.xtotal*yawPosMultiplier;
				
					pre_xtotal = xtotal;
				}

				/*******************************************************
				*********** Chasis turing speed limit control **********
				*******************************************************/				
				feedback_angle = output_angle;
				
				output_angle_speed = pid_process(&state_angle,&setpoint_angle, &feedback_angle, kp_chassisAngle, ki_chassisAngle, kd_chassisAngle);
				windowLimit(&output_angle_speed, CHASSIS_ANGULAR_VELOCITY_LIMIT, -CHASSIS_ANGULAR_VELOCITY_LIMIT);

				int32_t max_wheel_setpoint = abs(forward_speed) + abs(right_speed);	

				int32_t larger_abs_speed = max(abs(forward_speed), abs(right_speed));
				
				wheel_setpoints[0] = (  forward_speed + right_speed) * larger_abs_speed / max_wheel_setpoint ;
				wheel_setpoints[1] = (- forward_speed + right_speed) * larger_abs_speed / max_wheel_setpoint ;
				wheel_setpoints[2] = (- forward_speed - right_speed) * larger_abs_speed / max_wheel_setpoint ;
				wheel_setpoints[3] = (  forward_speed - right_speed) * larger_abs_speed / max_wheel_setpoint ;
				//but the code above is only for the moving back and forth, left and right
								
				for (int i=0; i<4; i++){
					buffer_in(buffer[i], BUFFER_LENGTH, ticks_msimg , wheel_setpoints[i]);
					wheel_setpoints[i] = buffer_out(buffer[i], BUFFER_LENGTH, ticks_msimg);

					if(cameraPositionId == 0)
						wheel_setpoints[i] += output_angle_speed;
					else 
						wheel_setpoints[i] += DBUS_ReceiveData.mouse.x * 6;
				}	
				
				/*******************************************************
				******************** Power Control *********************
				*******************************************************/
				for(int i=0; i<4; i++)
					wheel_setpoints[i] = wheel_setpoints[i] * wheel_setpoint_coefficient / 1000;
				
				
				wheel_setpoints_adjust(&wheel_setpoints[0], &wheel_setpoints[1],&wheel_setpoints[2],&wheel_setpoints[3] , filter_rate_limit);
				
				//these are the feed back as the current state 
				wheel_feedbacks[0] = CM1Encoder.filter_rate;
				wheel_feedbacks[1] = CM2Encoder.filter_rate;
				wheel_feedbacks[2] = CM3Encoder.filter_rate;
	 			wheel_feedbacks[3] = CM4Encoder.filter_rate;
			
				//pid process to get the output as the torque
				for (int i=0; i<4; i++)
					wheel_outputs[i] = pid_process(&states[i], &wheel_setpoints[i], &wheel_feedbacks[i], kp, ki, kd);
	
				/*******************************************************
				******** Chasis turing speed limit control ends ********
				*******************************************************/

				if(DBUS_ReceiveData.rc.switch_left == 3 || DBUS_ReceiveData.rc.switch_left == 1){
					
				//position setpoint is done above
									
					isGimbalPositionSetpointIncrease = (bufferedGimbalPositionSetpoint < gimbalPositionSetpoint);

					if(isGimbalPositionSetpointIncrease){
						bufferedGimbalPositionSetpoint += 5;

						if (bufferedGimbalPositionSetpoint > gimbalPositionSetpoint)
							bufferedGimbalPositionSetpoint = gimbalPositionSetpoint;
					}
					else {
						bufferedGimbalPositionSetpoint -= 5;

						if(bufferedGimbalPositionSetpoint < gimbalPositionSetpoint) 
							bufferedGimbalPositionSetpoint = gimbalPositionSetpoint;
					}
													
				}

				/*******************************************************
				*************** Pitch setpoint control *****************
				*******************************************************/

				//limit pitch position
				windowLimit(&DBUS_ReceiveData.mouse.ytotal, -460/pitchPosMultiplier, -1100/pitchPosMultiplier);
				//pitch setpoint
				pitchPositionSetpoint = -DBUS_ReceiveData.mouse.ytotal * pitchPosMultiplier;

				isPitchPositionSetpointIncrease = (bufferedPitchPositionSetpoint < pitchPositionSetpoint);
				
				if(isPitchPositionSetpointIncrease) {
					bufferedPitchPositionSetpoint += 70;

					if (bufferedPitchPositionSetpoint > pitchPositionSetpoint)
						bufferedPitchPositionSetpoint = pitchPositionSetpoint;
				}
				else {
					bufferedPitchPositionSetpoint -= 70;

					if(bufferedPitchPositionSetpoint < pitchPositionSetpoint) 
						bufferedPitchPositionSetpoint = pitchPositionSetpoint;
				}
						
				/*******************************************************
				************ Yaw & Pitch velocity control **************
				*******************************************************/			
				
				gimbalPositionFeedback = GMYawEncoder.ecd_angle;
				gimbalSpeedSetpoint = (int32_t)fpid_process(&gimbalPositionState, &bufferedGimbalPositionSetpoint, &gimbalPositionFeedback, kp_gimbalPosition, ki_gimbalPosition, kd_gimbalPosition);

				pitchPositionFeedback = GMPitchEncoder.ecd_angle;
				pitchSpeedSetpoint = (int32_t)fpid_process(&pitchPositionState, &pitchPositionSetpoint, &pitchPositionFeedback, kp_pitchPosition, ki_pitchPosition, kd_pitchPosition);

				cameraPositionFeedback = GMCameraEncoder.ecd_angle;

				//Limit the output
				windowLimit(&gimbalSpeedSetpoint, 80, -80);
				windowLimit(&pitchSpeedSetpoint, 80, -80);
				//mock speed here
				//gimbalSpeedSetpoint = DBUS_ReceiveData.rc.ch2 * 0.5;

				//Get the speed here
				incPIDsetpoint(&gimbalSpeedMoveState, gimbalSpeedSetpoint);
				gimbalSpeedMoveOutput += incPIDcalc(&gimbalSpeedMoveState, GMYawEncoder.filter_rate);
				
				incPIDsetpoint(&pitchSpeedMoveState, pitchSpeedSetpoint);
				pitchSpeedMoveOutput += incPIDcalc(&pitchSpeedMoveState, GMPitchEncoder.filter_rate);

				/*******************************************************
				******************* Camera control *********************
				*******************************************************/	
				cameraPositionFeedback = GMCameraEncoder.ecd_angle;
				cameraPositionOutput = fpid_process(&cameraPositionState, &cameraPositionSetpoint, &cameraPositionFeedback, kp_cameraPosition, ki_cameraPosition, kd_cameraPosition);

				cameraSpeedSetpoint = (int32_t) cameraPositionOutput;

				cameraSpeedFeedback = GMCameraEncoder.filter_rate;
				cameraSpeedOutput = pid_process(&cameraSpeedState, &cameraSpeedSetpoint, &cameraSpeedFeedback, kp_cameraSpeed, ki_cameraSpeed, kd_cameraSpeed);

				//call the acturater function
				//if (ticks_msimg % 20 == 0)
				GUN_PokeControl();

		  	Set_CM_Speed(CAN1, gimbalSpeedMoveOutput,pitchSpeedMoveOutput,gunSpeed,cameraSpeedOutput);						 
				Set_CM_Speed(CAN2, wheel_outputs[0], wheel_outputs[1], wheel_outputs[2], wheel_outputs[3]);
			}	
			
			/*
			else if (DBUS_ReceiveData.rc.switch_left == 3) { //Also the stop mode now
				Set_CM_Speed(CAN1,0,0,0,0);
				Set_CM_Speed(CAN2,0,0,0,0);
			} 
			*/
		
				if(ticks_msimg % 20 == 0){
			
					if (DBUS_CheckPush(KEY_SHIFT)) { //SHIFT is pressed

						if(DBUS_CheckPush(KEY_R)){
							LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = 0;
							DataMonitor_Send(5, 0);
						}
						else if(DBUS_CheckPush(KEY_Z)){
							LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = checkSetpoint(LiftingMotorSetpoint[0], true);
							DataMonitor_Send(0, LiftingMotorSetpoint[0]);
						}
						else if(DBUS_CheckPush(KEY_X)){
							LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = checkSetpoint(LiftingMotorSetpoint[0], false);
							DataMonitor_Send(0, LiftingMotorSetpoint[0]);
						}
						else if(DBUS_CheckPush(KEY_C)){
							LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = checkSetpoint(LiftingMotorSetpoint[2], true);
							DataMonitor_Send(2, LiftingMotorSetpoint[2]);
						}
						else if(DBUS_CheckPush(KEY_V)){
							LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = checkSetpoint(LiftingMotorSetpoint[2], false);
							DataMonitor_Send(2, LiftingMotorSetpoint[2]);
						}
						else if(!lower_pneumatic_prev && DBUS_CheckPush(KEY_F)){
							//pneumatic
							lower_pneumatic_state = !lower_pneumatic_state;
							pneumatic_control(1, lower_pneumatic_state);
							pneumatic_control(2, lower_pneumatic_state);
						}
						else if(!upper_pneumatic_prev && DBUS_CheckPush(KEY_E)){
							//pneumatic
							if(upper_pneumatic_state == 0){
								DataMonitor_Send(16, 0);
								upper_pneumatic_state = 1;
								pneumatic_control(3, true);
							}
							else if(upper_pneumatic_state == 1){
								DataMonitor_Send(17, 0);
								upper_pneumatic_state = 2;
							}
							else if(upper_pneumatic_state == 2){
								DataMonitor_Send(18,0);
								upper_pneumatic_state = 0;
								pneumatic_control(3, false);
							}
						}
					
					}

					else { //SHIFT is not pressed
						if(DBUS_CheckPush(KEY_R)){
							LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = UP_SETPOINT/8;
							DataMonitor_Send(0xFF, LiftingMotorSetpoint[0]);        //GO_ON_STAGE_ONE_KEY
						}
						else if(DBUS_CheckPush(KEY_F)){
							LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = UP_SETPOINT/8;
							LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = MID_SETPOINT/8;
							DataMonitor_Send(0xFE, LiftingMotorSetpoint[0]);        //GO_DOWN_STAGE_ONE_KEY
						}
						//else if(DBUS_CheckPush(KEY_A)){
							//DataMonitor_Send(0xFD,LiftingMotorSetpoint[0]);   //BREAK
						//}
						else if(DBUS_CheckPush(KEY_X)){
							LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = DOWN_SETPOINT/8;
							DataMonitor_Send(0xFC, LiftingMotorSetpoint[0]);		//ONE_KEY_DOWN_FRONT					
						}
						else if(DBUS_CheckPush(KEY_V)){
							LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = DOWN_SETPOINT/8;
							DataMonitor_Send(0xFB, LiftingMotorSetpoint[2]);		//ONE_KEY_DOWN_BACK					
						}
						else if(DBUS_CheckPush(KEY_Z)){
							LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = UP_SETPOINT/8;
							DataMonitor_Send(0xFA, LiftingMotorSetpoint[0]);		//ONE_KEY_UP_FRONT						
						}
						else if(DBUS_CheckPush(KEY_C)){
							LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = UP_SETPOINT/8;
							DataMonitor_Send(0xF9, LiftingMotorSetpoint[2]);		//ONE_KEY_UP_BACK					
						}	
					
					}
					
					lower_pneumatic_prev = DBUS_CheckPush(KEY_SHIFT) && DBUS_CheckPush(KEY_F);
					upper_pneumatic_prev = DBUS_CheckPush(KEY_SHIFT) && DBUS_CheckPush(KEY_E);
					
					//for(uint8_t i=2; i<12; i++)
					//	tft_clear_line(i);
					//for(uint8_t i=0; i<4; i++){
					//	tft_prints(1, i+2, "LMP %d %d", i, LiftingMotorSetpoint[i]);
					//}
					
					tft_clear();
					for(uint8_t i=0; i<4; i++)
						tft_prints(1, i+2, "%d %d", i,wheel_feedbacks[i]);

					//tft_prints(1, 6, "yback=%.1f", gimbalPositionFeedback);
					//tft_prints(1, 7, "pback=%.1f", pitchPositionFeedback);
					//tft_prints(1, 8, "cback-%.1f", cameraPositionFeedback);
					//tft_prints(1, 7, "gyro:%d", output_angle);

					tft_prints(1, 5, "camSpS: %d", cameraSpeedSetpoint);
					tft_prints(1, 6, "camSpf:%d", GMCameraEncoder.filter_rate);
					tft_prints(1, 7, "camPst:%.1f", cameraPositionSetpoint);
					tft_prints(1, 8, "camPsf:%.1f", GMCameraEncoder.ecd_angle);

					tft_update();
					
				}
				
				//Set_CM_Speed(CAN1, 0, 0, 0, 0);
				//Set_CM_Speed(CAN2, 0, 0, 0, 0);
			//}		

			if(DBUS_ReceiveData.rc.switch_left == 2){
				Set_CM_Speed(CAN1,0,0,0,0);
				Set_CM_Speed(CAN2,0,0,0,0);
			}
			
			if ( ticks_msimg % 20 == 0 ){
				//tft_prints(1,9,"mode:%d", DBUS_ReceiveData.rc.switch_left );
				//tft_update();
			}

			pressCameraChangePrev = DBUS_CheckPush(KEY_Q);	
			
		} //main loop with ticks	
	}
	
	
} //main
