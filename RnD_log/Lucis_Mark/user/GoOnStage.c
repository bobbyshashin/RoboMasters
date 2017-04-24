
#include "GoOnStage.h"


void readFeedback(){
      LiftingMotorSpeedFeedback[0] = CM1Encoder.filter_rate;
			LiftingMotorSpeedFeedback[1] = CM2Encoder.filter_rate;
			LiftingMotorSpeedFeedback[2] = CM3Encoder.filter_rate;
			LiftingMotorSpeedFeedback[3] = CM4Encoder.filter_rate;
			LiftingMotorPositionFeedback[0] = CM1Encoder.ecd_angle;
			LiftingMotorPositionFeedback[1] = CM2Encoder.ecd_angle;
			LiftingMotorPositionFeedback[2] = CM3Encoder.ecd_angle;
			LiftingMotorPositionFeedback[3] = CM4Encoder.ecd_angle;
}

void speedProcess(){
    for(uint8_t i=0;i<4;i++){
        LiftingMotorSpeedSetpoint[i] = (int32_t)fpid_process(&LiftingMotorPositionState[i], &LiftingMotorPositionSetpoint[i], &LiftingMotorPositionFeedback[i],LMpos_kp,LMpos_ki,LMpos_kd );
        
    }
    
    
    
    
    for(uint8_t i=0;i<4;i++){
        if(LiftingMotorSpeedSetpoint[i]>SPEED_SETPOINT_LIMIT) LiftingMotorSpeedSetpoint[i]=SPEED_SETPOINT_LIMIT;
        else if(LiftingMotorSpeedSetpoint[i]<-SPEED_SETPOINT_LIMIT) LiftingMotorSpeedSetpoint[i]=-SPEED_SETPOINT_LIMIT;
    }
    
    
    for (int i =0;i<4;i++){
        if (LiftingMotorSpeedSetpointBuffered[i]<LiftingMotorSpeedSetpoint[i]) LiftingMotorSpeedSetpointBuffered[i]+=5;
        else if (LiftingMotorSpeedSetpointBuffered[i]>LiftingMotorSpeedSetpoint[i]) LiftingMotorSpeedSetpointBuffered[i]-=5;
    }
    
    
    
    for(uint8_t i=0;i<4;i++){
        if(LiftingMotorSpeedSetpointBuffered[i]>SPEED_SETPOINT_LIMIT) LiftingMotorSpeedSetpointBuffered[i]=SPEED_SETPOINT_LIMIT;
        else if(LiftingMotorSpeedSetpointBuffered[i]<-SPEED_SETPOINT_LIMIT) LiftingMotorSpeedSetpointBuffered[i]=-SPEED_SETPOINT_LIMIT;
    }
    
    
    for (uint8_t i = 0 ; i < 4 ; i++){
        LiftingMotorOutput[i] = pid_process(&LiftingMotorState[i], &LiftingMotorSpeedSetpointBuffered[i], &LiftingMotorSpeedFeedback[i], kp,ki,kd);
    }
    
    for (uint8_t i = 0 ; i < 4 ; i++){
        if (LiftingMotorOutput[i]>30000) LiftingMotorOutput[i]=30000;
        if (LiftingMotorOutput[i]<-30000) LiftingMotorOutput[i]=-30000;
    }
    
}

void setSetpoint(){
    if(BREAK && GO_ON_STAGE_ONE_KEY) GO_ON_STAGE_ONE_KEY=false;
    if(BREAK && GO_DOWN_STAGE_ONE_KEY) GO_DOWN_STAGE_ONE_KEY=false;
    if(!GO_ON_STAGE_ONE_KEY_PREV && GO_ON_STAGE_ONE_KEY) {
        //ticks_msimg_on_prev=ticks_msimg;			//starting counting time for the procedure
        for(uint8_t i=0;i<4;i++)
            LiftingMotorPositionSetpoint[i]=UP_SETPOINT;
        GO_ON_STAGE_ONE_KEY=false;
    }
    if(!GO_DOWN_STAGE_ONE_KEY_PREV && GO_DOWN_STAGE_ONE_KEY) {
        //ticks_msimg_down_prev=ticks_msimg;			//starting counting time for the procedure
        LiftingMotorPositionSetpoint[2]=LiftingMotorPositionSetpoint[3]=MID_SETPOINT;
        LiftingMotorPositionSetpoint[0]=LiftingMotorPositionSetpoint[1]=UP_SETPOINT;
        GO_DOWN_STAGE_ONE_KEY=false;
    }
    if(ONE_KEY_UP_FRONT){LiftingMotorPositionSetpoint[0]=LiftingMotorPositionSetpoint[1]=UP_SETPOINT;}
    else if(ONE_KEY_DOWN_FRONT){LiftingMotorPositionSetpoint[0]=LiftingMotorPositionSetpoint[1]=DOWN_SETPOINT;}
    if(ONE_KEY_UP_BACK){LiftingMotorPositionSetpoint[2]=LiftingMotorPositionSetpoint[3]=UP_SETPOINT;}
    else if(ONE_KEY_DOWN_BACK){LiftingMotorPositionSetpoint[2]=LiftingMotorPositionSetpoint[3]=DOWN_SETPOINT;}
				
    
    GO_ON_STAGE_ONE_KEY_PREV=GO_ON_STAGE_ONE_KEY;			//don't forget to update
    GO_DOWN_STAGE_ONE_KEY_PREV=GO_DOWN_STAGE_ONE_KEY;			//don't forget to update
    

}
