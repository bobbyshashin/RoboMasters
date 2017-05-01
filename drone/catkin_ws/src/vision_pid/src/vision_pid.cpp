  /**
  ******************************************************************************
  * @file    pid_controller.cpp
  * @author  Bobby SHEN 
  * @version V1.2.0
  * @date    06-September-2016
  * @brief   This is a PID controller node based on ROS, modified from gaowenliang's code
  *           
  ******************************************************************************  
  */ 

#include <iostream>
#include <stdio.h>
#include <ros/ros.h>

#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>

#include "pid.h"

using namespace std;
using namespace ros;
//using namespace DJI::onboardSDK;

ros::Publisher ctrl_vel_pub;
ros::Publisher pos_error_pub;

ros::Subscriber pid_parameter_sub;       // For tuning PID parameters (Kp, Ki and Kd)
ros::Subscriber target_pos_sub;          // Target position  
ros::Subscriber current_pos_sub;         // Current position (calculated from sensor data)
ros::Subscriber pid_ctrl_limit_sub;      // For tuning velocity limits 
ros::Subscriber marker_center_sub;       // Centroid's coordinate of detected markers

PID *pid_vision_x;
PID *pid_vision_y;
PID *pid_z;
PID *pid_yaw;

int camera_offset = 0;
int gripper_offset = 0;

/*****************************
*	     ^ x             *  
*	     |               *
*	     |               * 
*	     |             y *
* -----------|-------------> *
*	     |               *
*	     |               *
*	     |               *
*	     |               *
*****************************/

double Kp_vision = 0.8;
double Ki_vision = 0;
double Kd_vision = 0.1;

double Kp_z;
double Ki_z;
double Kd_z;

double Kp_yaw = 0;
double Ki_yaw = 0;
double Kd_yaw = 0;

double pid_yaw_limit = 0;

int img_center[2] = {376, 240}; // x & y center of image pixels

float ctrl_data[4] = {0, 0, 0, 0}; // Velocity of x, y, z and yaw
float vision_ctrl[2] = {0, 0};

float target_position[3] = {376, 240, 2.8};
float target_yaw = 0;

float dt = 0.02;

double vision_ctrl_limit = 0.6;
double height_ctrl_limit = 0.6;

void delay_s(int x) { // delay in second

    ros::Duration(x).sleep();

}

void window(float* src, float limit) {
    
    if(limit < 0) {
        cout << "Window limit must be greater than zero!" << endl;
	return;
    }
	
    if(*src > limit)
	*src = limit;
    if(*src < -limit)
 	*src = -limit;
}

void target_pos_callback(const geometry_msgs::Vector3& target_pos) {

    /* Ignore tiny differences within 1 cm */
    if( abs(target_pos.x) < 0.01 ) 
        
        target_position[0] = 0;

    else 

        target_position[0] = target_pos.x;


    if( abs(target_pos.y) < 0.01 ) 
        
        target_position[1] = 0;

    else 
    
        target_position[1] = target_pos.y;


    if( target_pos.z < 2 && target_pos.z > 0.1 ) 
        
        target_position[2] = target_pos.z;
    
    else 
    
        target_position[2] = target_position[2];

    /* Update the target position */
    pid_vision_x->set_point(target_position[0]);
    pid_vision_y->set_point(target_position[1]);
    pid_z->set_point(target_position[2]);
    pid_yaw->set_point(0);
 
}


void pid_update(geometry_msgs::Vector3 current_position) {

    /* PID position update */
    ctrl_data[0] = pid_vision_x -> update(current_position.x, dt);
    ctrl_data[1] = pid_vision_y -> update(current_position.y, dt);
    ctrl_data[2] = pid_z -> update(current_position.z, dt);
    ctrl_data[3] = 0; //yaw

    /* Control speed limiting */
    window(&ctrl_data[0], vision_ctrl_limit);
    window(&ctrl_data[1], vision_ctrl_limit);
    window(&ctrl_data[2], vision_ctrl_limit);

    /* Publish the output control velocity from PID controller */
    geometry_msgs::Vector3 velocity;

    velocity.x = ctrl_data[0];
    velocity.y = ctrl_data[1];
    velocity.z = ctrl_data[2];

    ctrl_vel_pub.publish(velocity);


}

void current_pos_callback(const geometry_msgs::Vector3& current_position) {

    geometry_msgs::Vector3 pos_error;

    pos_error.x = target_position[0] - current_position.x;
    pos_error.y = target_position[1] - current_position.y;
    pos_error.z = target_position[2] - current_position.z;

    pos_error_pub.publish(pos_error);

    cout << "error_x -> " << pos_error.x << "      error_y -> " << pos_error.y << endl << "      error_z -> " << pos_error.z << endl;

    pid_update(current_position);

}

void vision_ctrl_callback(const geometry_msgs::Vector3& msg) {

    pid_vision_x -> update(msg.x, dt);
    pid_vision_y -> update(msg.y, dt);
    // msg.z is the marker ID
    cout << "Vision-based PID has been updated!" << endl;
}

void pid_parameter_tuning_callback(const geometry_msgs::Vector3& msg) {

   
    pid_vision_x -> set_param(msg.x, msg.y, msg.z); //Set Kp, Ki & Kd
    pid_vision_y -> set_param(msg.x, msg.y, msg.z); //Set Kp, Ki & Kd

    cout << "PID parameters for horizontal position control have been updated!" << endl;
    cout << "Kp: " << msg.x << endl;
    cout << "Ki: " << msg.y << endl;
    cout << "Kd: " << msg.z << endl;

}

void pid_parameter_vert_tuning_callback(const geometry_msgs::Vector3& msg) {

    pid_z -> set_param(msg.x, msg.y, msg.z); //Set Kp, Ki & Kd

    cout << "PID parameters for vertical position control have been updated!" << endl;

    cout << "Kp: " << msg.x << endl;
    cout << "Ki: " << msg.y << endl;
    cout << "Kd: " << msg.z << endl;

}

void pid_ctrl_limit_callback(const geometry_msgs::Vector3& msg) {

    //pid_ctrl_limit_horz = msg.x;
    //pid_ctrl_limit_vert = msg.y;
    
    //cout << "Speed limit has been updated!" << endl;
    //cout << "Horizontal: " << pid_ctrl_limit_horz << " m/s" << endl;
    //cout << "Vertical: " << pid_ctrl_limit_vert << " m/s" << endl;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "pid_controller");
    ros::NodeHandle nh("~");
    std_msgs::UInt8 bias_correction_msg;

    //drone = new DJIDrone(nh);


    pid_z   = new PID( Kp_z, Ki_z, Kd_z, -5, 5, -height_ctrl_limit, height_ctrl_limit, false);
    pid_yaw = new PID( Kp_yaw, Ki_yaw, Kd_yaw, -5, 5, -pid_yaw_limit, pid_yaw_limit, false);

    pid_vision_x = new PID(Kp_vision, Ki_vision, Kd_vision, -5, 5, -vision_ctrl_limit, vision_ctrl_limit, false);
    pid_vision_y = new PID(Kp_vision, Ki_vision, Kd_vision, -5, 5, -vision_ctrl_limit, vision_ctrl_limit, false);

    //vision_pid_x->set_point(target_position[0]);
    //vision_pid_y->set_point(target_position[1]);
    pid_z->set_point(target_position[2]);
    pid_yaw->set_point(target_yaw);

    pid_vision_x->set_point(img_center[0]);
    pid_vision_y->set_point(img_center[1]);

    ros::Rate loop_rate(50);

    target_pos_sub       = nh.subscribe("/target_position",  1, target_pos_callback);
    current_pos_sub      = nh.subscribe("/current_position", 1, current_pos_callback);

    pid_parameter_sub    = nh.subscribe("/pid_parameter",    1, pid_parameter_tuning_callback);
    //pid_parameter_vert_sub = nh.subscribe("/pid_vert_parameter",    1, pid_parameter_vert_tuning_callback);
    //TODO Union both horz and vert pid param into one subscriber or do a better renaming with two
    pid_ctrl_limit_sub   = nh.subscribe("/pid_ctrl_limit",   1, pid_ctrl_limit_callback);

    marker_center_sub = nh.subscribe("/marker_centers", 1, vision_ctrl_callback);

    ctrl_vel_pub         = nh.advertise<geometry_msgs::Vector3>("/ctrl_vel", 10);
    pos_error_pub        = nh.advertise<geometry_msgs::Vector3>("/position_error", 1);

    cout << "PID controller activated!" << endl;
    cout << "Last modified: " << "2017-03-15" << endl;
    
    while(ros::ok()) {

        loop_rate.sleep();
        ros::spinOnce();
        
    }

    return 0;
}

