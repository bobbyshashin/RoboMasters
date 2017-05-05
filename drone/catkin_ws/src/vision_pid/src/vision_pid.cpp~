  /**
  ******************************************************************************
  * @file    pid_controller.cpp
  * @author  Bobby SHEN 
  * @version V1.4.2
  * @date    01-May-2017
  * @brief   This is a vision-based PID controller, for RoboMasters 2017 UAV auto-landing 
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

Publisher ctrl_vel_pub;
Publisher pos_error_pub;

Subscriber pid_parameter_sub;       // For tuning PID parameters (Kp, Ki and Kd)
Subscriber target_pos_sub;          // Target position  
Subscriber marker_center_sub;       // Detected circle's center coordinate in camera frame

PID *pid_vision_x;
PID *pid_vision_y;
PID *pid_z;
PID *pid_yaw;

int camera_offset = 0;
int gripper_offset = 0;

/****** UAV Body Frame *******
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

double Kp_vision = 0.001;
double Ki_vision = 0;
double Kd_vision = 0.0005;

double Kp_z;
double Ki_z;
double Kd_z;

double Kp_yaw = 0;
double Ki_yaw = 0;
double Kd_yaw = 0;

int img_center[2] = {320, 240}; // x & y center of image pixels

float ctrl_data[4] = {0, 0, 0, 0}; // Velocity of x, y, z and yaw
float vision_ctrl[2] = {0, 0};

float target_position[3] = {0, 0, 2.8};
float target_yaw = 0;

float dt = 0.1;

int threshold = 20;
bool arrived = false;

double visionLimit = 0.4;
double height_ctrl_limit = 0.6;
double pid_yaw_limit = 0;

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

void pid_update() {

    /* PID position update */
    ctrl_data[0] = pid_vision_x -> update(img_center[1], dt); // OpenCV's x-axis is drone's y-axis
    ctrl_data[1] = pid_vision_y -> update(img_center[0], dt);
    //ctrl_data[2] = pid_z -> update(current_position.z, dt);
    ctrl_data[3] = 0; //yaw

    /* Control speed limiting */
    window(&ctrl_data[0], visionLimit);
    window(&ctrl_data[1], visionLimit);
    window(&ctrl_data[2], visionLimit);

    /* Publish the output control velocity from PID controller */
    geometry_msgs::Vector3 velocity;

    velocity.x = ctrl_data[0];
    velocity.y = ctrl_data[1];
    velocity.z = ctrl_data[2];

    ctrl_vel_pub.publish(velocity);


}

void target_pos_callback(const geometry_msgs::Vector3& target_pos) {

    //TODO do some boundary checking and filtering
    target_position[0] = target_pos.x;
    target_position[1] = target_pos.y;
    if( target_pos.z < 2 && target_pos.z > 0.1 ) 
        target_position[2] = target_pos.z;
    else 
        target_position[2] = target_position[2];

    if((target_position[0] - img_center[0] <= threshold) && (target_position[1] - img_center[1] <= threshold))
	arrived = true;
    /* Update the target position */
    pid_vision_x->set_point(target_position[0]);
    pid_vision_y->set_point(target_position[1]);
    pid_z->set_point(target_position[2]);
    pid_yaw->set_point(0);

    pid_update();
 
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

int main(int argc, char** argv) {

    ros::init(argc, argv, "pid_controller");
    ros::NodeHandle nh("~");

    pid_vision_x = new PID(Kp_vision, Ki_vision, Kd_vision, 0, 0, -visionLimit, visionLimit, false);
    pid_vision_y = new PID(Kp_vision, Ki_vision, Kd_vision, 0, 0, -visionLimit, visionLimit, false);
    pid_z   = new PID( Kp_z, Ki_z, Kd_z, 0, 0, -height_ctrl_limit, height_ctrl_limit, false);
    pid_yaw = new PID( Kp_yaw, Ki_yaw, Kd_yaw, 0, 0, -pid_yaw_limit, pid_yaw_limit, false);

    //vision_pid_x->set_point(target_position[0]);
    //vision_pid_y->set_point(target_position[1]);
    pid_z->set_point(target_position[2]);
    pid_yaw->set_point(target_yaw);

    pid_vision_x->set_point(img_center[1]); // OpenCV's x-axis is drone's y-axis
    pid_vision_y->set_point(img_center[0]);

    ros::Rate loop_rate(20);

    target_pos_sub       = nh.subscribe("/target_position",  1, target_pos_callback);
    pid_parameter_sub    = nh.subscribe("/pid_parameter",    1, pid_parameter_tuning_callback);
    marker_center_sub = nh.subscribe("/marker_centers", 1, vision_ctrl_callback);

    ctrl_vel_pub         = nh.advertise<geometry_msgs::Vector3>("/ctrl_vel", 10);
    pos_error_pub        = nh.advertise<geometry_msgs::Vector3>("/position_error", 1);

    cout << "PID controller is activated!" << endl;
    cout << "Last modified: " << "2017-05-01" << endl;
    
    while(ros::ok()) {

        loop_rate.sleep();
        ros::spinOnce();
        
    }

    return 0;
}

