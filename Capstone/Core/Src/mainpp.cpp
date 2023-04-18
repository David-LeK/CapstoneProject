/*
 * main.cpp

 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */
#include <mainpp.h>
#include <custom_msg/encoder_input_msg.h>
#include <custom_msg/encoder_output_msg.h>
#include <custom_msg/mpu_msg.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <stdint.h>
#include <stdio.h>

ros::NodeHandle nh;

void vel_cb(const custom_msg::encoder_input_msg& msg);

//Dummy publisher for testing
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[] = "Hello world!";

//Create a publisher node to topic PID_data
custom_msg::encoder_output_msg pid_msg;
ros::Publisher pid_pub("PID_data", &pid_msg);

//Create a publisher node to topic MPU_data
custom_msg::mpu_msg mpu_msg;
ros::Publisher mpu_pub("MPU_data", &mpu_msg);

//Create a subscriber node to receive setpoint, Kp, Ki and Kd
ros::Subscriber<custom_msg::encoder_input_msg> vel_sub("PID_ctrl", &vel_cb);

void setup(void)
{
  nh.initNode();
  nh.advertise(chatter);
  nh.advertise(pid_pub);
  nh.advertise(mpu_pub);
  nh.subscribe(vel_sub);
}

void loop(void)
{
	str_msg.data = hello;

	pid_msg.output_controller_m1 = m1.u;
	pid_msg.output_rpm_m1 = m1.v;
	pid_msg.output_controller_m2 = m2.u;
	pid_msg.output_rpm_m2 = m2.v;

	mpu_msg.roll = -attitude.r;
	mpu_msg.pitch = attitude.p;
	mpu_msg.yaw = attitude.y;

	chatter.publish(&str_msg);
	pid_pub.publish(&pid_msg);
	mpu_pub.publish(&mpu_msg);

	nh.spinOnce();
	HAL_Delay(50);
}

void vel_cb(const custom_msg::encoder_input_msg& msg){
	Setpoint_value_m1 = msg.input_setpoint_m1;
	Kp_m1 = msg.input_Kp_m1;
	Ki_m1 = msg.input_Ki_m1;
	Kd_m1 = msg.input_Kd_m1;
	Setpoint_value_m2 = msg.input_setpoint_m2;
	Kp_m2 = msg.input_Kp_m2;
	Ki_m2 = msg.input_Ki_m2;
	Kd_m2 = msg.input_Kd_m2;
}
