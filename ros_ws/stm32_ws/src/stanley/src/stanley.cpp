#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <iomanip>
#include <exception>
#include <string>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <cmath>

using namespace std;

//Reference vars
std::vector<float> ref_easting; //X_ref
std::vector<float> ref_northing; //Y_ref
std::vector<float> ref_angle; //Theta_ref

//Real vars
double car_x = 0.0; //X_current
double car_y = 0.0; //Y_current
double car_yaw = 0.0; //Theta_current
int current_path_index = 0;

//Steering vars
double max_speed;
double max_steering_angle;
double k = 1.0;
double k_v = 0.1;

// Reference callback
void refEastingCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  ref_easting = msg->data;
  ROS_INFO("Ref_East: %f ", ref_easting);
}

void refNorthingCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  ref_northing = msg->data;
  ROS_INFO("Ref_Northing: %f ", ref_northing);
}

void angleCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  ref_angle = msg->data;
  ROS_INFO("Ref_yaw: %f ", ref_angle);
}

//Real callback

void eastingCallback(const std_msgs::Float64::ConstPtr& msg) {
  car_x = msg->data;
  ROS_INFO("Easting: %f ", car_x);
}

void northingCallback(const std_msgs::Float64::ConstPtr& msg) {
  car_y = msg->data;
  ROS_INFO("Northing: %f ", car_y);
}

void yawCallback(const std_msgs::Float64::ConstPtr& msg) {
  car_yaw = msg->data;
  ROS_INFO("Yaw: %f ", car_yaw);
}

void calculateSteeringAngle() {
		/*
		car_x, car_y: The car's current position in the x and y coordinates
		car_yaw: The car's current orientation in radians
		ref_easting, ref_northing: The x and y coordinates of the current point on the path
		k: The gain parameter for the cross-track error
		max_speed: The maximum speed of the car
		max_steering_angle: The maximum steering angle of the car
		*/
		
		// Calculate lateral error
		double car_direction_x = std::cos(car_yaw);
		double car_direction_y = std::sin(car_yaw);
		double dx = ref_easting[current_path_index] - car_x;
		double dy = ref_northing[current_path_index] - car_y;
		double lateral_error = -dy * car_direction_x + dx * car_direction_y;
		
		// Check if the car has reached the current path index
		double distance_to_current_point = std::sqrt(dx*dx + dy*dy);
		if (distance_to_current_point < 0.5 && current_path_index < ref_easting.size() - 1) {
			// Update the current path index to the next point on the path
			current_path_index++;
		}
		
		// Calculate the desired steering angle using the Stanley control algorithm
		double steering_angle = atan2(k * lateral_error, max_speed);
		
		ROS_INFO("Steering angle: %f ", steering_angle);
		// Limit steering angle
		steering_angle = std::max(-max_steering_angle, std::min(steering_angle, max_steering_angle));
		
		// double velocity = calculateVelocity();
		
		// Publish cmd_vel
		geometry_msgs::Twist cmd_vel;
		// cmd_vel.linear.x = velocity;
		// cmd_vel.angular.z = steering_angle;
		//cmd_vel_pub_.publish(cmd_vel);
	}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stanley_node");
  ros::NodeHandle nh;

  // Real data
  ros::Subscriber easting_sub = nh.subscribe("easting", 1000, eastingCallback);
  ros::Subscriber northing_sub = nh.subscribe("northing", 1000, northingCallback);
  ros::Subscriber yaw_sub = nh.subscribe("yaw", 1, yawCallback);

  //Reference data
  ros::Subscriber ref_easting_sub = nh.subscribe("ref_easting", 1, refEastingCallback);
  ros::Subscriber ref_northing_sub = nh.subscribe("ref_northing", 1, refNorthingCallback);
  ros::Subscriber angle_sub = nh.subscribe("ref_angle", 1, angleCallback);

  ros::Rate loop_rate(10); //10Hz
  while (ros::ok())
  {
    calculateSteeringAngle();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

