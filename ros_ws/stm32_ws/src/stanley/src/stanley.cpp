#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <iomanip>
#include <exception>
#include <string>
#include <GeographicLib/UTMUPS.hpp>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <vector>

using namespace std;
using namespace GeographicLib;

std::vector<float> ref_easting; //X_ref
std::vector<float> ref_northing; //Y_ref
std::vector<float> ref_angle; //Theta_ref
double lat = 0.0, lon = 0.0;
double x = 0.0, y = 0.0; //X_current, Y_current
double yaw = 0.0; //Theta_current

void eastingCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  ref_easting = msg->data;
  for (int i = 0; i < ref_easting.size(); i++) {
      ROS_INFO("Easting: %f", ref_easting[i]);
  }
}

void northingCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  ref_northing = msg->data;
}

void angleCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  ref_angle = msg->data;
}

void yawCallback(const std_msgs::Float64::ConstPtr& msg) {
  yaw = msg->data;
  ROS_INFO("Yaw value: %f", yaw);
}

void coords_function(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    // Process the received message
    ROS_INFO("Latitude: %f, Longitude: %f", msg->latitude, msg->longitude);
    lat = msg->latitude;
    lon = msg->longitude;
}

int main(int argc, char **argv)
{
  int zone;
  bool northp;
  ros::init(argc, argv, "stanley_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/ublox/fix", 1000, coords_function);
  ros::Subscriber easting_sub = nh.subscribe("ref_easting", 1, eastingCallback);
  ros::Subscriber northing_sub = nh.subscribe("ref_northing", 1, northingCallback);
  ros::Subscriber angle_sub = nh.subscribe("ref_angle", 1, angleCallback);
  ros::Subscriber yaw_sub = nh.subscribe("yaw", 1, yawCallback);
  ros::Rate loop_rate(10); //10Hz
  while (ros::ok())
  {
    UTMUPS::Forward(lat, lon, zone, northp, x, y);
    string zonestr = UTMUPS::EncodeZone(zone, northp);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

