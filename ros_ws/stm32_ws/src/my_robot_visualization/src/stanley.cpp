#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <iostream>
#include <iomanip>
#include <exception>
#include <string>
#include <GeographicLib/UTMUPS.hpp>
#include <sensor_msgs/NavSatFix.h>

using namespace std;
using namespace GeographicLib;


double lat = 0.0, lon = 0.0;

void callback_function(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    // Process the received message
    ROS_INFO("Latitude: %f, Longitude: %f", msg->latitude, msg->longitude);
    lat = msg->latitude;
    lon = msg->longitude;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/ublox/fix", 1000, callback_function);
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    int zone;
    bool northp;
    double x, y;
    std_msgs::String msg;
    std::stringstream ss;
    UTMUPS::Forward(lat, lon, zone, northp, x, y);
    string zonestr = UTMUPS::EncodeZone(zone, northp);
    ss << fixed << setprecision(2)
       << zonestr << " " << x << " " << y << "\n";
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
