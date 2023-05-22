#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from roslibpy import Ros, Topic

rospy.init_node('local_subscriber')

# Callback function to handle received messages from the local topic
def local_callback(message):
    # Publish the received message to the ROS Bridge server
    x=list(message.data)
    pub = Float32MultiArray(data=[float(j) for j in x])
    ros_bridge_publisher.publish(pub)

# Create a ROS Bridge client
ros_bridge_client = Ros('103.200.20.166', 9090)  # Replace with the IP address or hostname of the ROS Bridge server running on the VMware machine
ros_bridge_client.run()

# Create a publisher to publish messages to the ROS Bridge server
ros_bridge_publisher = Topic(ros_bridge_client, '/rosbridge_topic', 'std_msgs/Float32MultiArray')

# Subscribe to the local topic
local_subscriber = rospy.Subscriber('/Autonomous_Robot_GUI/easting_kml', Float32MultiArray, local_callback)

# Keep the script running until interrupted
rospy.spin()

# Clean up and close the connection
ros_bridge_publisher.unadvertise()
ros_bridge_client.close()
