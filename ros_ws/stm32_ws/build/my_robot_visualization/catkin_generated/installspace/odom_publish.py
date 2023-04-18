#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from custom_msg.msg import mpu_msg
import math

easting = 0.0
northing = 0.0
roll = 0.0
pitch = 0.0
yaw = 0.0

def callback_easting(data):
    global easting
    easting = data.data

def callback_northing(data):
    global northing
    northing = data.data
    
def callback_mpu(data):
    global roll
    global pitch
    global yaw
    roll = data.roll
    pitch = data.pitch
    yaw = data.yaw

def quaternion_from_euler(roll, pitch, yaw):
    roll = roll * math.pi/180
    pitch = pitch * math.pi/180
    yaw = yaw * math.pi/180
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    w = cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr
    return [x, y, z, w]

def listener():
    
    rospy.init_node('listener', anonymous=True)
    pub_marker = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    pub_odom = rospy.Publisher('odom', Odometry, queue_size=10)
    rospy.Subscriber("easting", Float64, callback_easting)
    rospy.Subscriber("northing", Float64, callback_northing)
    rospy.Subscriber("MPU_data", Float64, callback_mpu)
    odom = Odometry()
    marker = Marker()
    marker.header.frame_id = "map"
    marker.type = marker.CUBE
    #marker.type = marker.MESH_RESOURCE
    #marker.mesh_resource = "file:///home/tien/Downloads/v1-01/model.dae";
    marker.action = marker.ADD
    marker.scale.x = 9.0
    marker.scale.y = 3.0
    marker.scale.z = 1.0
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0

    rate = rospy.Rate(20) # 20Hz
    while not rospy.is_shutdown():
        #Rviz marker
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        print("RPY: ")
        print(roll, pitch, yaw)
        print("Quaternion: ")
        print(quaternion)
        marker.pose.orientation.x = quaternion[0]
        marker.pose.orientation.y = -quaternion[1]
        marker.pose.orientation.z = -quaternion[2]
        marker.pose.orientation.w = quaternion[3]
        
        #Odom data
        odom.pose.pose.position.x = easting
        odom.pose.pose.position.y = northing
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = -quaternion[1]
        odom.pose.pose.orientation.z = -quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]
        pub_marker.publish(marker)
        pub_odom.publish(odom)
        rate.sleep()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass



