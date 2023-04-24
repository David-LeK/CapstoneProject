#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import numpy
import scipy.interpolate
import math

easting = []
northing = []

def cubic_spline(easting, northing):
    path_x = numpy.asarray(easting,dtype=float)
    path_y = numpy.asarray(northing,dtype=float)

    # defining arbitrary parameter to parameterize the curve
    path_t = numpy.linspace(0,1,path_x.size)

    # this is the position vector with
    # x coord (1st row) given by path_x, and
    # y coord (2nd row) given by path_y
    r = numpy.vstack((path_x.reshape((1,path_x.size)),path_y.reshape((1,path_y.size))))

    # creating the spline object
    spline = scipy.interpolate.interp1d(path_t,r,kind='cubic')

    # defining values of the arbitrary parameter over which
    # you want to interpolate x and y
    # it MUST be within 0 and 1, since you defined
    # the spline between path_t=0 and path_t=1
    t = numpy.linspace(numpy.min(path_t),numpy.max(path_t),100)

    # interpolating along t
    # r[0,:] -> interpolated x coordinates
    # r[1,:] -> interpolated y coordinates
    r = spline(t)
    return r

def eastingCallback(data):
    global easting
    # Convert the Float32MultiArray message to a Python list
    easting = [float(x) for x in data.data]
    
def northingCallback(data):
    global northing
    northing = [float(x) for x in data.data]

def read():
    global easting, northing
    path_pub = rospy.Publisher('path', Path, queue_size=10)
    ref_yaw_pub = rospy.Publisher('ref_yaw', Float32MultiArray, queue_size=10)
    rospy.Subscriber('Autonomous_Robot_GUI/easting_kml', Float32MultiArray, eastingCallback)
    rospy.Subscriber('Autonomous_Robot_GUI/northing_kml', Float32MultiArray, northingCallback)
    path = Path()
    while not rospy.is_shutdown():
        while easting and northing:
            r = cubic_spline(easting, northing)
            if r is not None:
                x_array = r[0,:]
                y_array = r[1,:]
                print(x_array)
                print(y_array)
                path.header.frame_id = "map"  # replace with your desired frame ID
                for i in range (0, len(x_array)):
                    pose = PoseStamped()
                    pose.header.frame_id = "map"  # replace with your desired frame ID
                    pose.pose.position.x = x_array[i]
                    pose.pose.position.y = y_array[i]
                    path.poses.append(pose)
                angle = []
                for i in range(len(x_array)-1):
                    delta_x = r[0,i+1] - r[0,i]
                    delta_y = r[1,i+1] - r[1,i]
                    angle.append(math.atan2(delta_x, delta_y))
                angle.append(0.0)
                ref_yaw = Float32MultiArray(data=[float(x) for x in angle])
                path_pub.publish(path)
                ref_yaw_pub.publish(ref_yaw)
                easting = []
                northing = []

# Initialize the ROS node and subscribe/publish to the topics
if __name__ == '__main__':
    rospy.init_node('pc_receive_node')
    read()
