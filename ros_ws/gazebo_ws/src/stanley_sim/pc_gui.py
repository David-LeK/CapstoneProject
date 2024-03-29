#!/usr/bin/env python

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

def downsample_coordinates(coordinate_x, coordinate_y, min_distance=0.5):
    # initialize variables to store sampled points
    sampled_x = [coordinate_x[0]]
    sampled_y = [coordinate_y[0]]
    last_sampled_x = coordinate_x[0]
    last_sampled_y = coordinate_y[0]

    # iterate over remaining points and select those that are at least min_distance apart
    for i in range(1, len(coordinate_x)):
        distance = math.sqrt((coordinate_x[i] - last_sampled_x)**2 + (coordinate_y[i] - last_sampled_y)**2)
        if distance >= min_distance:
            sampled_x.append(coordinate_x[i])
            sampled_y.append(coordinate_y[i])
            last_sampled_x = coordinate_x[i]
            last_sampled_y = coordinate_y[i]

    # return sampled points as lists
    return sampled_x, sampled_y

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
    rospy.Subscriber('Autonomous_Robot_GUI/easting_kml', Float32MultiArray, eastingCallback)
    rospy.Subscriber('Autonomous_Robot_GUI/northing_kml', Float32MultiArray, northingCallback)
    while not rospy.is_shutdown():
        while easting and northing:
            r = cubic_spline(easting, northing)
            if r is not None:
                x_array = r[0,:]
                y_array = r[1,:]
                x_array, y_array = downsample_coordinates(x_array, y_array)
                print(len(x_array))
                print(len(y_array))
                print(x_array)
                print(y_array)
                path = Path()
                path.header.frame_id = "map"  # replace with your desired frame ID
                for i in range (0, len(x_array)):
                    pose = PoseStamped()
                    pose.header.frame_id = "map"  # replace with your desired frame ID
                    pose.pose.position.x = x_array[i]
                    pose.pose.position.y = y_array[i]
                    path.poses.append(pose)
                path_pub.publish(path)
                easting = []
                northing = []

# Initialize the ROS node and subscribe/publish to the topics
if __name__ == '__main__':
    rospy.init_node('pc_receive_node')
    read()
