#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import serial
import json
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import time
import numpy
import scipy.interpolate
import utm
import math

# Define the serial port and baudrate
serial_port = '/dev/ttyUSB0'
baudrate = 115200

# Initialize the serial connection
ser_pc = serial.Serial(serial_port, baudrate)

# Initialize the dictionary to store the values
data = {}

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
    

def parse_serial(received):
    # Extract the data between the "BEGIN:" and ":END" patterns
    try:
        data = received.split("BEGIN:")[1].split(":END")[0]
    except IndexError:
        return None

    # Split the string into the two separate arrays of double
    str1, str2 = data.split(";")
    lat = [float(x) for x in str1.split(",")]
    lon = [float(x) for x in str2.split(",")]
    
    # Convert the coordinates
    utm_coords = [utm.from_latlon(lat[i], lon[i]) for i in range(len(lat))]

    # Extract the easting and northing coordinates
    easting = [c[0] for c in utm_coords]
    northing = [c[1] for c in utm_coords]
    
    return cubic_spline(easting, northing)


def read():
    path_pub = rospy.Publisher('path', Path, queue_size=10)
    ref_yaw_pub = rospy.Publisher('ref_yaw', Float32MultiArray, queue_size=10)
    path = Path()
    if ser_pc.inWaiting() > 0:
        line = ser_pc.readline().decode('utf-8').strip()
        print(line)
        r = parse_serial(line)
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

# Initialize the ROS node and subscribe/publish to the topics
if __name__ == '__main__':
    rospy.init_node('serial_pc')
    rospy.Subscriber('latitude', Float64, latitude_callback)
    rospy.Subscriber('longitude', Float64, longitude_callback)
    rospy.Subscriber('speed_kmh', Float64, speed_kmh_callback)
    rospy.Subscriber('track_angle', Float64, track_angle_callback)
    rospy.Subscriber('easting', Float64, easting_callback)
    rospy.Subscriber('northing', Float64, northing_callback)
    read()
    rospy.spin()
