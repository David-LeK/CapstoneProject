#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import serial
import json

# Define the serial port and baudrate
serial_port = '/home/tien/Documents/COM10'
baudrate = 115200

# Initialize the serial connection
ser = serial.Serial(serial_port, baudrate)

# Initialize the dictionary to store the values
data = {}

# Define the callback functions for each subscribed topic
def latitude_callback(msg):
    data['latitude'] = msg.data
    send_data()

def longitude_callback(msg):
    data['longitude'] = msg.data
    send_data()

def speed_kmh_callback(msg):
    data['speed'] = msg.data
    send_data()

def track_angle_callback(msg):
    data['angle'] = msg.data
    send_data()

def easting_callback(msg):
    data['easting'] = msg.data
    send_data()

def northing_callback(msg):
    data['northing'] = msg.data
    send_data()

# Define the function to send the data
def send_data():
    if all(key in data for key in ['latitude', 'longitude', 'speed', 'angle', 'easting', 'northing']):
        json_data = json.dumps(data)
        # Clear the output buffer
        ser.flushOutput()
        ser.write(json_data.encode() + b'\r\n')

# Initialize the ROS node and subscribe to the topics
if __name__ == '__main__':
    rospy.init_node('sending_pc')

    rospy.Subscriber('latitude', Float64, latitude_callback)
    rospy.Subscriber('longitude', Float64, longitude_callback)
    rospy.Subscriber('speed_kmh', Float64, speed_kmh_callback)
    rospy.Subscriber('track_angle', Float64, track_angle_callback)
    rospy.Subscriber('easting', Float64, easting_callback)
    rospy.Subscriber('northing', Float64, northing_callback)

    rospy.spin()
