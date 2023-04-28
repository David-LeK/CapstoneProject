#!/usr/bin/env python

import rospy
from serial import Serial
import utm
from custom_msg.msg import gps_msg

def publish_topic(topic_name, message):
    pub = rospy.Publisher(topic_name, gps_msg, queue_size=10)
    #rospy.loginfo(message)
    pub.publish(message)

def parse_rmc_sentence(rmc_sentence):
    rmc_fields = rmc_sentence.split(',')
    if len(rmc_fields) < 12 or rmc_fields[2] != 'A':
        return

    # Parse latitude and longitude
    latitude_degrees = int(rmc_fields[3][:2])
    latitude_minutes = float(rmc_fields[3][2:])
    longitude_degrees = int(rmc_fields[5][:3])
    longitude_minutes = float(rmc_fields[5][3:])
    longitude = longitude_degrees + (longitude_minutes / 60)
    latitude = latitude_degrees + (latitude_minutes / 60)
    if rmc_fields[4] == 'S':
        latitude *= -1
    if rmc_fields[6] == 'W':
        longitude *= -1

    utm_coord = utm.from_latlon(latitude, longitude)
    easting = utm_coord[0]
    northing = utm_coord[1]
    # Convert speed from knots to km/h
    speed_knots = float(rmc_fields[7])
    speed_kmh = speed_knots * 1.852
    
    gps_data = gps_msg()
    gps_data.latitude = latitude
    gps_data.longitude = longitude
    gps_data.speed_kmh = speed_kmh
    gps_data.easting = easting
    gps_data.northing = northing
    if (rmc_fields[8] == ''):
        gps_data.tracking_angle = 0
    else:
        gps_data.tracking_angle = float(rmc_fields[8])
    publish_topic('GPS_data', gps_data)
    

def serial_talker():
    rospy.init_node('read_gps', anonymous=True)
    serial_port = Serial("/dev/ttyACM0", baudrate=115200)
    serial_port.flushInput()
    while not rospy.is_shutdown():
        serial_data = serial_port.read(serial_port.inWaiting()).decode()
        nmea_sentences = serial_data.split("\n")
        for nmea_sentence in nmea_sentences:
            if "$GNRMC" in nmea_sentence:
                parse_rmc_sentence(nmea_sentence)
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        serial_talker()
    except rospy.ROSInterruptException:
        pass

