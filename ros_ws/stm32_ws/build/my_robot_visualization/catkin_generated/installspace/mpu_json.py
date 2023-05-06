#!/usr/bin/env python2

import socket
import json
import rospy
from custom_msg.msg import mpu_msg, gps_msg
import utm

class MPU_JSON(object):
    def __init__(self):
        # initialize ROS node
        rospy.init_node('tcp_server')
        
        # create a TCP/IP socket
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # bind the socket to a specific address and port
        self.server_address = ('', 2055)
        self.server_socket.bind(self.server_address)

        # listen for incoming connections
        self.server_socket.listen(1)

        # Set a timeout of 30 seconds for the accept() method
        self.server_socket.settimeout(30)
        
        self.mpu_msg = mpu_msg()
        self.mpu_pub = rospy.Publisher('/MPU_data', mpu_msg, queue_size=10)
        
        self.gps_msg = gps_msg()
        self.gps_pub = rospy.Publisher('/GPS_data', gps_msg, queue_size=10)
        
    def receive_json_data(self):
        try:
            while not rospy.is_shutdown():
                # wait for a client connection
                rospy.loginfo('Waiting for a client connection...')
                client_socket, client_address = self.server_socket.accept()
                rospy.loginfo('Accepted connection from %s', client_address)

                # receive and process JSON data from the client
                while not rospy.is_shutdown():
                    data = client_socket.recv(1024)
                    if not data:
                        break
                    # Find the index of the starting and ending braces
                    start_index = data.find(b'{')
                    end_index = data.find(b'}', start_index) + 1
                    if start_index != -1 and end_index != 0:
                        try:
                            json_data = json.loads(data[start_index:end_index].decode())
                            # get data from the keys in ROS Python
                            orientation = json_data['orientation']
                            self.mpu_msg.roll = orientation[2]
                            self.mpu_msg.pitch = orientation[1]
                            self.mpu_msg.yaw = orientation[0]
                            self.mpu_pub.publish(self.mpu_msg)
                            
                            gps = json_data['GPS']
                            self.gps_msg.latitude = gps[0]
                            self.gps_msg.longitude = gps[1]
                            utm_coord = utm.from_latlon(gps[0], gps[1])
                            self.gps_msg.easting = utm_coord[0]
                            self.gps_msg.northing = utm_coord[1]
                            self.gps_msg.speed_kmh = 1
                            self.gps_pub.publish(self.gps_msg)
                        except Exception as e:
                            rospy.logerr('Error processing: %s', str(e))

                # close the connection
                rospy.loginfo('Closing connection with %s', client_address)
                client_socket.close()

        except socket.timeout:
            print("Timeout waiting for client connection")
            self.server_socket.close()
    
if __name__ == '__main__':
    try:
        mpu_json = MPU_JSON()
        mpu_json.receive_json_data()
    except rospy.ROSInterruptException:
        pass
        
