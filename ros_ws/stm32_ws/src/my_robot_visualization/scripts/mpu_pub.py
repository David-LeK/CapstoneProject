#!/usr/bin/env python

import socket
import json
import rospy
from custom_msg.msg import mpu_msg
from serial import Serial

class MPU(object):
    def __init__(self):
        # initialize ROS node
        rospy.init_node('imu_node')
        
        # create a TCP/IP socket
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # bind the socket to a specific address and port
        self.server_address = ('', 2055)
        self.server_socket.bind(self.server_address)

        # listen for incoming connections
        self.server_socket.listen(1)

        # Set a timeout of 30 seconds for the accept() method
        self.server_socket.settimeout(10)
        
        self.mpu_msg = mpu_msg()
        self.mpu_pub = rospy.Publisher('/MPU_data', mpu_msg, queue_size=10)
        
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
                            
                        except Exception as e:
                            rospy.logerr('Error processing: %s', str(e))

                # close the connection
                rospy.loginfo('Closing connection with %s', client_address)
                client_socket.close()

        except socket.timeout:
            print("Timeout waiting for client connection")
            self.server_socket.close()
            
    def receive_imu_serial(self):
        serial_port = Serial("/dev/imu_usb", baudrate=230400)
        serial_port.flushInput()
        rate = rospy.Rate(100)  # create a rate object to run loop at 100Hz
        while not rospy.is_shutdown():
            if serial_port.in_waiting >= 62:
                data = serial_port.read_until(b'\x0D')
                if data.startswith(b'\x0A'):  # check for 0x0A byte at beginning
                    data_str = data[1:-1].decode('utf-8').strip().split()  # extract string between 0x0A and 0x0D
                    try:
                        self.mpu_msg.roll = float(int(data_str[0])/1000)
                        self.mpu_msg.pitch = float(int(data_str[1])/1000)
                        self.mpu_msg.yaw = float(int(data_str[2])/1000)
                        self.mpu_pub.publish(self.mpu_msg)
                    except ValueError:
                        continue
                    except IndexError:
                        continue
                    print(data_str)
            rate.sleep()
    
if __name__ == '__main__':
    try:
        imu_serial = MPU()
        imu_serial.receive_imu_serial()
    except rospy.ROSInterruptException:
        pass
        
