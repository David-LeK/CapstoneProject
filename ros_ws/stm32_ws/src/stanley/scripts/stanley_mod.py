#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray
import math
from custom_msg.msg import obj_msgs, stanley_constants
from custom_msg.msg import encoder_input_msg, encoder_output_msg
from custom_msg.msg import gps_msg, stanley_outputs
from custom_msg.msg import mpu_msg
import numpy as np

class StanleyController(object):
    def __init__(self):
        rospy.init_node('stanley_controller')
        self.k = 0.5   # gain parameter for the cross-track error
        self.k_soft = 0.8 # Correction factor for low speeds
        self.max_speed = 0.5   # maximum speed of the car
        self.max_steering_angle = math.pi / 4   # maximum steering angle of the car
        self.current_path_index = 0
        self.search_offset = 5
        self.delta_t = 0.05 # time between updates (20 Hz)
        self.wheelbase = 0.5 # Distance between base to front wheel

        self.car_x = 0.0
        self.car_y = 0.0
        self.car_yaw = 0.0
        self.car_vel = 0.0
        
        self.v_left = 0.0 # m/s (from encoder)
        self.v_right = 0.0 # m/s (from encoder)
        self.v_linear = 0.0
        
        self.circumference = math.pi*0.165 # meter
        
        self.ref_x = []
        self.ref_y = []
        self.ref_yaw = []
        self.avoiding_state = False

        self.object_data = []
        
        self.u = 0 # Fuzzy Pi output
        self.pre_u = 0
        
        self.steering_angle = 0.0
        self.pre_steering_angle = 0.0
        self.flag = 0 
        self.cmd_vel = encoder_input_msg()
        self.stanley_msg = stanley_outputs()
        
        # Subscribe to messages
        rospy.Subscriber('/GPS_data', gps_msg, self.odom_callback)
        rospy.Subscriber('/path', Path, self.path_callback)
        rospy.Subscriber('/object', obj_msgs, self.object_callback)
        rospy.Subscriber('/MPU_data', mpu_msg, self.mpu_callback)
        rospy.Subscriber('/PID_data', encoder_output_msg, self.speed_callback)
        rospy.Subscriber('/Stanley_ctrl', stanley_constants, self.stanley_callback)

        # Publish commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', encoder_input_msg, queue_size=10)
        self.stanley_pub = rospy.Publisher('/Stanley_outputs', stanley_outputs, queue_size=10)
        self.cmd_vel_pub.publish(self.cmd_vel)

    def Pi_to_Pi(self, angle):
        pi = math.pi
        if angle > pi:
            angle = angle - 2 * pi
        elif angle < -pi:
            angle = angle + 2 * pi
        return angle
    
    def calculate_angles(self, x_array, y_array):
        angles = []
        for i in range(len(x_array) - 1):
            delta_x = x_array[i + 1] - x_array[i]
            delta_y = y_array[i + 1] - y_array[i]
            angle = math.atan2(delta_y, delta_x)
            angles.append(angle)
        angles.append(0.0)  # Appending the last angle as 0.0
        return angles
    
    def odom_callback(self, msg):
        # Update the car's current position and orientation
        self.car_x = msg.easting
        self.car_y = msg.northing
        
    def mpu_callback(self, msg):
        # Update the car's current position and orientation
        self.car_yaw = -self.Pi_to_Pi((msg.yaw - 90)*math.pi/180.0) # -pi to pi
        
    def speed_callback(self, msg):
        # Update the car's current speed from encoder
        self.v_left = msg.output_rpm_m1*self.circumference/60.0
        self.v_right = msg.output_rpm_m2*self.circumference/60.0
        
    def path_callback(self, msg):
        # Update the reference path
        self.ref_x = [poses.pose.position.x for poses in msg.poses]
        self.ref_y = [poses.pose.position.y for poses in msg.poses]
        self.ref_yaw = self.calculate_angles(self.ref_x, self.ref_y)
        
    def stanley_callback(self, msg):
        # Update the Stanley constant
        self.car_vel = msg.V_desired
        self.k = msg.K
        self.cmd_vel.input_Kp_m1 = msg.input_Kp_m1
        self.cmd_vel.input_Ki_m1 = msg.input_Ki_m1
        self.cmd_vel.input_Kd_m1 = msg.input_Kd_m1
        self.cmd_vel.input_Kp_m2 = msg.input_Kp_m2
        self.cmd_vel.input_Ki_m2 = msg.input_Ki_m2
        self.cmd_vel.input_Kd_m2 = msg.input_Kd_m2
        
    def avoidance_processing():
        pass    
        
    def object_callback(self, msg):
        num_obj = 0
        self.object_data.clear()
        for i in range(len(msg.distance)):
            if msg.distance[i] < 7:
                num_obj += 1
                data_element = [msg.distance[i],msg.northing[i],msg.easting[i]]
                self.object_data.append(data_element)
        if num_obj == 0:
            self.avoiding_state = False
        else:
            self.avoiding_state = True

    def calculate_steering_angle(self):
        # Step 1: Check if the vehicle has reached the target point
        if self.current_path_index >= len(self.ref_x):
            # Reached the end of the path
            self.steering_angle = 0.0
            return

        # Step 2: Check if reach the endpoint of path
        dx = self.ref_x[-1] - self.car_x
        dy = self.ref_y[-1] - self.car_y
        target_radius = math.sqrt(dx*dx + dy*dy)
        print("Target radius: "+ str(target_radius))
        if target_radius < 5 and self.current_path_index == (len(self.ref_x) - 1):
            # Trajectory has been completed, stop the robot
            self.steering_angle = 0.0
            self.car_vel = 0.0
            self.differential_controller()
            return

        # Step 3: Determine next waypoint
        min_distance = float('inf')
        dx = self.car_x - self.ref_x[self.current_path_index]
        dy = self.car_y - self.ref_y[self.current_path_index]
        print("dx: " + str(dx))
        print("dy: " + str(dy))
        distance = math.sqrt(dx*dx + dy*dy)
        # if distance < 0.1:
            # Next point reached, change to next index
        for i in range(self.current_path_index, min(self.current_path_index+self.search_offset, len(self.ref_x))):
            check_dx = self.ref_x[i] - self.car_x
            check_dy = self.ref_y[i] - self.car_y
            check_distance = math.sqrt(check_dx*check_dx + check_dy*check_dy)
            if check_distance < min_distance:
                min_distance = check_distance
                self.current_path_index = i
        print("Current path index: " + str(self.current_path_index))
        print("Current distance to path: "+str(distance))
        
        # Step 4: Calculate deviation angle and steering angle
        #e_fa = -(dx*math.cos(self.car_yaw + math.pi/2) + dy*math.sin(self.car_yaw + math.pi/2))
        e_fa = -(dx*math.cos(self.ref_yaw[self.current_path_index] + math.pi/2) + dy*math.sin(self.ref_yaw[self.current_path_index] + math.pi/2))
        theta_e = self.ref_yaw[self.current_path_index] - self.car_yaw
        self.v_linear = (self.v_left + self.v_right)/2
        theta_d = math.atan2(self.k * e_fa, self.v_linear + self.k_soft)
        delta = theta_e + theta_d
        print("Delta: " + str(delta))
        if abs(delta) > self.max_steering_angle:
            delta = math.copysign(self.max_steering_angle, delta)
            
        self.steering_angle = delta
        
        self.stanley_msg.e_fa = e_fa
        self.stanley_msg.theta_d = theta_d
        self.stanley_msg.theta_e = theta_e
        self.stanley_msg.v_linear = self.v_linear
        self.stanley_msg.delta = delta
        self.stanley_msg.steering_angle = self.steering_angle
        self.stanley_msg.car_yaw = self.car_yaw
        self.stanley_msg.ref_yaw = self.ref_yaw[self.current_path_index]
        self.stanley_msg.dx = dx
        self.stanley_msg.dy = dy
        self.stanley_msg.target_radius = target_radius
        self.stanley_msg.distance = distance
        self.stanley_msg.total_path_index = len(self.ref_x) - 1
        self.stanley_msg.current_path_index = self.current_path_index
        
        self.differential_controller()
        
        self.stanley_pub.publish(self.stanley_msg)

    def differential_controller(self):
        # Publish cmd_vel
        L_BacktoFront = 0.36
        L_LefttoRight = 0.385
        
        w = (self.car_vel * math.tan(self.steering_angle)) / L_BacktoFront
        self.stanley_msg.omega = w
        print("w:" + str(w))
        self.v_set_left = (2*self.car_vel - w*L_LefttoRight)/2
        self.v_set_right = (2*self.car_vel + w*L_LefttoRight)/2

        self.cmd_vel.input_setpoint_m1 = self.v_set_left*120
        self.cmd_vel.input_setpoint_m2 = self.v_set_right*120
        print("M1:"+ str(self.v_set_left*120))
        print("M2:" + str(self.v_set_right*120))
        self.cmd_vel_pub.publish(self.cmd_vel)
        

    def run(self):
        # Run the controller
        rate = rospy.Rate(20) # 20 Hz
        while not rospy.is_shutdown():
            if (self.avoiding_state):
                self.avoidance_processing()
                rate.sleep()
            else:
                print("*************************")
                self.calculate_steering_angle()
                print("Steering angle: " + str(self.steering_angle))
                if (self.flag == 3):
                    break
                elif (self.flag == 4):
                    self.ref_x.reverse()
                    self.ref_y.reverse()
                    self.ref_yaw = self.calculate_angles(self.ref_x, self.ref_y)
                    self.current_path_index = 0
                rate.sleep()

if __name__ == '__main__':
    try:
        controller = StanleyController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
