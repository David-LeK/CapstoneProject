#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray
# from geometry_msgs.msg import Twist, PoseStamped
import math
from custom_msg.msg import obj_msgs  
from custom_msg.msg import encoder_input_msg, encoder_output_msg
from custom_msg.msg import gps_msg
from custom_msg.msg import mpu_msg
import skfuzzy as fuzz
import numpy as np
import matplotlib.pyplot as plt

class StanleyController(object):
    def __init__(self):
        rospy.init_node('stanley_controller')
        self.k = 0.5   # gain parameter for the cross-track error
        self.k_soft = 1 # Correction factor for low speeds
        self.max_speed = 0.5   # maximum speed of the car
        self.max_steering_angle = math.pi / 4   # maximum steering angle of the car
        self.current_path_index = 0
        self.search_offset = 5
        self.delta_t = 0.05 # time between updates (20 Hz)
        self.wheelbase = 0.5 # Distance between base to front wheel

        self.car_x = 0.0
        self.car_y = 0.0
        self.car_yaw = 0.0
        
        self.v_left = 0.0 # m/s (from encoder)
        self.v_right = 0.0 # m/s (from encoder)
        self.v_fuzzy_left = 0.0 # m/s (from Fuzzy Pi)
        self.v_fuzzy_right = 0.0 # m/s (from Fuzzy Pi)
        
        self.circumference = math.pi*0.165 # meter
        
        # self.ref_x = [681392.75,681396.61]
        # self.ref_y = [1191301.73,1191295.89]
        self.ref_x = [677673.87, 677674.42]
        self.ref_y = [1217604.07,1217604.62]
        self.ref_yaw = [-math.pi/4, 0] # receive in radian

        self.avoiding_state = False

        self.object_data = []
        
        self.u = 0 # Fuzzy Pi output
        self.pre_u = 0
        
        self.steering_angle = 0.0
        self.pre_steering_angle = 0.0
        self.cmd_vel = encoder_input_msg()
        
        # Subscribe to messages
        rospy.Subscriber('/GPS_data', gps_msg, self.odom_callback)
        rospy.Subscriber('/path', Path, self.path_callback)
        rospy.Subscriber('/object', obj_msgs, self.object_callback)
        rospy.Subscriber('/PID_ctrl', encoder_input_msg, self.pid_callback)
        rospy.Subscriber('/MPU_data', mpu_msg, self.mpu_callback)
        rospy.Subscriber('/PID_data', encoder_output_msg, self.speed_callback)
        rospy.Subscriber('/ref_yaw', Float32MultiArray, self.yaw_callback)

        # Publish commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', encoder_input_msg, queue_size=10)

    def odom_callback(self, msg):
        # Update the car's current position and orientation
        self.car_x = msg.easting
        self.car_y = msg.northing
        
    def mpu_callback(self, msg):
        # Update the car's current position and orientation
        self.car_yaw = msg.yaw*math.pi/180.0
        
    def speed_callback(self, msg):
        # Update the car's current speed from encoder
        self.v_left = msg.output_rpm_m1*self.circumference/60.0
        self.v_right = msg.output_rpm_m2*self.circumference/60.0
        
    def yaw_callback(self, msg):
        # Update the car's reference car_yaw
        self.ref_yaw = msg.data

    def path_callback(self, msg):
        # Update the reference path
        self.ref_x = [poses.pose.position.x for poses in msg.poses]
        self.ref_y = [poses.pose.position.y for poses in msg.poses]
        
    def pid_callback(self, msg):
        # Update the PID constant
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

        # Step 2: Calculate current position of front wheel
        v_linear = (self.v_left + self.v_right) / 2.0
        x_wheel = self.car_x + math.cos(self.car_yaw) * self.wheelbase
        y_wheel = self.car_y + math.sin(self.car_yaw) * self.wheelbase
        x_deviation = v_linear * math.cos(self.car_yaw) * self.delta_t
        y_deviation = v_linear * math.sin(self.car_yaw) * self.delta_t
        x_wheel += x_deviation
        y_wheel += y_deviation
        print("Front wheel coords: " + str(x_wheel) + ", " + str(y_wheel))

        # Step 3: Check if the target radius is less than 0.5m
        dx = self.ref_x[-1] - x_wheel
        dy = self.ref_y[-1] - y_wheel
        target_radius = math.sqrt(dx*dx + dy*dy)
        print("Target radius: "+ str(target_radius))
        if target_radius < 0.5:
            # Trajectory has been completed, stop the robot
            self.steering_angle = 0.0
            return

        # Step 4: Determine next waypoint
        min_distance = float('inf')
        j = self.current_path_index
        for i in range(self.current_path_index, min(self.current_path_index+self.search_offset, len(self.ref_x))):
            dx = self.ref_x[i] - x_wheel
            dy = self.ref_y[i] - y_wheel
            distance = math.sqrt(dx*dx + dy*dy)
            if distance < 0.1:
                # Next point reached, increment index
                j += 1
            if distance < min_distance:
                min_distance = distance
                        
        print("Distance to nearest point: " + str(min_distance))
        #Update index
        self.current_path_index = j
        print("Current path index: " + str(self.current_path_index))

        # Step 5: Calculate deviation angle and steering angle
        e_fa = -(dx*math.cos(self.car_yaw) + dy*math.sin(self.car_yaw))
        theta_e = self.car_yaw - self.ref_yaw[self.current_path_index]
        v = v_linear
        theta_d = -math.atan2(self.k * e_fa, v + self.k_soft)
        delta = theta_e + theta_d
        print("Delta: " + str(delta))
        if abs(delta) > self.max_steering_angle:
            delta = math.copysign(self.max_steering_angle, delta)
            
        self.steering_angle = delta
        
    def fuzzy_pi_controller(self):
        # Calculate angular error and change in angular error
        e = self.steering_angle
        e_dot = (self.steering_angle - self.pre_steering_angle) / self.delta_t
        self.pre_steering_angle = self.steering_angle

        # Normalize input values
        e_normalized = e / np.pi
        e_dot_normalized = 6 / np.pi * e_dot

        # Define input and output variables
        e_range = np.arange(-1, 1.01, 0.01)
        e_dot_range = np.arange(-1, 1.01, 0.01)
        u_dot_range = np.arange(-1, 1.01, 0.01)

        # Define fuzzy membership functions for input and output variables
        e_nb = fuzz.trapmf(e_range, [-2, -1, -0.22, -0.17])
        e_ns = fuzz.trimf(e_range, [-0.22, -0.11, 0])
        e_ze = fuzz.trimf(e_range, [-0.11, 0, 0.11])
        e_ps = fuzz.trimf(e_range, [0, 0.11, 0.22])
        e_pb = fuzz.trapmf(e_range, [0.17, 0.22, 1, 2])

        e_dot_ne = fuzz.trapmf(e_dot_range, [-2, -1, -0.22, -0.17])
        e_dot_ze = fuzz.trimf(e_dot_range, [-0.4, 0, 0.4])
        e_dot_po = fuzz.trapmf(e_dot_range, [0.17, 0.22, 1, 2])
        
        u_dot_nb = fuzz.trimf(u_dot_range, [-1, -0.95, -0.8])
        u_dot_nm = fuzz.trimf(u_dot_range, [-0.95, -0.8, -0.4])
        u_dot_ns = fuzz.trimf(u_dot_range, [-0.8, -0.4, 0])
        u_dot_ze = fuzz.trimf(u_dot_range, [-0.4, 0, 0.4])
        u_dot_ps = fuzz.trimf(u_dot_range, [0, 0.4, 0.8])
        u_dot_pm = fuzz.trimf(u_dot_range, [0.4, 0.8, 1])
        u_dot_pb = fuzz.trimf(u_dot_range, [0.8, 1, 1])

        # Fuzzify input variables
        e_nb_membership = fuzz.interp_membership(e_range, e_nb, e_normalized)
        e_ns_membership = fuzz.interp_membership(e_range, e_ns, e_normalized)
        e_ze_membership = fuzz.interp_membership(e_range, e_ze, e_normalized)
        e_ps_membership = fuzz.interp_membership(e_range, e_ps, e_normalized)
        e_pb_membership = fuzz.interp_membership(e_range, e_pb, e_normalized)

        e_dot_ne_membership = fuzz.interp_membership(e_dot_range, e_dot_ne, e_dot_normalized)
        e_dot_ze_membership = fuzz.interp_membership(e_dot_range, e_dot_ze, e_dot_normalized)
        e_dot_po_membership = fuzz.interp_membership(e_dot_range, e_dot_po, e_dot_normalized)
        
        # Apply fuzzy rules
        # Rule 1: If error is NB and error_dot is NE, then output is NB
        rule1 = np.fmin(e_nb_membership, e_dot_ne_membership)
        u_nb_activation = np.fmin(rule1, u_dot_nb)

        # Rule 2: If error is NS and error_dot is NE, then output is NM
        rule2 = np.fmin(e_ns_membership, e_dot_ne_membership)
        u_nm_activation = np.fmin(rule2, u_dot_nm)

        # Rule 3: If error is ZE and error_dot is NE, then output is NS
        rule3 = np.fmin(e_ze_membership, e_dot_ne_membership)
        u_ns_activation = np.fmin(rule3, u_dot_ns)

        # Rule 4: If error is PS and error_dot is NE, then output is ZE
        rule4 = np.fmin(e_ps_membership, e_dot_ne_membership)
        u_ze_activation = np.fmin(rule4, u_dot_ze)

        # Rule 5: If error is PB and error_dot is NE, then output is PS
        rule5 = np.fmin(e_pb_membership, e_dot_ne_membership)
        u_ps_activation = np.fmin(rule5, u_dot_ps)

        # Rule 6: If error is NB and error_dot is ZE, then output is NM
        rule6 = np.fmin(e_nb_membership, e_dot_ze_membership)
        u_nm_activation_2 = np.fmin(rule6, u_dot_nm)

        # Rule 7: If error is NS and error_dot is ZE, then output is NS
        rule7 = np.fmin(e_ns_membership, e_dot_ze_membership)
        u_ns_activation_2 = np.fmin(rule7, u_dot_ns)

        # Rule 8: If error is ZE and error_dot is ZE, then output is ZE
        rule8 = np.fmin(e_ze_membership, e_dot_ze_membership)
        u_ze_activation_2 = np.fmax(rule8, u_dot_ze)

        # Rule 9: If error is PS and error_dot is ZE, then output is PS
        rule9 = np.fmin(e_ps_membership, e_dot_ze_membership)
        u_ps_activation_2 = np.fmin(rule9, u_dot_ps)

        # Rule 10: If error is PB and error_dot is ZE, then output is PM
        rule10 = np.fmin(e_pb_membership, e_dot_ze_membership)
        u_pm_activation = np.fmin(rule10, u_dot_pm)

        # Rule 11: If error is NB and error_dot is PO, then output is NS
        rule11 = np.fmin(e_nb_membership, e_dot_po_membership)
        u_ns_activation_3 = np.fmin(rule11, u_dot_ns)

        # Rule 12: If error is NS and error_dot is PO, then output is ZE
        rule12 = np.fmin(e_ns_membership, e_dot_po_membership)
        u_ze_activation_3 = np.fmin(rule12, u_dot_ze)

        # Rule 13: If error is ZE and error_dot is PO, then output is PS
        rule13 = np.fmin(e_ze_membership, e_dot_po_membership)
        u_ps_activation_3 = np.fmin(rule13, u_dot_ps)

        # Rule 14: If error is PS and error_dot is PO, then output is PM
        rule14 = np.fmin(e_ps_membership, e_dot_po_membership)
        u_pm_activation_2 = np.fmin(rule14,u_dot_pm)
        
        #Rule 15: If error is PB and error_dot is PO, then output is PB
        rule15 = np.fmin(e_pb_membership, e_dot_po_membership)
        u_pb_activation = np.fmin(rule15, u_dot_pb)

        # Aggregate all the output membership functions
        aggregated = np.fmax.reduce([u_nb_activation, u_nm_activation, u_ns_activation,
                                     u_ze_activation, u_ps_activation, u_pm_activation,
                                     u_ns_activation_2, u_ps_activation_2, u_pm_activation_2, 
                                     u_nm_activation_2, u_ns_activation_3, u_ze_activation_2, 
                                     u_ps_activation_3, u_ze_activation_3, u_pb_activation])
        
        # Defuzzify to get output
        u_dot = fuzz.defuzz(u_dot_range, aggregated, 'centroid')
        self.u = self.pre_u + u_dot*self.delta_t
        self.pre_u = self.u
        
        print("Fuzzy controller output: " + str(self.u))
        
        #Plot the membership functions and the aggregated output membership function
        # fig, ax0 = plt.subplots(figsize=(8, 3))
        # ax0.plot(u_dot_range, u_dot_nb, 'b', linewidth=1.5, label='NB')
        # ax0.plot(u_dot_range, u_dot_nm, 'g', linewidth=1.5, label='NM')
        # ax0.plot(u_dot_range, u_dot_ns, 'r', linewidth=1.5, label='NS')
        # ax0.plot(u_dot_range, u_dot_ze, 'm', linewidth=1.5, label='ZE')
        # ax0.plot(u_dot_range, u_dot_ps, 'y', linewidth=1.5, label='PS')
        # ax0.plot(u_dot_range, u_dot_pm, 'k', linewidth=1.5, label='PM')
        # ax0.fill_between(u_dot_range, u_ns_activation, facecolor='r', alpha=0.2)
        # ax0.plot(u_dot_range, u_ns_activation, 'r', linestyle='--', linewidth=0.5)
        # ax0.fill_between(u_dot_range, u_ze_activation, facecolor='m', alpha=0.2)
        # ax0.plot(u_dot_range, u_ze_activation, 'm', linestyle='--', linewidth=0.5)
        # ax0.fill_between(u_dot_range, u_ps_activation, facecolor='y', alpha=0.2)
        # ax0.plot(u_dot_range, u_ps_activation, 'y', linestyle='--', linewidth=0.5)
        # ax0.plot([u_dot, u_dot], [0, 1], 'k', linewidth=1.5, alpha=0.9)
        # ax0.legend()

        # plt.tight_layout()
        # plt.show()
        
        if self.u >= 0:
            self.v_fuzzy_left = (1 + self.u) * self.v_left
            self.v_fuzzy_right = (1 - self.u) * self.v_right
        else:
            self.v_fuzzy_left = (1 - self.u) * self.v_left
            self.v_fuzzy_right = (1 + self.u) * self.v_right
            
        print("Left motor fuzzy speed: " + str(self.v_fuzzy_left))
        print("Right motor fuzzy speed: " + str(self.v_fuzzy_right))
        
        # Publish cmd_vel
        self.cmd_vel.input_setpoint_m1 = self.v_fuzzy_left*120
        self.cmd_vel.input_setpoint_m2 = self.v_fuzzy_right*120
        self.cmd_vel_pub.publish(self.cmd_vel)
        

    def run(self):
        # Run the controller
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            if (self.avoiding_state):
                self.avoidance_processing()
                rate.sleep()
            else:
                print("*************************")
                self.calculate_steering_angle()
                print("Steering angle: " + str(self.steering_angle))
                self.fuzzy_pi_controller()
                rate.sleep()

if __name__ == '__main__':
    try:
        controller = StanleyController()
        controller.run()
    except rospy.ROSInterruptException:
        pass