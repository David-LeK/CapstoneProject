#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from custom_msg.msg import encoder_input_msg, encoder_output_msg
import math

class ManualController(object):
    def __init__(self):
        rospy.init_node('manual_controller')
        
        self.r = 0.165/2 # meter
        self.linear_x = self.linear_y = self.linear_z = 0.0
        self.angular_x = self.angular_y = self.angular_z = 0.0
        self.current_v_left = self.current_v_right = 0.0
        self.set_v_left = self.set_v_right = 0.0
        self.pid_data = encoder_output_msg()
        self.cmd_vel_msg = encoder_input_msg()
        
        # Subscribe to messages
        rospy.Subscriber('/cmd_vel_twist', Twist, self.odom_callback)
        rospy.Subscriber('/PID_data', encoder_output_msg, self.pid_callback)
        
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', encoder_input_msg, queue_size=10)
        
    def odom_callback(self, msg):
        self.linear_x = msg.linear.x
        self.linear_y = msg.linear.y
        self.linear_z = msg.linear.z
        self.angular_x = msg.angular.x
        self.angular_y = msg.angular.y
        self.angular_z = msg.angular.z
        
    def pid_callback(self, msg):
        self.current_v_left = msg.output_rpm_m1*2*math.pi*self.r/60.0
        self.current_v_right = msg.output_rpm_m2*2*math.pi*self.r/60.0
        self.pid_data.output_rpm_m1 = msg.output_rpm_m1
        self.pid_data.output_rpm_m2 = msg.output_rpm_m2
        self.pid_data.output_controller_m1 = msg.output_controller_m1
        self.pid_data.output_controller_m2 = msg.output_controller_m2
        self.pid_data.error_m1 = msg.error_m1
        self.pid_data.error_m2 = msg.error_m2
    
    def publish_setpoint(self, v_left, v_right):
        self.cmd_vel_msg.input_Kp_m1 = 1.1
        self.cmd_vel_msg.input_Ki_m1 = 1.1
        self.cmd_vel_msg.input_Kd_m1 = 0.0
        self.cmd_vel_msg.input_Kp_m2 = 1.1
        self.cmd_vel_msg.input_Ki_m2 = 1.1
        self.cmd_vel_msg.input_Kd_m2 = 0.0
        self.cmd_vel_msg.input_setpoint_m1 = v_left / (2*math.pi*self.r) * 60
        self.cmd_vel_msg.input_setpoint_m2 = v_right / (2*math.pi*self.r) * 60
        self.cmd_vel_pub.publish(self.cmd_vel_msg)
    
    def ddr_ik(self, v_desired, omega, L, r):
        #v_desire (m/s)
        #omega desired angle (rad/s)
        #L distance between two wheels (m)
        #r radius of wheel (m)
        return (v_desired - (L/2)*omega), (v_desired + (L/2)*omega)
    
    
    def calculate_velocity(self):
        L = 0.385
        
        #Vmax of this motor is 1.3 m/s
        phidot_L, phidot_R = self.ddr_ik(self.linear_x, self.angular_z, L, self.r)
        print(phidot_L, phidot_R)
        self.publish_setpoint(phidot_L, phidot_R)
    
    def run(self):
        # Run the controller
        rate = rospy.Rate(20) # 20 Hz
        while not rospy.is_shutdown():
            self.calculate_velocity()
            rate.sleep()
    
if __name__ == '__main__':
    try:
        controller = ManualController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
        