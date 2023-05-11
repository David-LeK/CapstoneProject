#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
# from geometry_msgs.msg import Twist, PoseStamped
import math
from custom_msg.msg import obj_msgs  
from custom_msg.msg import encoder_input_msg
from custom_msg.msg import gps_msg
from custom_msg.msg import mpu_msg

class StanleyController(object):
    def __init__(self):
        rospy.init_node('stanley_controller')
        self.k = 0.5   # gain parameter for the cross-track error
        self.max_speed = 0.5   # maximum speed of the car
        self.max_steering_angle = math.pi / 4   # maximum steering angle of the car
        self.current_path_index = 0

        self.car_x = 0.0
        self.car_y = 0.0
        self.car_yaw = 0.0
        self.car_vel = 0.0

        # self.ref_x = [681392.75,681396.61]
        # self.ref_y = [1191301.73,1191295.89]
        self.ref_x = [677694.42, 677714.20]
        self.ref_y = [1217644.66,1217684.26]

        self.avoiding_state = False

        self.object_data = []

        self.car_m1 = 0.0
        self.car_m2 = 0.0
        self.steering_angle = 0.0
        self.cmd_vel = encoder_input_msg()
        
        # Subscribe to messages
        rospy.Subscriber('/GPS_data', gps_msg, self.odom_callback)
        #rospy.Subscriber('/path', Path, self.path_callback)
        rospy.Subscriber('/object', obj_msgs, self.object_callback)
        rospy.Subscriber('/PID_ctrl', encoder_input_msg, self.pid_callback)
        rospy.Subscriber('/MPU_data', mpu_msg, self.mpu_callback)

        # Publish commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', encoder_input_msg, queue_size=10)

    def odom_callback(self, msg):
        # Update the car's current position and orientation
        self.car_x = msg.easting
        self.car_y = msg.northing
        self.car_vel = (msg.speed_kmh*5)/18
        
    def mpu_callback(self, msg):
        # Update the car's current position and orientation
        self.car_yaw = msg.yaw*math.pi/180.0

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
        if not self.ref_x or not self.ref_y:
            # No reference path available
            return
        
        if self.current_path_index >= len(self.ref_x):
            # Reached the end of the path
            return
        
        # Calculate lateral error
        car_direction_x = math.cos(self.car_yaw)
        
        car_direction_y = math.sin(self.car_yaw)
        dx = self.ref_x[self.current_path_index] - self.car_x
        print("car_x "+ str(self.car_x))
        print("car_y "+ str(self.car_y))
        print("dx "+ str(dx))
        dy = self.ref_y[self.current_path_index] - self.car_y
        print("dy "+ str(dy))
        lateral_error = -dy * car_direction_x + dx * car_direction_y
        print("lateral_error "+ str(lateral_error))
        # Check if the car has reached the current point on the path
        distance_to_current_point = math.sqrt(dx*dx + dy*dy)
        print("distance " + str(distance_to_current_point))

        if distance_to_current_point < 0.5 and self.current_path_index < len(self.ref_x) - 1:
            # Update the current path index to the next point on the path
            self.current_path_index += 1

        # Calculate the desired steering angle using the Stanley control algorithm
        self.steering_angle = math.atan2(self.k * lateral_error, self.max_speed)
        print("Steering " + str(math.degrees(self.steering_angle)))
        # Limit steering angle
        self.steering_angle = max(-self.max_steering_angle, min(self.steering_angle, self.max_steering_angle))
        print("Limit Steering " + str(math.degrees(self.steering_angle)))
        
        self.calculate_velocity()

        # Publish cmd_vel
        self.cmd_vel.input_setpoint_m1 = self.car_m1*120
        self.cmd_vel.input_setpoint_m2 = self.car_m2*120
        
        print("Car Vel " + str(self.car_m1*120) +  " " +str(self.car_m2**120))
        self.cmd_vel_pub.publish(self.cmd_vel)

    def calculate_velocity(self):
        # Calculate the car's current speed from the odometry data
        # return math.sqrt( self.car_x**2 + self.car_y**2 )
        # L is the distance between 2 wheels
        L = 0.5
        w = (2 * self.car_vel * math.tan(self.steering_angle)) / L
        self.car_m1 = (2*self.car_vel - w*L)/2
        self.car_m2 = (2*self.car_vel + w*L)/2

    def run(self):
        # Run the controller
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            if (self.avoiding_state):
                self.avoidance_processing()
                rate.sleep()
            else:
                self.calculate_steering_angle()
                rate.sleep()

if __name__ == '__main__':
    try:
        controller = StanleyController()
        controller.run()
    except rospy.ROSInterruptException:
        pass