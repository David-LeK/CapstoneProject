#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
import math
from custom_msg.msg import obj_msgs  

class StanleyController(object):
    def __init__(self):
        rospy.init_node('stanley_controller')
        self.k = 0.5   # gain parameter for the cross-track error
        self.max_speed = 1.0   # maximum speed of the car
        self.max_steering_angle = math.pi / 4   # maximum steering angle of the car
        self.current_path_index = 0
        self.car_x = 0.0
        self.car_y = 0.0
        self.car_yaw = 0.0
        self.ref_easting = []
        self.ref_northing = []
        self.fuzzy_on = False

        # Subscribe to messages
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/path', Path, self.path_callback)
        rospy.Subscriber('/object', Path, self.object_callback)

        # Publish commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def odom_callback(self, msg):
        # Update the car's current position and orientation
        self.car_x = msg.pose.pose.position.x
        self.car_y = msg.pose.pose.position.y
        self.car_yaw = msg.pose.pose.orientation.z

    def path_callback(self, msg):
        # Update the reference path
        self.ref_easting = [pose.pose.position.x for pose in msg.poses]
        self.ref_northing = [pose.pose.position.y for pose in msg.poses]
        
    def fuzzy_processing():
        pass    
        
    def object_callback(self, msg):
        num_obj = 0
        obj_distance = []
        obj_northing = []
        obj_easting = []
        for i in range(len(msg.distance)):
            if msg.distance[i] < 7:
                num_obj += 1
                obj_distance.append(msg.distance[i])
                obj_northing.append(msg.northing[i])
                obj_easting.append(msg.easting[i])
        if num_obj == 0:
            self.fuzzy_on = False
        else:
            self.fuzzy_on = True

    def calculate_steering_angle(self):
        if not self.ref_easting or not self.ref_northing:
            # No reference path available
            return
        
        if self.current_path_index >= len(self.ref_easting):
            # Reached the end of the path
            return
        
        # Calculate lateral error
        car_direction_x = math.cos(self.car_yaw)
        car_direction_y = math.sin(self.car_yaw)
        dx = self.ref_easting[self.current_path_index] - self.car_x
        dy = self.ref_northing[self.current_path_index] - self.car_y
        lateral_error = -dy * car_direction_x + dx * car_direction_y

        # Check if the car has reached the current point on the path
        distance_to_current_point = math.sqrt(dx*dx + dy*dy)
        if distance_to_current_point < 0.5 and self.current_path_index < len(self.ref_easting) - 1:
            # Update the current path index to the next point on the path
            self.current_path_index += 1

        # Calculate the desired steering angle using the Stanley control algorithm
        steering_angle = math.atan2(self.k * lateral_error, self.max_speed)

        # Limit steering angle
        steering_angle = max(-self.max_steering_angle, min(steering_angle, self.max_steering_angle))

        velocity = self.calculate_velocity()

        # Publish cmd_vel
        cmd_vel = Twist()
        cmd_vel.linear.x = velocity
        cmd_vel.angular.z = steering_angle
        self.cmd_vel_pub.publish(cmd_vel)

    def calculate_velocity(self):
        # Calculate the car's current speed from the odometry data
        return math.sqrt( self.car_x**2 + self.car_y**2 )

    def run(self):
        # Run the controller
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            if (self.fuzzy_on):
                self.fuzzy_processing()
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
