import rospy
import math
import utm
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray, Int16
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, Imu
from custom_msg.msg import gps_msg, mpu_msg, stanley_constants, stanley_outputs, obj_msgs
from tf.transformations import euler_from_quaternion

class StanleyController(object):
    def __init__(self):
        rospy.init_node('stanley_controller')
        # Run the controller
        rospy.Subscriber('/ublox_gps/fix', NavSatFix, self.gps_callback)
        rospy.Subscriber('/imu', Imu, self.imu_callback)
        rospy.Subscriber('/path', Path, self.path_callback)
        rospy.Subscriber('/Stanley_ctrl', stanley_constants, self.stanley_callback)
        rospy.Subscriber('/stop', Int16, self.state_callback)
        rospy.Subscriber('/object', obj_msgs, self.object_callback)
        
        self.gps_msg = gps_msg()
        self.imu_msg = mpu_msg()
        self.cmd_vel = Twist()
        self.stanley_msg = stanley_outputs()
        
        self.gps_pub = rospy.Publisher('/GPS_data', gps_msg, queue_size=10)
        self.imu_pub = rospy.Publisher('/MPU_data', mpu_msg, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/jetbot_velocity_controller/cmd_vel', Twist, queue_size=10)
        self.stanley_pub = rospy.Publisher('/Stanley_outputs', stanley_outputs, queue_size=10)
        
        self.k = 0.5   # gain parameter for the cross-track error
        self.k_soft = 0.8 # Correction factor for low speeds
        self.max_speed = 0.5   # maximum speed of the car
        self.max_steering_angle = math.pi / 4   # maximum steering angle of the car
        self.current_path_index = 0
        self.search_offset = 5

        self.car_x = 0.0
        self.car_y = 0.0
        self.car_yaw = 0.0
        self.car_vel = 0.0
        
        self.v_left = 0.0 # m/s (from encoder)
        self.v_right = 0.0 # m/s (from encoder)
        self.v_linear = 0.0
        
        self.ref_x = []
        self.ref_y = []
        self.ref_yaw = [] # receive in radian    
        self.flag = 0 
        
        self.w_avoid_low = 10*math.pi/180
        self.w_avoid_med = 30*math.pi/180
        self.w_avoid_high = 45*math.pi/180
        self.avoiding_state = False
        self.pre_avoid = False
        self.object_x = 0.0
        self.object_distance = 0.0
        
    def __del__(self):
        print("Resetting...")
    
    def avoidance_processing(self):
        avoid_steering = 0
        x = self.object_x
        z = self.object_distance
        if (z>1 and z < 2):
            avoid_steering = self.w_avoid_low
        elif(z<1 and z>=0.5):
            if(abs(x)>0.4):
                avoid_steering = self.w_avoid_low
            if(abs(x)<0.4):
                avoid_steering = self.w_avoid_med
        elif(z<0.5):
            if(abs(x)>0.4):
                avoid_steering = self.w_avoid_med
            if(abs(x)<0.4):
                avoid_steering = self.w_avoid_high
        if(x<0):
            self.steering_angle = -avoid_steering
        else:
            self.steering_angle = avoid_steering
        print("Avoid angle: " + str(math.degrees(self.steering_angle)))    
        self.differential_controller()
        
    def object_callback(self, msg):
        if 0 < msg.distance <= 5 and 0 < abs(msg.x) <= 0.5:
            self.object_x = msg.x
            self.object_distance = msg.distance
            self.avoiding_state = True
        else:
            self.avoiding_state = False
            
    def return_stanley(self):
        min_distance_return = float('inf')
        next_index = 0
        for i in range(self.current_path_index,len(self.ref_x)):
            dx = self.car_x - self.ref_x[self.current_path_index]
            dy = self.car_y - self.ref_y[self.current_path_index]
            distance = math.sqrt(dx*dx + dy*dy)
            if distance < min_distance_return:
                min_distance_return = distance
                next_index = i
        self.current_path_index = next_index
    
    def calculate_angles(self, x_array, y_array):
        angles = []
        for i in range(len(x_array) - 1):
            delta_x = x_array[i + 1] - x_array[i]
            delta_y = y_array[i + 1] - y_array[i]
            angle = math.atan2(delta_y, delta_x)
            angles.append(angle)
        angles.append(0.0)  # Appending the last angle as 0.0
        return angles

    def imu_callback(self, msg):
        roll, pitch, yaw = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        yaw = -self.Pi_to_Pi(yaw - math.pi/2)
        self.imu_msg.roll = math.degrees(roll)
        self.imu_msg.pitch = math.degrees(pitch)
        self.imu_msg.yaw = math.degrees(yaw)
        self.car_yaw = -self.Pi_to_Pi(yaw - math.pi/2)
        self.imu_pub.publish(self.imu_msg)

    def Pi_to_Pi(self, angle):
        pi = math.pi
        if angle > pi:
            angle = angle - 2 * pi
        elif angle < -pi:
            angle = angle + 2 * pi
        return angle        

    def gps_callback(self,msg):
        self.gps_msg.latitude = msg.latitude
        self.gps_msg.longitude = msg.longitude
        utm_coord = utm.from_latlon(msg.latitude, msg.longitude)
        self.gps_msg.easting = utm_coord[0]
        self.gps_msg.northing = utm_coord[1]
        self.car_x = utm_coord[0]
        self.car_y = utm_coord[1]
        self.gps_msg.speed_kmh = 0
        self.gps_msg.tracking_angle = 0
        self.gps_pub.publish(self.gps_msg)

    def path_callback(self, msg):
        # Update the reference path
        self.ref_x = [poses.pose.position.x for poses in msg.poses]
        self.ref_y = [poses.pose.position.y for poses in msg.poses]
        self.ref_yaw = self.calculate_angles(self.ref_x, self.ref_y)
        
    def stanley_callback(self, msg):
        # Update the Stanley constant
        self.car_vel = msg.V_desired
        self.k = msg.K

    def state_callback(self, msg):
        # Update the car's reference car_yaw
        self.flag = msg.data

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
        #if distance < 2.5:
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
        theta_e = self.Pi_to_Pi(self.ref_yaw[self.current_path_index] - self.car_yaw)
        self.v_linear = self.car_vel
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
        
        w = (self.car_vel * math.tan(self.steering_angle)) / L_BacktoFront
        self.stanley_msg.omega = w
        self.cmd_vel.linear.x = self.car_vel
        self.cmd_vel.angular.z = w
        self.cmd_vel_pub.publish(self.cmd_vel)

    def run(self):
    # Run the controller
        rate = rospy.Rate(20) # 20 Hz
        while not rospy.is_shutdown():
            if (self.avoiding_state):
                self.avoidance_processing()
            else:
                self.calculate_steering_angle()
                if([self.pre_avoid, self.avoiding_state] == [1,0]):
                    self.return_stanley()
                    print("Going back to Stanley")
                if (self.flag == 3):
                    break
                elif (self.flag == 4):
                    self.ref_x.reverse()
                    self.ref_y.reverse()
                    self.ref_yaw = self.calculate_angles(self.ref_x, self.ref_y)
                    self.current_path_index = 0
            self.pre_avoid = self.avoiding_state
            rate.sleep()

if __name__ == '__main__':
    try:
        while True:
            controller = StanleyController()
            controller.run()
            del controller
    except rospy.ROSInterruptException:
        pass