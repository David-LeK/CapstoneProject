#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <geodesy/utm.h>

	double max_speed_;
	double max_steering_angle_;
	double k_ = 1.0;
	double k_v_ = 0.1;
	double car_yaw = 0.0;
	double lat = 0.0;
	double lon = 0.0;
	double car_x = 0.0;
	double car_y = 0.0;
	
	int zone;
	bool northp;
	
	int current_path_index = 0;
	
	//nav_msgs::Path path_;
	std::vector<float> ref_easting; //X_ref
	std::vector<float> ref_northing; //Y_ref
	std::vector<float> ref_angle; //Theta_ref
	
	
	void yawCallback()(const std_msgs::Float64::ConstPtr& msg) {
		//Save yaw
		car_yaw = msg->data;
	}
	
	void odomCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
		// Save odometry
		lat = msg->latitude;
		lon = msg->longitude;
		UTMUPS::Forward(lat, lon, zone, northp, car_x, car_y);
	}

	void eastingCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
		ref_easting = msg->data;
		if(ref_easting.size()==0){
			ROS_INFO("No easting data receive, Please send data again");
			return;
		}
		for (int i = 0; i < ref_easting.size(); i++) {
			ROS_INFO("Easting: %f", ref_easting[i]);
		}
	}	
	
	void northingCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
		ref_northing = msg->data;
		if(ref_northing.size()==0){
			ROS_INFO("No easting data receive, Please send data again");
			return;
		}
		for (int i = 0; i < ref_northing.size(); i++) {
			ROS_INFO("Northing: %f", ref_northing[i]);
		}
	}

	void pathCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
		ros::Subscriber northing_ = nh_.subscribe("path", 1000, northingCallback);
		ros::Subscriber easting_ = nh_.subscribe("path", 1000, eastingCallback);
	}
	
	double calculateVelocity() {
		// Calculate velocity as a function of cross-track error
		//double velocity = max_speed_ / (1.0 + k_v_*std::abs(calculateCrossTrackError()));
		double velocity = 0;
		//velocity = std::max(0.0, std::min(max_speed_, velocity));
		return velocity;
	}
	
	void calculateSteeringAngle() {
		/*
		car_x, car_y: The car's current position in the x and y coordinates
		car_yaw: The car's current orientation in radians
		ref_easting, ref_northing: The x and y coordinates of the current point on the path
		k: The gain parameter for the cross-track error
		max_speed: The maximum speed of the car
		max_steering_angle: The maximum steering angle of the car
		*/
		
		// Calculate lateral error
		double car_direction_x = std::cos(car_yaw);
		double car_direction_y = std::sin(car_yaw);
		double dx = ref_easting[current_path_index] - car_x;
		double dy = ref_northing[current_path_index] - car_y;
		double lateral_error = -dy * car_direction_x + dx * car_direction_y;
		
		// Check if the car has reached the current path index
		double distance_to_current_point = std::sqrt(dx*dx + dy*dy);
		if (distance_to_current_point < 0.5 && current_path_index < path_.poses.size() - 1) {
			// Update the current path index to the next point on the path
			current_path_index++;
		}
		
		// Calculate the desired steering angle using the Stanley control algorithm
		double steering_angle = atan2(k * lateral_error, max_speed);
		
		
		// Limit steering angle
		steering_angle = std::max(-max_steering_angle, std::min(steering_angle, max_steering_angle));
		
		double velocity = calculateVelocity();
		
		// Publish cmd_vel
		geometry_msgs::Twist cmd_vel;
		cmd_vel.linear.x = velocity;
		cmd_vel.angular.z = steering_angle;
		cmd_vel_pub_.publish(cmd_vel);
	}
	
	int main(int argc, char** argv) {
	ros::init(argc, argv, "stanley_controller_node");
	ros::NodeHandle nh_;
	
	// Subcribe to topics
	ros::Subscriber path_sub_ = nh_.subscribe("path", 1000, pathCallback);

    ros::Subscriber odom_sub_ = nh_.subscribe("/ublox/fix", 1000, odomCallback);
	ros::Subscriber yaw_sub_ = nh_.subscribe("yaw", 1000,yawCallback);
	
	//Pubish topic
	ros::Publisher cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_topic", 10);
	
	// Set the loop rate to 10Hz
	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		loop_rate.sleep();
		calculateSteeringAngle();
	}
	return 0;
	}
	
