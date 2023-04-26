#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <geometry_msgs/Point32.h>
#include <custom_msg/obj_msgs.h>
#include <custom_msg/mpu_msg.h>
#include <custom_msg/gps_msg.h>

// Camera parameters
double fx = 596.413;
double fy = 596.413;
double cx = 420.8239;
double cy = 245.0385;
double baseline = 0.0055;
double depth_scale = 0.001;
double yaw_cam = 0.0;

double test_distance =0.0;
unsigned short test_depth_value =0.0;

double car_x,car_y = 0.0;

cv::Mat depth_roi;
cv_bridge::CvImagePtr cv_ptr;

void depthImageCallback(const sensor_msgs::ImageConstPtr& depth_msg)
{
    try
    {
        cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void bboxCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg) {
    int num_boxes = msg->bounding_boxes.size();

    for (int i = 0; i < num_boxes; i++) {
        int xmin = msg->bounding_boxes[i].xmin;
        int ymin = msg->bounding_boxes[i].ymin;
        int xmax = msg->bounding_boxes[i].xmax;
        int ymax = msg->bounding_boxes[i].ymax;
	if (!cv_ptr) {
	    ROS_ERROR("cv_ptr is null!");
	    return;
	}
	
	depth_roi = cv_ptr->image;
	unsigned short depth_value = depth_roi.at<ushort>((ymin + ymax) / 2,(xmin + xmax) / 2);
    double distance = depth_value*depth_scale;

    //Test value
	test_depth_value = depth_value;
    test_distance = depth_value*depth_scale;
    
    double obj_x_img = xmin + (xmax - xmin) / 2;
    double obj_y_img = ymin + (ymax - ymin) / 2;

        // Calculate object position in camera coordinates
        //double cam_x = median_depth * (obj_x_img - fx) / fx;
        //double cam_y = median_depth * (obj_y_img - fy) / fy;
        //double cam_z = median_depth;

        //double obj_easting = cam_easting + cam_x * cos(cam_yaw) - cam_y * sin(cam_yaw);
        //double obj_northing = cam_northing + cam_x * sin(cam_yaw) + cam_y * cos(cam_yaw);
        //double obj_altitude = cam_altitude + cam_z;
    }
    //object_pub.pubish(object_info);
}

void yawCamCallback(const custom_msg::mpu_msg::ConstPtr& yaw_msg)
{
    yaw_cam = yaw_msg->yaw;
}

void odomCallback(const custom_msg::gps_msg::ConstPtr& odom_msg)
{
    car_x = odom_msg->easting;
    car_y = odom_msg->northing;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "distance_node");
    ros::NodeHandle nh;

    // Subscribe to depth map topic
    ros::Subscriber depth_sub = nh.subscribe("camera/depth/image_rect_raw", 100, depthImageCallback);
    ros::Subscriber bbox_sub = nh.subscribe("darknet_ros/bounding_boxes", 100, bboxCallback);
    ros::Subscriber yaw_cam_sub = nh.subscribe("/yaw",10,yawCamCallback);
    ros::Subscriber odom_sub = nh.subscribe("/odom",10,odomCallBack);

    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        ROS_INFO("Distance to object: %f",test_distance);
	    ROS_INFO("Depth value: %d",test_depth_value);
	    ros::spinOnce();
	    loop_rate.sleep();
    }
    return 0;
}
