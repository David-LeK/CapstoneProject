#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
//#include <sensor_msgs/PointCloud.h>
//#include <sensor_msgs/ChannelFloat32.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Float32.h>
//#include <librealsense2/rs.hpp>

// Camera parameters
double fx = 596.413;
double fy = 596.413;
double cx = 420.8239;
double cy = 245.0385;
double baseline = 0.0055;
double depth_scale = 0.001;
double yaw_cam = 0.0;

//int x_min_test = 0.0;

double test_distance =0.0;
unsigned short test_depth_value =0.0;
// Extrinsic parameters
cv::Mat right_camera_rotation;
cv::Mat right_camera_translation;
cv::Mat left_camera_rotation;
cv::Mat left_camera_translation;

cv::Mat depth_roi;

//sensor_msgs::PointCloud object_info;
//sensor_msgs::ChannelFloat32 distance_channel;
//geometry_msgs::Point32 point_data;

cv_bridge::CvImagePtr cv_ptr;

void depthImageCallback(const sensor_msgs::ImageConstPtr& depth_msg)
{
    //ROS_INFO("Receive topic");
    try
    {
        cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    //depth_roi = cv_ptr->image(cv::Rect(xmin, ymin, xmax-xmin, ymax-ymin));
}

void bboxCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg) {
    //ROS_INFO("Receive bounding box");
    //object_info.points.clear();
    //object_info.channels.clear();
    int num_boxes = msg->bounding_boxes.size();
    //double median_depth;
    unsigned short distance;
    for (int i = 0; i < num_boxes; i++) {
        int xmin = msg->bounding_boxes[i].xmin;
        int ymin = msg->bounding_boxes[i].ymin;
        int xmax = msg->bounding_boxes[i].xmax;
        int ymax = msg->bounding_boxes[i].ymax;
	if (!cv_ptr) {
	ROS_ERROR("cv_ptr is null!");
	return;
	}
	

	
	//rs2::frameset frames;
	//rs2::depth_frame depth_frame = frames.get_depth_frame();

        //depth_roi = cv_ptr->image(cv::Rect(xmin, ymin, xmax-xmin, ymax-ymin));
	
	depth_roi = cv_ptr->image;
	distance = depth_roi.at<ushort>((ymin + ymax) / 2,(xmin + xmax) / 2);
	test_depth_value = distance;
	//cv::Scalar mean_depth = cv::mean(depth_roi);
	//median_depth = mean_depth.val[0];
	//distance = depth_frame.get_distance((xmin + xmax) / 2, (ymin + ymax) / 2);
	//ROS_INFO("Object locate at x = %d ,y = %d",((xmin + xmax) / 2),((ymin + ymax) / 2));
/*
        cv::Mat depth_roi_sorted;
	cv::Mat depth_roi_reshaped = depth_roi.reshape(1, 1);
	if (depth_roi_reshaped.isContinuous()) {
		depth_roi_reshaped.copyTo(depth_roi_sorted);
		cv::sort(depth_roi_sorted, depth_roi_sorted, cv::SORT_ASCENDING);
		median_depth = depth_roi_sorted.at<float>(0, depth_roi_sorted.cols/2);
	} else {
		ROS_ERROR("Depth ROI matrix is not continuous!");
	}
*/
	//median_depth = cv::median(depth_roi);
        //distance = fx * baseline / median_depth;
        
        double obj_x_img = xmin + (xmax - xmin) / 2;
        double obj_y_img = ymin + (ymax - ymin) / 2;

        // Calculate object position in camera coordinates
        //double cam_x = median_depth * (obj_x_img - fx) / fx;
        //double cam_y = median_depth * (obj_y_img - fy) / fy;
        //double cam_z = median_depth;

        //double obj_easting = cam_easting + cam_x * cos(cam_yaw) - cam_y * sin(cam_yaw);
        //double obj_northing = cam_northing + cam_x * sin(cam_yaw) + cam_y * cos(cam_yaw);
        //double obj_altitude = cam_altitude + cam_z;
	/*
        point_data.x = 0.0;
        point_data.y = 0.0;
        point_data.z = 0.0;

        object_info.points[i] = point_data;
        distance_channel.values[i] = distance;
	*/
	test_distance = distance*depth_scale;
    }
    //object_info.channels[0] = distance_channel;
    //object_pub.pubish(object_info);
}

void yawCamCallback(const std_msgs::Float32ConstPtr& yaw_msg)
{
    yaw_cam = yaw_msg->data;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "distance_node");
    ros::NodeHandle nh;

    //distance_channel.name = "distance";

    // Subscribe to the left infrared image and camera info topics
    ros::Subscriber depth_sub = nh.subscribe("camera/depth/image_rect_raw", 100, depthImageCallback);
    ros::Subscriber bbox_sub = nh.subscribe("darknet_ros/bounding_boxes", 100, bboxCallback);
    ros::Subscriber yaw_cam_sub = nh.subscribe("/yaw",10,yawCamCallback);

    //ros::Publisher object_pub = nh.advertise<sensor_msgs::PointCloud>("object_data", 10);
    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        ROS_INFO("Distance to object: %f",test_distance);
	ROS_INFO("Depth value: %d",test_depth_value);
        //loop_rate.sleep();
	ros::spinOnce();
	loop_rate.sleep();
    }

    //ros::spin();

    return 0;
}


