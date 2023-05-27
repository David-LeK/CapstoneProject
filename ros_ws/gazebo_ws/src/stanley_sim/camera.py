#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
import numpy as np
from custom_msg.msg import obj_msgs
import cv2

class ImageProcessor:
    def __init__(self):
        self.fx = None  # Focal length along the x-axis
        self.fy = None  # Focal length along the y-axis
        self.cx = None  # Principal point x-coordinate
        self.cy = None  # Principal point y-coordinate
        self.depth_image = None  # Depth image
        self.bridge = CvBridge()

        self.object_msg = obj_msgs()
        rospy.init_node("image_listener")

        # Subscribe to the camera info and depth image topics
        self.camera_info_sub = rospy.Subscriber("/realsense_d435/depth/camera_info", CameraInfo, self.camera_info_callback)
        self.depth_image_sub = rospy.Subscriber("/realsense_d435/depth/image_raw", Image, self.depth_image_callback)

        # Wait for camera info and depth image to be received before starting the image callback
        while self.fx is None or self.fy is None or self.cx is None or self.cy is None or self.depth_image is None:
            rospy.sleep(0.1)

        self.image_sub = rospy.Subscriber("/realsense_d435/color/image_raw", Image, self.image_callback)
        self.obj_pub = rospy.Publisher('/object', obj_msgs, queue_size=10)

    def camera_info_callback(self, msg):
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]

    def depth_image_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.depth_image = cv2.resize(depth_image, (640, 480))

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Define the desired color (e.g., red pixel)
            desired_color = np.array([0, 0, 100])  # BGR color values

            # Find the pixels where the RGB values match the desired color
            matching_pixels = np.where(np.all(cv_image == desired_color, axis=-1))

            # Create a list of pixel coordinates
            desired_color_pixels = list(zip(matching_pixels[0], matching_pixels[1]))
            if desired_color_pixels is not None:
                depth_array = np.array(self.depth_image, dtype=np.float32)
                
                depth_scale = 0.001
                zDepth = 0.0
                pixel_x = 0
                pixel_y = 0
                min_distance = float('inf')
                for coord in desired_color_pixels:
                    zDepth = depth_array[coord[0],coord[1]]
                    if 0 < zDepth < min_distance:
                            min_distance = zDepth
                            pixel_x = coord[1]
                            pixel_y = coord[0]
                    if 0 < zDepth:
                        pixel_x = coord[1]
                        pixel_y = coord[0]
                
                depth = depth_array[pixel_y, pixel_x]

                # Calculate the distance from the camera to the nearest pixel
                distance = depth / np.sqrt((pixel_y - self.cx)**2 / self.fx**2 + (pixel_x - self.cy)**2 / self.fy**2 + 1)
                x = (pixel_x / self.fx) - 0.5
                
                distance = distance*depth_scale

                print("Distance:", distance, "meters")
                print("X:", x, "meters")
                
                self.object_msg.distance = distance
                self.object_msg.x = x
                #self.obj_pub.publish(self.object_msg)
            else:
                print("The desired color does not exist in the image.")
                self.object_msg.distance = 0.0
                self.object_msg.x = 0.0
                #self.obj_pub.publish(self.object_msg)

        except Exception as e:
            print(e)
            
    def run(self):
        rate = rospy.Rate(1) # 10 Hz
        while not rospy.is_shutdown():
            self.obj_pub.publish(self.object_msg)
            rate.sleep()
            
if __name__ == "__main__":
    try:
        image_processor = ImageProcessor()
        image_processor.run()
    except rospy.ROSInterruptException:
        pass