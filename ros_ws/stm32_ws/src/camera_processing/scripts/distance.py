import rospy
from sensor_msgs.msg import Image
from custom_msg.msg import obj_msgs
from sensor_msgs.msg import image_encodings
from cv_bridge import CvBridge
from darknet_ros_msgs.msg import BoundingBoxes
import numpy as np
import cv2


class DistanceNode:
    def __init__(self):
        # Camera parameters
        self.fx = 894.6195678710938
        self.fy = 894.6195678710938
        self.cx = 635.2359008789062
        self.cy = 367.5578308105469
        self.baseline = 0.0055
        self.depth_scale = 0.001
        self.yaw_cam = 0.0

        self.test_X = 0.0
        self.test_Y = 0.0
        self.test_Z = 0.0
        self.test_distance = 0.0
        self.test_depth_value = 0

        self.object_info = obj_msgs()
        self.object_info.distance = []
        self.object_info.x = []

        self.depth_roi = None
        self.cv_bridge = CvBridge()

        rospy.Subscriber("camera/depth/image_rect_raw", Image, self.depth_image_callback)
        rospy.Subscriber("darknet_ros/bounding_boxes", BoundingBoxes, self.bbox_callback)

        self.object_pub = rospy.Publisher("object", obj_msgs, queue_size=1000)

        self.rate = rospy.Rate(1)  # 1 Hz

    def depth_image_callback(self, depth_msg):
        try:
            self.depth_roi = self.cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr("CvBridge exception: %s", e)

    def bbox_callback(self, msg):
        num_boxes = len(msg.bounding_boxes)
        self.object_info.distance.clear()
        self.object_info.x.clear()

        for i in range(num_boxes):
            xmin = msg.bounding_boxes[i].xmin
            ymin = msg.bounding_boxes[i].ymin
            xmax = msg.bounding_boxes[i].xmax
            ymax = msg.bounding_boxes[i].ymax

            if self.depth_roi is None:
                rospy.logerr("depth_roi is None!")
                return

            depth_value = self.depth_roi[int((ymin + ymax) / 2), int((xmin + xmax) / 2)]
            distance = depth_value * self.depth_scale

            # Test value
            self.test_distance = distance

            u = xmin + (xmax - xmin) / 2
            v = ymin + (ymax - ymin) / 2

            x_norm = distance * ((u - self.cx) / self.fx)
            y_norm = distance * ((v - self.cy) / self.fy)
            z_norm = distance

            self.test_X = x_norm
            self.test_Y = y_norm
            self.test_Z = z_norm

            self.object_info.distance.append(distance)
            self.object_info.x.append(x_norm)

            self.test_depth_value = depth_value

    def run(self):
        rospy.init_node("distance_node")
        rospy.loginfo("Starting distance_node")

        while not rospy.is_shutdown():
            rospy.loginfo("Distance to object: %f", self.test_distance)
            rospy.loginfo("X: %f", self.test_X)
            self.object_pub.publish(self.object_info)
            self.rate.sleep()


if __name__ == "__main__":
    try:
    	distance_node = DistanceNode()
    	distance_node.run()
    except rospy.ROSInterruptException:
        pass
