#!/usr/bin/env python3

from torch import le
import rospy
import time
import cv2
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from apriltag_ros.msg import AprilTagDetectionArray
# from move_robot import MoveTurtlebot3
import numpy as np
from std_msgs.msg import Int16, Int32MultiArray

x_diff = 0
depth_diff = 0


class Apriltag_follower(object):

    def __init__(self):
        self.bridge = CvBridge()
        # to use the CvBridge, you will need to transfer the message from compressed to raw (follow the instructions)

        self.publish = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.image_sub = rospy.Subscriber(
            '/tag_detections_image', Image, self.camera_callback)
        self.sub = rospy.Subscriber(
            '/tag_detections', AprilTagDetectionArray, self.callback)
        self.stop_sign_detect = rospy.Subscriber(
            "/detect_stop", Int16, self.stop_detection)
        self.markerCenterSub = rospy.Subscriber(
            "/tag_center", Int32MultiArray, self.checkMarkerCenter)
        self.yoloCenterSub = rospy.Subscriber(
            "/darknet_ros/bounding_boxes", BoundingBoxes, self.checkYoloCenter)

        self.yoloXCenterMax = []
        self.yoloXCenterMin = []
        self.yoloYCenterMax = []
        self.yoloYCenterMin = []
        self.id = []
        self.markerXCenter = 0
        self.markerYCenter = 0
        global follow
        follow = False
        # rospy.spin()

    def stop_detection(self, msg):
        global stop_detected
        stop_detected = msg.data

    def camera_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        cv2.imshow("Scan", cv_image)
        cv2.waitKey(1)

    def callback(self, data):
        global x_diff, depth_diff, follow
        if follow:
            try:
                global april_tag_pub
                global stop_detected

                if stop_detected == 1 and april_tag_pub == 0:
                    now = time.time()
                    diff = 0
                    while diff < 5:
                        current = time.time()
                        diff = current - now
                    april_tag_pub = 1

                # compensate the width to determine the right x difference
                x_diff = data.detections[0].pose.pose.pose.position.x
                # the z position represents the depth from the camera cooridnates
                depth_diff = data.detections[0].pose.pose.pose.position.z
                vel_msg = Twist()

                # the proportianal variables values
                Kp_depth_diff = 0.15  # the proportional value
                Kp_x_diff = 0.5  # the proportional value
                distance_threshold = 0.5

                max_lin_speed = 0.2
                max_ang_speed = 0.2

                vel_msg.angular.z = -1*x_diff * Kp_x_diff

                if ((vel_msg.angular.z < max_ang_speed) and (vel_msg.angular.z > -1*max_ang_speed)):
                    vel_msg.angular.z = vel_msg.angular.z
                elif vel_msg.angular.z > max_ang_speed:
                    vel_msg.angular.z = max_ang_speed
                elif vel_msg.angular.z < -1*max_ang_speed:
                    vel_msg.angular.z = -1*max_ang_speed

                # the velocity message
                # the robot will maintain distance of 10 cm
                if (depth_diff < distance_threshold) and (depth_diff > 0):
                    vel_msg.linear.x = 0
                    vel_msg.angular.z = vel_msg.angular.z
                elif depth_diff > distance_threshold:
                    vel_msg.linear.x = np.minimum(
                        (depth_diff * Kp_depth_diff), max_lin_speed)
                    vel_msg.angular.z = vel_msg.angular.z
                else:
                    vel_msg.linear.x = 0
                    vel_msg.angular.z = vel_msg.angular.z

                if april_tag_pub == 1:
                    print('here')
                    self.publish.publish(vel_msg)
                    print("I am publishing")

            except IndexError:
                rospy.loginfo('Can not detect the tags')

    def checkYoloCenter(self, boundbox_msg):
        for i in range(len(boundbox_msg.bounding_boxes)):
            if boundbox_msg.bounding_boxes[i].Class == 'person':
                oj = boundbox_msg.bounding_boxes[i]
                self.yoloXCenterMax.append(oj.xmax)
                self.yoloXCenterMin.append(oj.xmin)
                self.yoloYCenterMax.append(oj.ymax)
                self.yoloYCenterMin.append(oj.ymin)
                self.id.append(oj.id)

    def checkMarkerCenter(self, marker_msg):
        global follow
        if len(marker_msg.data) > 0:
            self.markerXCenter = marker_msg.data[0]
            self.markerYCenter = marker_msg.data[1]
            for i in range(len(self.yoloXCenterMin)):
                if (self.yoloXCenterMin[i] < self.markerXCenter < self.yoloXCenterMax[i]) and (self.yoloYCenterMin[i] < self.markerYCenter < self.yoloYCenterMax[i]):
                    string = "It is inside the bounding box, the id is: " + \
                        str(self.id[i])
                    rospy.loginfo(string)
                    follow = True
            self.yoloXCenterMin = []
            self.yoloXCenterMax = []
            self.yoloYCenterMin = []
            self.yoloYCenterMax = []
            self.id = []
        else:
            rospy.loginfo("No tag")


def main():
    global stop_detected
    global april_tag_pub
    april_tag_pub = 0
    stop_detected = 0
    rospy.init_node('april_tag_node', anonymous=True)
    Apriltag_follower()
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    april_tag_pub = 0
    while True:
        main()
