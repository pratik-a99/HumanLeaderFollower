#!/usr/bin/env python

__author__ = "Pratik Acharya, Sumedh Koppula, and Orlandis Smith"
__copyright__ = "Copyright 2022"
__license__ = "MIT"


from wsgiref.util import is_hop_by_hop
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import darknet_ros_msgs
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox, ObjectCount
from sensor_msgs.msg import Image
from apriltag_ros.msg import AprilTagDetectionArray
import numpy as np
import message_filters
import sys
import os
from std_msgs.msg import Int32MultiArray
import time


# class for checking the centers of the april tags, bounding boxes and controlling the robot
class CheckCenters:
    def __init__(self):
        self.bridge = CvBridge()
        self.markerCenterSub = rospy.Subscriber(
            "/tag_center", Int32MultiArray, self.checkMarkerCenter)
        self.yoloCenterSub = rospy.Subscriber(
            "/darknet_ros/bounding_boxes", BoundingBoxes, self.checkYoloCenter)
        self.aprilDetection = rospy.Subscriber(
            '/tag_detections', AprilTagDetectionArray, self.driveCallback)
        self.publish = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.image_sub = rospy.Subscriber(
            '/tag_detections_image', Image, self.camera_callback)
        self.isHuman = False
        self.yoloXCenterMax = []
        self.yoloXCenterMin = []
        self.yoloYCenterMax = []
        self.yoloYCenterMin = []
        self.id = []
        self.markerXCenter = 0
        self.markerYCenter = 0
        rospy.spin()

    def checkYoloCenter(self, boundbox_msg):
        for i in range(len(boundbox_msg.bounding_boxes)):
            if boundbox_msg.bounding_boxes[i].Class == 'person':
                oj = boundbox_msg.bounding_boxes[i]
                self.yoloXCenterMax.append(oj.xmax)
                self.yoloXCenterMin.append(oj.xmin)
                self.yoloYCenterMax.append(oj.ymax)
                self.yoloYCenterMin.append(oj.ymin)
                self.id.append(oj.id)
                rospy.loginfo("Human detected")

    def checkMarkerCenter(self, marker_msg):
        print(len(marker_msg.data))
        if len(marker_msg.data) > 0:
            self.markerXCenter = marker_msg.data[0]
            self.markerYCenter = marker_msg.data[1]
            for i in range(len(self.yoloXCenterMin)):
                if (self.yoloXCenterMin[i] < self.markerXCenter < self.yoloXCenterMax[i]) and (self.yoloYCenterMin[i] < self.markerYCenter < self.yoloYCenterMax[i]):
                    print('here')
                    string = "It is inside the bounding box, the id is: " + \
                        str(self.id[i])
                    rospy.loginfo(string)
                    self.isHuman = True
            self.yoloXCenterMin = []
            self.yoloXCenterMax = []
            self.yoloYCenterMin = []
            self.yoloYCenterMax = []
            self.id = []
        else:
            self.isHuman = False
            rospy.loginfo("No tag")

    def driveCallback(self, data):
        if self.isHuman:
            x_diff = 0
            depth_diff = 0
            try:
                global april_tag_pub

                if april_tag_pub == 0:
                    now = time.time()
                    diff = 0
                    while diff < 5:
                        current = time.time()
                        diff = current - now
                    april_tag_pub = 1
                if len(data.detections) > 0:
                    # compensate the width to determine the right x difference
                    x_diff = data.detections[0].pose.pose.pose.position.x
                # the z position represents the depth from the camera cooridnates
                    depth_diff = data.detections[0].pose.pose.pose.position.z
                else:
                    rospy.loginfo("No tag")
                vel_msg = Twist()

                # the proportianal variables values
                Kp_depth_diff = 0.15  # the proportional value
                Kp_x_diff = 0.5  # the proportional value
                distance_threshold = 1

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
                    self.publish.publish(vel_msg)
                    rospy.loginfo('velocity published')

            except IndexError:
                rospy.loginfo('Can not detect the tags')
        else:
            rospy.loginfo("No human detected ::: Cannot follow")

    def camera_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        cv_image = cv2.resize(cv_image, (640, 480))
        cv2.imshow("Scan", cv_image)
        cv2.waitKey(1)


def main():
    global april_tag_pub
    april_tag_pub = 0
    rospy.init_node('center_checking_node', anonymous=True)

    try:
        chk_ctr = CheckCenters()
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        print("exception")
        pass


if __name__ == '__main__':
    april_tag_pub = 0
    while True:
        main()
