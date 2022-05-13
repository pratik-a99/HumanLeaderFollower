#!/usr/bin/env python

__author__ = "Pratik Acharya, Sumedh Koppula, and Orlandis Smith"
__copyright__ = "Copyright 2022"
__license__ = "MIT"

import rospy
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


# class for checking the centers of the april tags and controlling the robot
class CheckCenters:
    def __init__(self):
        self.aprilDetector = rospy.Subscriber(
            '/tag_detections', AprilTagDetectionArray, self.driveCallback)
        self.publish = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    def driveCallback(self, data):
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


# main function
def main():
    global april_tag_pub
    april_tag_pub = 0
    rospy.init_node('tag_follower', anonymous=True)

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
