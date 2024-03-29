#!/usr/bin/env python3

import rospy
import numpy as np
import time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

class Navigator:
    THRESHOLD_YAW = .1
    THRESHOLD_DIST = .1

    def __init__(self):    
        self.scan_topic_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.position_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=10)
        self.goal = np.array([0.5, -0.5])
        self.obstacle_detected = None

    def obstacle_left(self, data):
        if (min(data[:90]) < 0.4):
            return True

    def obstacle_right(self, data):
        if (min(data[-90:]) < 0.4):
            return True

    def scan_callback(self, data : LaserScan):
        left = self.obstacle_left(data.ranges)
        right = self.obstacle_right(data.ranges)
        msg = Twist()
        if left or right:
            rospy.loginfo("Desviando")
            # self.obstacle_detected = True
            msg.linear.x = .0
            msg.angular.z = -.1
        else:
            rospy.loginfo("Sem obstaculos")
            # self.obstacle_detected = False
            msg.angular.z = .0
            msg.linear.x = .1

            if self.diff_yaw > self.THRESHOLD_YAW:
                rospy.loginfo("Rotacionando")
                msg.linear.x = 0
                msg.angular.z = .1
            elif self.diff_yaw < -self.THRESHOLD_YAW:
                rospy.loginfo("Rotacionando")
                msg.linear.x = 0
                msg.angular.z = -.1
            else:
                msg.linear.x = .1
                msg.angular.z = .0
        self.cmd_vel_pub.publish(msg)

        if self.dist < self.THRESHOLD_DIST:
            rospy.loginfo("Atingiu o objetivo")
            msg.angular.z = .0
            msg.linear.x = .0
            self.cmd_vel_pub.publish(msg)
            return


    def odom_callback(self, data : Odometry):
        quaternion = data.pose.pose.orientation 
        quaternion_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        _, _, self.yaw = euler_from_quaternion(quaternion_list)

        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

        self.pos = np.array([self.x, self.y])
        self.dist_vec = self.goal - self.pos
        self.dist = np.linalg.norm(self.dist_vec)

        self.yaw_d = np.arctan2(self.dist_vec[1], self.dist_vec[0])
        self.diff_yaw = self.yaw_d - self.yaw

        # rospy.loginfo(f"Odom: {self.x:.2f}, {self.y:.2f}, {self.yaw:.2f}")
        # rospy.loginfo(f"Yaw_d: {self.yaw_d:.2f}")

if __name__ == '__main__':
    rospy.init_node('bug1')
    Navigator()
    rospy.spin()