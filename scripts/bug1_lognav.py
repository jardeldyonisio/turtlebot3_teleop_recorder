#!/usr/bin/env python3

import time
import rospy
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion

class Navigator:
    THRESHOLD_YAW = .1
    THRESHOLD_DIST = .1

    def __init__(self):    
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.position_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=10)

        self.read_points_from_file('/home/jardeldyonisio/lognav_ws/src/turtlebot3_teleop_recorder/scripts/recorded_points.txt')

    def read_points_from_file(self, file_path):
        self.goals = np.array([[]])
        with open(file_path, 'r') as f:
            for line in f:
                x, y = line.strip().split()
                point = np.array([[float(x), float(y)]])
                self.goals = np.append(self.goals, point, axis = 0)
        return

    def odom_callback(self, data: Odometry):
        msg = Twist()

        if len(self.goals) == 0:
            msg.angular.z = 0.0
            msg.linear.x = 0.0
            return

        quaternion = data.pose.pose.orientation 
        quaternion_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        _, _, self.yaw = euler_from_quaternion(quaternion_list)

        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

        self.pos = np.array([self.x, self.y])

        goal = self.goals[0]
        goal_vec = np.array(goal)
        self.dist_vec = goal_vec - self.pos
        self.dist = np.linalg.norm(self.dist_vec)

        self.yaw_d = np.arctan2(self.dist_vec[1], self.dist_vec[0])
        self.diff_yaw = self.yaw_d - self.yaw

        print("Goals", self.goals)
    
        if self.diff_yaw > self.THRESHOLD_YAW:
            msg.linear.x = .0
            msg.angular.z = .1
        elif self.diff_yaw < -self.THRESHOLD_YAW:
            msg.linear.x = .0
            msg.angular.z = -.1
        else:
            msg.linear.x = .1
            msg.angular.z = .0

        if self.dist < self.THRESHOLD_DIST:
            rospy.loginfo("Atingiu o objetivo")
            msg.angular.z = .0
            msg.linear.x = .0
            self.goals = np.delete(self.goals, 0, 0)
            self.cmd_vel_pub.publish(msg)
            time.sleep(3)
            rospy.loginfo(self.goals)
            if len(self.goals) == 0:
                rospy.loginfo("Objetivos concluídos")
                msg.angular.z = .0
                msg.linear.x = .0
        
        self.cmd_vel_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('bug1')
    Navigator()
    rospy.spin()