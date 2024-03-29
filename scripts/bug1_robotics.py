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
    THRESHOLD_DIST = .4

    def __init__(self):    
        self.scan_topic_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.position_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=10)
        self.goals = np.array([[1.4, -0.1], [1.8, -2.0], [0.4, -3.0],[0.7, -0.9], [0.8, 0.0], [-0.6, -0.6], [-0.7, -1.9], [-0.9, -3.0], [-1.7, -2.0], [-1.8, 0.0]])
        self.parado_desviando = None

    def obstacle_left_stop(self, data):
        if (min(data[:40]) < 0.2):
            return True

    def obstacle_right_stop(self, data):
        if (min(data[-40:]) < 0.2):
            return True
        
    def obstacle_left_move(self, data):
        if (min(data[:15]) > 0.2 and min(data[:15]) < 0.5):
            return True
        
    def obstacle_right_move(self, data):
        if (min(data[-10:]) < 0.2 and min(data[-10:]) < 0.5):
            return True
        
    def obstacle_left_side(self, data):
        if (min(data[55:115]) < 0.2):
            return True
        
    def obstacle_right_side(self, data):
        if (min(data[-115:-55]) < 0.2):
            return True
        
    def scan_callback(self, data : LaserScan):
        left_stop = self.obstacle_left_stop(data.ranges)
        right_stop = self.obstacle_right_stop(data.ranges)
        left_move = self.obstacle_left_move(data.ranges)
        right_move = self.obstacle_right_move(data.ranges)
        left_side = self.obstacle_left_side(data.ranges)
        right_side = self.obstacle_right_side(data.ranges)
        msg = Twist()

        rospy.loginfo("Goals %s", self.goals)

        if len(self.goals) == 0:
            msg.angular.z = .0
            msg.linear.x = .0
            return
        
        if left_stop:
            rospy.loginfo("Parado e desviando para a esquerda")
            msg.linear.x = .0
            msg.angular.z = .1
        elif right_stop:
            rospy.loginfo("Parado e desviando para a direita")
            msg.linear.x = .0
            msg.angular.z = .1
        else:
            msg.angular.z = .0
            msg.linear.x = .1
            if left_side or right_side:
                rospy.loginfo("Obstaculo no lado")
                msg.linear.x = .1
                msg.angular.z = .0
            elif left_move:
                rospy.loginfo("Andando e desviando para a esquerda")
                msg.linear.x = .1
                msg.angular.z = .1
            elif right_move:
                rospy.loginfo("Andando e desviando para a direita")
                msg.linear.x = .1
                msg.angular.z = -.1
            elif self.diff_yaw > self.THRESHOLD_YAW:
                rospy.loginfo("Rotacionando")
                msg.linear.x = .0
                msg.angular.z = .1
            elif self.diff_yaw < -self.THRESHOLD_YAW:
                rospy.loginfo("Rotacionando")
                msg.linear.x = .0
                msg.angular.z = -.1
            else:
                msg.angular.z = .0
                msg.linear.x = .1

        if self.dist < self.THRESHOLD_DIST:
            rospy.loginfo("Atingiu o objetivo")
            msg.angular.z = .0
            msg.linear.x = .0
            self.goals = np.delete(self.goals, 0, 0)
            self.cmd_vel_pub.publish(msg)
            time.sleep(3)
            if len(self.goals) == 0:
                rospy.loginfo("Objetivos concluídos")
                msg.angular.z = .0
                msg.linear.x = .0
                return
        self.cmd_vel_pub.publish(msg)


    def odom_callback(self, data : Odometry):
        msg = Twist()

        quaternion = data.pose.pose.orientation 
        quaternion_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        _, _, self.yaw = euler_from_quaternion(quaternion_list)

        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

        self.pos = np.array([self.x, self.y])
        if len(self.goals) == 0:
            rospy.loginfo("Objetivos concluídos")
            msg.angular.z = .0
            msg.linear.x = .0
            return

        goal = self.goals[0]
        goal_vec = np.array(goal)
        self.dist_vec = goal_vec - self.pos
        self.dist = np.linalg.norm(self.dist_vec)

        self.yaw_d = np.arctan2(self.dist_vec[1], self.dist_vec[0])
        self.diff_yaw = self.yaw_d - self.yaw

if __name__ == '__main__':
    rospy.init_node('bug1')
    Navigator()
    rospy.spin()