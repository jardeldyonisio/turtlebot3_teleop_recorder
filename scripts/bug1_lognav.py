#!/usr/bin/env python3

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
        self.goals = np.array([[0.7, -1.5], [1.5, 2.0], [-0.3, -0.0]])

    def odom_callback(self, data: Odometry):
        quaternion = data.pose.pose.orientation 
        quaternion_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        _, _, self.yaw = euler_from_quaternion(quaternion_list)

        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

        self.pos = np.array([self.x, self.y])

        goal = self.goals[0]  # Obtenha o próximo objetivo da lista
        goal_vec = np.array(goal)
        self.dist_vec = goal_vec - self.pos
        self.dist = np.linalg.norm(self.dist_vec)

        self.yaw_d = np.arctan2(self.dist_vec[1], self.dist_vec[0])
        self.diff_yaw = self.yaw_d - self.yaw

        msg = Twist()

        print("Goals", self.goals)
        
        # Se não houver metas definidas, não faça nada
        if len(self.goals) == 0:
            msg.angular.z = 0.0
            msg.linear.x = 0.0
            return

        if self.diff_yaw > self.THRESHOLD_YAW:
            # rospy.loginfo("Rotacionando")
            msg.linear.x = .0
            msg.angular.z = .1
        elif self.diff_yaw < -self.THRESHOLD_YAW:
            # rospy.loginfo("Rotacionando")
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
            rospy.loginfo(self.goals) # Remova o objetivo atual da lista
            if len(self.goals) == 0:
                rospy.loginfo("Objetivos concluídos")
                msg.angular.z = .0
                msg.linear.x = .0
        
        self.cmd_vel_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('bug1')
    Navigator()
    rospy.spin()