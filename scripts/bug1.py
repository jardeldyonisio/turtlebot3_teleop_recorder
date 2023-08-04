#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Navigator:

    def __init__(self):    
        self.scan_topic_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def obstacle_left(self, data):
        scan_state = data
        if (min(data[:20]) < 0.4):
            rospy.loginfo("Obstaculo esquerda")
            return True

    def obstacle_right(self, data):
        scan_state = data
        if (min(data[-20:]) < 0.4):
            rospy.loginfo("Obstaculo direita")
            return True

    def scan_callback(self, data : LaserScan):
        left = self.obstacle_left(data.ranges)
        right = self.obstacle_right(data.ranges)
        msg = Twist()
        if left or right:
            msg.angular.z = -.1
        else:
            msg.angular.z = .0
            
        self.cmd_vel_pub.publish(msg)
            

if __name__ == '__main__':
    rospy.init_node('bug1')
    Navigator()
    rospy.spin()