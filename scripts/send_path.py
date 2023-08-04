#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path

class PathControlNode:
    def __init__(self):
        rospy.init_node('path_control_node', anonymous=True)
        self.path_pub = rospy.Publisher('/move_base/NavfnROS/plan', Path, queue_size=10)
        self.path_sub = rospy.Subscriber('/path_to_follow', Path, self.path_callback)
        self.rate = rospy.Rate(10)  # 10 Hz

    def path_callback(self, path_msg):
        if len(path_msg.poses) > 0:
            goal_pose = path_msg.poses[0].pose
            self.move_turtlebot_to_goal(goal_pose)

    def move_turtlebot_to_goal(self, goal_pose):
        cmd_vel = Twist()
        self.path_pub.publish(cmd_vel)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = PathControlNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
