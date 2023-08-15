#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

class Recorder:

    def __init__(self):
        rospy.init_node('recorder', anonymous=True)
        self.sub = rospy.Subscriber('/odom', Odometry, self.callback)
        self.rate = rospy.Rate(50) # 10hz
        self.points = []
        self.file_path = '/home/jardeldyonisio/lognav_ws/src/turtlebot3_teleop_recorder/data/recorded_after.txt'

    def callback(self, msg : Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        point = Point(x, y, 0)
        self.points.append(point)

    def run(self):
        while not rospy.is_shutdown():
            with open(self.file_path, 'a') as f:
                while len(self.points) > 0:
                    point = self.points.pop(0)
                    f.write('{},{}\n'.format(point.x, point.y))
            rospy.loginfo("Points saved")
            self.rate.sleep()

if __name__ == '__main__':
    try:
        recorder = Recorder()
        recorder.run()
    except rospy.ROSInterruptException:
        pass
