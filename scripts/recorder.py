#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

class Recorder:
    def __init__(self):
        rospy.init_node('recorder', anonymous=True)
        self.sub = rospy.Subscriber('/odom', Odometry, self.callback)
        self.pub = rospy.Publisher('/recorded_points', Point, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz
        self.points = []

    def callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        point = Point(x, y, 0)
        self.points.append(point)
        print(self.points)

    def run(self):
        while not rospy.is_shutdown():
            for point in self.points:
                self.pub.publish(point)
                self.rate.sleep()
                with open('/home/jardeldyonisio/catkin_ws/src/turtlebot3_teleop_recorder/scripts/recorded_points.txt', 'w') as f:
                    for point in self.points:
                        print("here")
                        f.write('{} {}\n'.format(point.x, point.y))
                rospy.loginfo("Recorded points saved to file: recorded_points.txt")

if __name__ == '__main__':
    try:
        recorder = Recorder()
        recorder.run()
    except rospy.ROSInterruptException:
        pass