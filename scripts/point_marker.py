#!/usr/bin/env python3

import rospy
import math
from visualization_msgs.msg import Marker

class PointMarker:
    def __init__(self):
        rospy.init_node('point_marker')
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.scale.x = 100.1
        self.marker.scale.y = 100.1
        self.marker.scale.z = 100.1
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 1.0

    def run(self):
        with open('/home/jardeldyonisio/catkin_ws/src/turtlebot3_teleop_recorder/scripts/recorded_points.txt', 'r') as f:
            for line in f.readlines():
                try:
                    x, y = line.split()
                    print(x)
                except ValueError:
                    print("Linha inválida: ", line)
                    continue
                self.marker.pose.position.x = float(x)
                print(self.marker.pose.position.x)
                self.marker.pose.position.y = float(y)
                self.marker.pose.position.z = 0
                self.marker.header.stamp = rospy.Time.now()
                self.marker.id += 1
                self.marker_pub.publish(self.marker)
                rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        point_marker = PointMarker()
        point_marker.run()
    except rospy.ROSInterruptException:
        pass

