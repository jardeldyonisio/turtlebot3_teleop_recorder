#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PathPlanner:
    def __init__(self):
        rospy.init_node('path_planner', anonymous=True)
        self.pub = rospy.Publisher('/move_base/NavfnROS/plan', Path, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz

    def read_points_from_file(self, file_path):
        points = []
        with open(file_path, 'r') as f:
            for line in f:
                x, y = line.strip().split()
                point = PoseStamped()
                point.header.frame_id = "map"
                point.pose.position.x = float(x)
                point.pose.position.y = float(y)
                point.pose.position.z = 0.0
                point.pose.orientation.x = 0.0
                point.pose.orientation.y = 0.0
                point.pose.orientation.z = 0.0
                point.pose.orientation.w = 1.0
                points.append(point)
        return points

    def run(self):
        points = self.read_points_from_file('/home/jardeldyonisio/lognav_ws/src/turtlebot3_teleop_recorder/scripts/recorded_points.txt')

        path = Path()
        path.header.frame_id = "map"
        path.poses = points
        print(path.poses)

        while not rospy.is_shutdown():
            self.pub.publish(path)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        path_planner = PathPlanner()
        path_planner.run()
    except rospy.ROSInterruptException:
        pass
