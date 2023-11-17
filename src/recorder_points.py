#!/usr/bin/env python3

import rclpy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

class Recorder:

    def __init__(self):
        self.node = rclpy.create_node('recorder')
        self.sub = self.node.create_subscription(Odometry, '/odom', self.callback, 10)
        self.points = []
        self.file_path = '/home/jardeldyonisio/lognav_ws/src/turtlebot3_teleop_recorder/data/recorded_before.txt'

    def callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        point = Point(x, y, 0)
        self.points.append(point)

    def run(self):
        while rclpy.ok():
            with open(self.file_path, 'a') as f:
                while len(self.points) > 0:
                    point = self.points.pop(0)
                    f.write('{},{}\n'.format(point.x, point.y))
            self.node.get_logger().info("Points saved")

def main(args=None):
    rclpy.init(args=args)

    try:
        recorder = Recorder()
        recorder.run()
    finally:
        recorder.node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
