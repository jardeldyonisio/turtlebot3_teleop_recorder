def run(self):
    rospy.loginfo("Recorder node is running...")

    rate = rospy.Rate(self.frequency)
    while not rospy.is_shutdown():
        rate.sleep()

    # Save recorded points to a file
    with open('recorded_points.txt', 'w') as f:
        for point in self.points:
            f.write('{} {}\n'.format(point.x, point.y))
        rospy.loginfo("Recorded points saved to file: recorded_points.txt")
