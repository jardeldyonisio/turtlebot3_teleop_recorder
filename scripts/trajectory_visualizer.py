#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

rospy.init_node('trajectory_visualizer')

# Cria o objeto Marker
marker = Marker()
marker.header.frame_id = "odom"
marker.type = marker.POINTS
marker.action = marker.ADD
marker.scale.x = 0.05
marker.scale.y = 0.05
marker.color.r = 1.0
marker.color.a = 1.0

# Adiciona pontos à trajetória
#ponto = Point()
#ponto.x = 0
#ponto.y = 0
#ponto.z = 0
#marker.points.append(ponto)

#ponto = Point()
#ponto.x = 2
#ponto.y = 1
#ponto.z = 0
#marker.points.append(ponto)

with open('recorded_before.txt', 'r') as f:
    for line in f:
        ponto = Point()
        x, y = line.strip().split()
        ponto.x = float(x)
        ponto.y = float(y)
        ponto.z = 0
        marker.points.append(ponto)

# Publica a mensagem Marker
marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

while not rospy.is_shutdown():
    marker.header.stamp = rospy.Time.now()
    marker_publisher.publish(marker)
