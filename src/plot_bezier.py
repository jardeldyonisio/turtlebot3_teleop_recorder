#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import bezier
import numpy as np

def draw_bezier_curve():
    # Inicialize o nó ROS
    rospy.init_node('draw_bezier_curve', anonymous=True)
    
    # Configure o publisher para os markers
    marker_pub = rospy.Publisher('bezier_curve_marker', Marker, queue_size=10)

    # Dados da curva de Bezier
    ctrl_pts = np.array([[-0.490810, 0.639090, 0.239615, 1.442976, 1.445990,  0.888603],
                         [-0.500489, -2.935461, 1.333089, 0.588855, -1.396138,0.041683]])

    # Configuração da curva de Bezier
    curve = bezier.Curve(ctrl_pts, degree=5)
    vals = np.linspace(0.0, 1.0, num=20)
    points = curve.evaluate_multi(vals).T  # Transpor para ter as coordenadas certas

    # Configure o Marker
    marker = Marker()
    marker.header.frame_id = "odom"  # Substitua pelo seu frame
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.01  # Largura da linha
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    for p in points:
        point = Point()
        point.x, point.y, point.z = p[0], p[1], 0.0
        print("point: ", point)
        marker.points.append(point)

    # Publica o Marker
    while not rospy.is_shutdown():
        marker_pub.publish(marker)
        rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        draw_bezier_curve()
    except rospy.ROSInterruptException:
        pass

