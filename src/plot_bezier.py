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
    ctrl_pts = np.array([[-1.72690252, -1.46055208, -1.18822679, -0.95691139, -0.7087726, -0.5665215, -0.29682866, -0.1874317, -0.0664586, 0.04932819, 0.14167911, 0.24203602, 0.32158605, 0.38002748, 0.43653219, 0.46268872, 0.51848219, 0.40130823, 0.39856247, 0.39440755, 0.50973404, 0.49065901, 0.47722962, 0.35531731, 0.36700556, 0.37247529, 0.4007504, 0.45062568, 0.49150756, 0.55421677, 0.60943489, 0.78298443, 0.98122471, 1.16497901, 1.26029868, 1.37191027, 1.42801164, 1.50436401, 1.42977518, 1.41809855, 1.4007786, 1.42773201, 1.28265079, 1.25911546, 1.21435599, 1.19367788, 1.17671975, 1.25225169, 1.22633365, 1.03185622, 0.9664065, 0.82064844, 0.62250354, 0.40804228, 0.18311442, 0.07617435, -0.05091977, -0.16199967],
                         [-0.60320651, -0.62260841, -0.69106883, -0.82453109, -0.96769995, -1.27829224, -1.37491497, -1.41410857, -1.42612812, -1.41626638, -1.40840071, -1.40734423, -1.35978003, -1.32483699, -1.27493852, -1.21207157, -1.07797256, -0.92630468, -0.78108789, -0.56134365, -0.34715512, -0.12820093, 0.02594934, 0.16493713, 0.31922918, 0.391433, 0.46722684, 0.51972223, 0.56275177, 0.57848699, 0.60025459, 0.66866977, 0.70002962, 0.66786697, 0.6511831, 0.62386616, 0.5450193, 0.43771095, 0.28122597, 0.15004509, -0.04453553, -0.28932078, -0.42013758, -0.44135891, -0.42962179, -0.45363579, -0.47332969, -0.52635361, -0.52443446, -0.51003401, -0.23246536, -0.10291448, 0.07319822, 0.23222491, 0.37252653, 0.43923176, 0.46746287, 0.52701975]])

    # Configuração da curva de Bezier
    curve = bezier.Curve(ctrl_pts, degree=57)
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

