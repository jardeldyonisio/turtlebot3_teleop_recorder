#!/usr/bin/env python3

import rospy
import bezier
import numpy as np

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

def draw_bezier_points(num_points, rate, step):
    # Inicialize o nó ROS
    rospy.init_node('draw_bezier_point', anonymous=True)
    
    # Configure o publisher para os markers
    marker_pub = rospy.Publisher('bezier_point_marker', Marker, queue_size=10)

    # Dados da curva de Bezier
    ctrl_pts = np.array([[-1.72690252, -1.46055208, -1.18822679, -0.95691139, -0.7087726, -0.5665215, -0.29682866, -0.1874317, -0.0664586, 0.04932819, 0.14167911, 0.24203602, 0.32158605, 0.38002748, 0.43653219, 0.46268872, 0.51848219, 0.40130823, 0.39856247, 0.39440755, 0.50973404, 0.49065901, 0.47722962, 0.35531731, 0.36700556, 0.37247529, 0.4007504, 0.45062568, 0.49150756, 0.55421677, 0.60943489, 0.78298443, 0.98122471, 1.16497901, 1.26029868, 1.37191027, 1.42801164, 1.50436401, 1.42977518, 1.41809855, 1.4007786, 1.42773201, 1.28265079, 1.25911546, 1.21435599, 1.19367788, 1.17671975, 1.25225169, 1.22633365, 1.03185622, 0.9664065, 0.82064844, 0.62250354, 0.40804228, 0.18311442, 0.07617435, -0.05091977, -0.16199967],
                         [-0.60320651, -0.62260841, -0.69106883, -0.82453109, -0.96769995, -1.27829224, -1.37491497, -1.41410857, -1.42612812, -1.41626638, -1.40840071, -1.40734423, -1.35978003, -1.32483699, -1.27493852, -1.21207157, -1.07797256, -0.92630468, -0.78108789, -0.56134365, -0.34715512, -0.12820093, 0.02594934, 0.16493713, 0.31922918, 0.391433, 0.46722684, 0.51972223, 0.56275177, 0.57848699, 0.60025459, 0.66866977, 0.70002962, 0.66786697, 0.6511831, 0.62386616, 0.5450193, 0.43771095, 0.28122597, 0.15004509, -0.04453553, -0.28932078, -0.42013758, -0.44135891, -0.42962179, -0.45363579, -0.47332969, -0.52635361, -0.52443446, -0.51003401, -0.23246536, -0.10291448, 0.07319822, 0.23222491, 0.37252653, 0.43923176, 0.46746287, 0.52701975]])

    # Configuração da curva de Bezier
    curve = bezier.Curve(ctrl_pts, degree=57)
    t_values = np.arange(0.0, 1.0, step)
    points = curve.evaluate_multi(t_values).T 
    new_zero = 0

    rate = rospy.Rate(rate)

    # Configura o Marker
    marker = Marker()
    marker.header.frame_id = "odom"
    marker.type = Marker.POINTS
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    
    for t in t_values:
        # Adiciona o ponto selecionado ao Marker
        for p in points[new_zero:num_points]:
            selected_point = Point()
            selected_point.x, selected_point.y = p[0], p[1]
            marker.points.append(selected_point)

            if len(marker.points) > num_points:
                marker.points.pop(0)
                num_points += 1
                new_zero += 1

        # Publica o Marker
        if len(marker.points) == num_points:
            # Publica o Marker
            marker_pub.publish(marker)
            print("len(marker.points) :", len(marker.points))
        elif len(marker.points) > num_points:
            # Remove todos os pontos exceto os últimos 5
            print("marker.points before: ", marker.points)
            marker.points = marker.points[-5:]
            print("marker.points after: ", marker.points)
            marker_pub.publish(marker)
            asdad

        rate.sleep()

if __name__ == '__main__':
    try:
        draw_bezier_points(num_points = 5, rate = 1, step = 0.01)
    except rospy.ROSInterruptException:
        pass
