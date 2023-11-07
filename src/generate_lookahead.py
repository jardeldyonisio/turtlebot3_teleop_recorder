#!/usr/bin/env python3

import rospy
import numpy as np

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

LOOKAHEAD_ANGLESPAN = np.pi / 2.0
LOOKAHEAD_TOTALPATHS = 100
LOOKAHEAD_DISTBTWPOINTS = 0.3
LOOKAHEAD_POINTSPERPATH = 20
LOOKAHEAD_SIMSTEPS = 10

class lookahead:

    def __init__(self):
        rospy.init_node('lookahead_generator_node')
        self.pub = rospy.Publisher('lookahead_curves', Float32MultiArray, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry, self.callback)
        self.marker_pub = rospy.Publisher('lookahead_curves_markers', Marker, queue_size=10)
        self.rate = rospy.Rate(50) # 10hz
        self.points = []

    def callback(self, msg : Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.point = Point(x, y, 0)

    def get_state(self):
        '''
        Retorna o vetor de estados q
        '''
        q = [self.tractor.x, self.tractor.y,
                self.tractor.steer_angle, self.tractor.angle]
        for cart in self.train[1:]:
            q += [cart.angle+cart.steer_angle, cart.angle]
        q = np.array(q).T
        return q

    def generate_lookahead(self, angle_span=LOOKAHEAD_ANGLESPAN,
                           total_paths=LOOKAHEAD_TOTALPATHS,
                           dist_btw_points=LOOKAHEAD_DISTBTWPOINTS,
                           points_per_path=LOOKAHEAD_POINTSPERPATH,
                           sim_steps=LOOKAHEAD_SIMSTEPS):

        d = dict()
        tyre_radius = 0.2
        dt = 1.0 / sim_steps
        for steering_angle in np.linspace(-angle_span/2.0,
                                          angle_span/2.0, total_paths):
            # Reseta posição do trator e ajusta esterçamento fixo
            u = np.array([0.0, 0.0, steering_angle, 0.0]).T
            self.set_state(u)
            d[steering_angle] = list()
            # Para cada ponto
            for _ in range(points_per_path):
                # Grava coordenada atual
                x, y = self.point
                d[steering_angle].append((x, y))
                # Avança distância definida
                for _ in range(sim_steps):
                    step_dist = dist_btw_points/sim_steps
                    u = np.array([(step_dist / tyre_radius) / dt,
                                  0.0])
                    q = self.point
                    j = train.get_jacobian()
                    q_dot = np.dot(j, u)
                    q += q_dot*dt
                    self.set_state(q)
        self.lookahead_ref = d

if __name__ == '__main__':
    try:
        lookahead = lookahead()
        lookahead.generate_lookahead()
    except rospy.ROSInterruptException:
        pass
