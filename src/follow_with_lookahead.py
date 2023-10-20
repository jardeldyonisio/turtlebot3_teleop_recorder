#!/usr/bin/env python3

import rospy
import numpy as np

from bzpath import BzPath
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class TrainController:
    def __init__(self):
        # Inicializa o nó ROS
        rospy.init_node('lookahead_follow', anonymous=True)
        self.sub = rospy.Subscriber('/odom', Odometry, self.callback)

        # Cria um publisher para enviar comandos de controle para o comboio
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Define as constantes de controle
        self.KP_STEERING = 0.5
        self.KP_WHEEL = 0.2
        self.LOOKAHEAD_DISTBTWPOINTS = 0.3
        self.LOOKAHEAD_POINTSPERPATH = 20
        self.FOLLOW_STEP_LENGTH = 0.01
        
        # Originalmente na simulação começa como False e é seta pra True 
        # quando clica na tecla para seguir, devo manter sem tecla e ser
        # direto ou coloco o condional novamente?
        self.pd_controller = False 

        self.bzcurves = BzPath()

    def callback(self, msg : Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def lookahead_follow(self):
        self.lookahead = dict()
        for steer_angle, points in self.lookahead_ref.items():
            points = np.array(points)
            d = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
            angle = np.arctan2(points[:, 1], points[:, 0])
            xs = np.array(self.x +
                          d * np.cos(angle + tractor_angle)).reshape(-1, 1)
            ys = np.array(self.y +
                          d * np.sin(angle + tractor_angle)).reshape(-1, 1)
            points = np.concatenate((xs, ys), axis=1)
            self.lookahead[steer_angle] = points

        self.curr_dist += self.FOLLOW_STEP_LENGTH
        self.path_points = \
            self.bzcurves.getNextCoords(self.curr_dist,
                                        self.LOOKAHEAD_POINTSPERPATH,
                                        self.LOOKAHEAD_DISTBTWPOINTS)
        if self.curr_dist > self.bzcurves.getLength() \
            - self.LOOKAHEAD_POINTSPERPATH*self.LOOKAHEAD_DISTBTWPOINTS:
            self.path_points = []
            self.curr_dist = 0.0

        min_cost = None
        desired_angle = 0.0
        for angle, path in self.lookahead.items():
            cost = self.compare_paths(path, self.path_points)
            if min_cost is None or cost < min_cost:
                min_cost = cost
                desired_angle = angle
                self.min_path = path

        # Lê o vetor de estados atual do comboio
        q = self.train.get_state()

        # Calcula o Jacobiano atual do comboio
        j = self.train.get_jacobian()

        # Aplica controle PD para o ponto clicado na tela
        if self.pd_controller:
            gy, gx = self.goal[0]
            self.u = self.train.get_control_for_goal(gy, gx, self.dt)
        # Aplica controle PD para seguir trajetória
        elif len(self.path_points) == self.LOOKAHEAD_POINTSPERPATH:
            x1 = self.path_points[0][0][0]
            y1 = self.path_points[0][1][0]
            x2, y2 = self.lookahead[desired_angle][0]
            dx = x2 - x1
            dy = y2 - y1
            dist_err = np.sqrt(dx**2 + dy**2)
            current_angle = q[2]
            steer_err = desired_angle - current_angle
            steer_vel = self.KP_STEERING * steer_err
            wheel_vel = self.KP_WHEEL * dist_err
            self.u = np.array([wheel_vel, steer_vel])

        q_dot = np.dot(j, self.u)
        q += q_dot * self.dt
        self.train.set_state(q)

        # Para facilitar dirigibilidade, depois de
        # atualizar, reseta a velocidade do angulo
        # de esterçamento.
        if not self.pd_controller:
            self.u[1] = 0.0

    def compare_paths(self, path1, path2):
        '''
        Compare the distance between the respective
        points of two trajectories, in the same
        order as they appear in the list.
        '''
        total_cost = 0.0
        weight = self.LOOKAHEAD_POINTSPERPATH
        for point1, point2 in zip(path1, path2):
            x1, y1 = point1
            x2, y2 = point2
            dx = x2 - x1
            dy = y2 - y1
            cost = dx**2 + dy**2
            total_cost += cost*weight
            weight -= 1.0
        return total_cost

if __name__ == '__main__':
    try:
        controller = TrainController()
        controller.lookahead_follow()
    except rospy.ROSInterruptException:
        pass
