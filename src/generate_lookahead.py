#!/usr/bin/env python3

import rospy
import tugger
import numpy as np

LOOKAHEAD_ANGLESPAN = np.pi / 2.0
LOOKAHEAD_TOTALPATHS = 100
LOOKAHEAD_DISTBTWPOINTS = 0.3
LOOKAHEAD_POINTSPERPATH = 20
LOOKAHEAD_SIMSTEPS = 10

class lookahead:

    def generate_lookahead(self, angle_span=LOOKAHEAD_ANGLESPAN,
                           total_paths=LOOKAHEAD_TOTALPATHS,
                           dist_btw_points=LOOKAHEAD_DISTBTWPOINTS,
                           points_per_path=LOOKAHEAD_POINTSPERPATH,
                           sim_steps=LOOKAHEAD_SIMSTEPS):

        d = dict()
        train = tugger.Train(0) # Permanece ?
        dt = 1.0 / sim_steps
        for steering_angle in np.linspace(-angle_span/2.0,
                                          angle_span/2.0, total_paths):
            # Reseta posição do trator e ajusta esterçamento fixo
            u = np.array([0.0, 0.0, steering_angle, 0.0]).T
            train.set_state(u)
            d[steering_angle] = list()
            # Para cada ponto
            for _ in range(points_per_path):
                # Grava coordenada atual
                x, y = train.tractor.get_state()
                d[steering_angle].append((x, y))
                # Avança distância definida
                for _ in range(sim_steps):
                    step_dist = dist_btw_points/sim_steps
                    u = np.array([(step_dist / train.tractor.tyre_radius) / dt,
                                  0.0])
                    q = train.get_state()
                    j = train.get_jacobian()
                    q_dot = np.dot(j, u)
                    q += q_dot*dt
                    train.set_state(q)
        self.lookahead_ref = d

if __name__ == '__main__':
    try:
        lookahead = lookahead()
        lookahead.generate_lookahead()
    except rospy.ROSInterruptException:
        pass
