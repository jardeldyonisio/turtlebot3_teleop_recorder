#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
Este módulo define classes e funções para simulação cinemática
de um comboio logístico.
'''

import rospy
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

class Cart:
    '''
    Define um carrinho que fará parte de um comboio, a ser
    puxado por um trator.
    '''

    # Dynamic variables
    x_pos = 0.0
    y_pos = 0.0
    angle = 0.0
    steer_angle = 0.0

    # Fixed parameters
    tyre_radius = 0.288
    tyre_width = 0.167
    wheelbase = 1.5 # Não encontrei o que altera
    width = 1.0
    front_drawbar = 1.0 # Não encontrei o que altera
    back_drawbar = 5 # Não encontrei o que altera
    margin = 0.12
    tugger = None
    next_cart = None

    def __init__(self, x=0.0, y=0.0, angle=0.0, steer_angle=0.0):
        '''
        Inicializa variáveis dinâmicas
        '''
        self.x = x
        self.y = y
        self.angle = angle
        self.steer_angle = steer_angle

    def get_tug_coord(self):
        '''
        Retorna as coordenadas do ponto de contato do rebocador
        '''
        tug_x = self.x + self.back_drawbar * np.sin(self.angle)
        tug_y = self.y - self.back_drawbar * np.cos(self.angle)
        return tug_x, tug_y

    def set_tugged_by(self, tugger):
        '''
        Define o objeto que deve puxar este reboque. Você não
        precisa chamar esta função, pois a função abaixo já
        o faz.
        '''
        self.tugger = tugger

    def set_next_cart(self, cart):
        '''
        Define o próximo reboque da cadeia. Esta função
        já chama a acima, reciprocamente.
        '''
        cart.set_tugged_by(self)
        self.next_cart = cart

    def update_tugs(self):
        '''
        Atualiza a posição de todos elementos do comboio
        que vêm após este, chamando a função recursivamente.
        '''
        if self.tugger is not None:
            xt, yt = self.tugger.get_tug_coord()
            self.x = xt + self.front_drawbar \
                * np.sin(self.angle + self.steer_angle) \
                + self.wheelbase * np.sin(self.angle)
            self.y = yt - self.front_drawbar \
                * np.cos(self.angle + self.steer_angle) \
                - self.wheelbase * np.cos(self.angle)
        if self.next_cart is not None:
            self.next_cart.update_tugs()


class Tractor:
    '''
    Define um trator estilo triciclo que puxará o comboio.
    '''
    # Dynamic variables
    x = 0.0
    y = 0.0
    angle = 0.0
    steer_angle = 0.0

    # Fixed parameters
    tyre_radius = 0.288
    tyre_width = 0.167
    wheelbase = 1.2
    width = 1.0
    back_drawbar = 1.0
    next_cart = None
    max_vel = 40.0
    max_steer_vel = 100.0

    def __init__(self, x=0.0, y=0.0, steer_angle=0.0, angle=0.0):
        '''
        Inicializa variáveis dinâmicas
        '''
        self.angle = angle
        self.steer_angle = steer_angle
        self.x = x
        self.y = y

    def set_state(self, x, y, steer_angle, angle):
        '''
        Atualiza variáveis dinâmicas
        '''
        self.angle = angle
        self.steer_angle = steer_angle
        self.x = x
        self.y = y

    def get_state(self):
        return self.x, self.y

    def get_tug_coord(self):
        '''
        Retorna as coordenadas da junta de conexão do reboque
        '''
        tug_x = self.x + self.back_drawbar * np.sin(self.angle)
        tug_y = self.y - self.back_drawbar * np.cos(self.angle)
        return tug_x, tug_y

    def set_next_cart(self, cart):
        '''
        Define o reboque que vai preso ao rebocador.
        '''
        cart.set_tugged_by(self)
        self.next_cart = cart

    def update_tugs(self):
        '''
        Atualiza a posição dos reboques do comboio,
        chamando essa função para cada elemento, de
        forma recursiva.
        '''
        if self.next_cart is not None:
            self.next_cart.update_tugs()


class Train:
    '''
    Comboio completo, com trator e carrinhos de reboque
    '''

    # Número de reboques no comboio
    # (esse número não inclui o trator)
    n = 4

    # Posição do centro do eixo traseiro
    # do trator
    x = 0.0
    y = 0.0

    # Goal
    x_goal = 0.0
    y_goal = 0.0

    # Ganhos do controlador PD
    #kp1 = 0.1
    #kv1 = 0.1
    #kp2 = 1.0
    #kv2 = 0.05
    kp1 = 1000.0
    kv1 = 100.0
    kp2 = 10.0
    kv2 = 0.5

    # Parâmetros dos reboques
    cart_wheelbase = 1.5
    cart_front_drawbar = 1.0
    cart_back_drawbar = 0.5

    tractor = None

    def __init__(self, n=4):
        '''
        Inicializa um comboio com um trem e n reboques,
        total de n+1 elementos.
        '''
        rospy.init_node('train')

        self.sub = rospy.Subscriber('/odom', Odometry, self.callback)

        # Inicializa as coordenadas na
        # posição do trator
        x = self.x # Precisa conferir se precisa mudar
        y = self.y

        # Inicializa os erros para
        # buscar a posição alvo
        self.d_err = 0.0
        self.angle_err = 0.0
        self.d_err_dot = 0.0
        self.angle_err_dot = 0.0

        # Copia n
        self.n = n

        # Nosso comboio será uma lista de objetos
        # começando pelo trator, seguido dos respectivos
        # reboques, em ordem
        self.train = list()

        # Criamos o trator e colocamos na lista
        self.tractor = Tractor(x, y)
        self.train.append(self.tractor)

        # Calcula a próxima posição y para o primeiro
        # reboque
        y -= self.tractor.back_drawbar + \
            self.cart_front_drawbar + self.cart_wheelbase

        # Laço para criar n reboques
        for _ in range(self.n):

            # Cria um reboque
            cart = Cart(x, y)

            # Ajusta os parâmetros do reboque
            cart.wheelbase = self.cart_wheelbase
            cart.front_drawbar = self.cart_front_drawbar
            cart.back_drawbar = self.cart_back_drawbar

            # Conecta o reboque ao elemento da frente
            # que pode ser o trator ou outro reboque
            self.train[-1].set_next_cart(cart)

            # Adiciona esse reboque ao comboio
            self.train.append(cart)

            # Calcula a posição y do próximo reboque
            y -= cart.front_drawbar + cart.wheelbase + cart.back_drawbar

    def callback(self, msg : Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.point = Point(self.x, self.y)

    def update_tugs(self):
        '''
        Atualiza a posição de todos elementos, respeitando
        a posição das conexões. Essa função é chamada de
        forma recursiva de um elemento para o próximo,
        começando aqui pelo trator.
        '''
        if len(self.train) > 0:
            self.train[0].update_tugs()

    def get_state(self):
        '''
        Retorna o vetor de estados q
        '''
        if len(self.train) > 0: # É necessário?
            q = [self.tractor.x, self.tractor.y,
                 self.tractor.steer_angle, self.tractor.angle] # Tem que ajustar como é coletada a posição dos reboques
            for cart in self.train[1:]:
                q += [cart.angle + cart.steer_angle, cart.angle]
        q = np.array(q).T
        return q

    def get_steer_angle(self):
        '''
        Retorna o esterçamento do trator
        '''
        return self.train[0].steer_angle

    def set_state(self, q):
        '''
        Seta o vetor de estados q
        '''
        q = q.T
        # Ajusta o estado do trator
        if len(self.train) > 0: # É necessário?
            tractor = self.train[0]
            x, y, steer_angle, angle = q[:4]
            tractor.set_state(x, y, steer_angle, angle)
            # Ajusta o estado de cada reboque
            for i, cart in enumerate(self.train[1:]):
                cart.angle = q[4 + 2*i + 1]
                cart.steer_angle = q[4 + 2*i] - cart.angle
        # Atualiza posições
        self.update_tugs()

    def get_jacobian(self):
        '''
        Retorna o Jacobiano do sistema. Aqui
        eu segui o artigo https://doi.org/10.2507/IJSIMM20-2-550
        Foram feitos apenas alguns pequenos ajustes.
        '''

        # Iniciamos com uma lista, pois fica mais simples
        # de adicionar elementos. Mais ao final será
        # transformado em array do NumPy.
        J = list()

        # Começamos com o trator do comboio
        # (se ele existe)
        if len(self.train) >= 1: # É necessário?
            # Por conveniencia, dou nomes de variáveis
            # semelhantes aos nomes usados no artigo
            tractor = self.train[0]
            beta_0 = tractor.angle
            alpha_0s = tractor.steer_angle
            h_0 = tractor.wheelbase
            r_0f = tractor.tyre_radius
            d_0 = tractor.back_drawbar

            # Eq 4
            f_l0 = r_0f * np.cos(alpha_0s)

            # Eq 5
            f_a0 = (r_0f / h_0) * np.sin(alpha_0s)

            # Eq 6 (primeiros 4 elementos)
            J += [[-f_l0*np.sin(beta_0), 0.0],
                  [f_l0*np.cos(beta_0), 0.0],
                  [0.0, 1.0],
                  [f_a0, 0.0]]

            # Agora vamos para os reboques
            f_ai2 = None  # Essa variável será definida depois
            f_li2 = None  # Essa variável será definida depois
            for i in range(self.n):
                # Reboque atual é i+1 (pois i=0 é o trator,
                # e n é o número de reboques, ou seja,
                # temos um total de n+1 elementos)
                c_now = self.train[i+1]

                # Esse é o ângulo absoluto do esterçamento
                # desse reboque, no sistema de coordenadas
                # global
                beta_1 = c_now.angle + c_now.steer_angle

                # Comprimento do cambão dianteiro
                d_pi = c_now.front_drawbar

                # Ângulo de esterçamento
                beta_steer = c_now.steer_angle

                # Distância entre os eixos
                h_i = c_now.wheelbase

                # Tratamos separado o primeiro reboque,
                # como sugere o artigo
                if i == 0:
                    # Esse é o ângulo entre o reboque da
                    # frente e o cambão do reboque atual
                    beta_tug = beta_0 - beta_1

                    # Eq 7
                    f_li1 = f_l0*np.cos(beta_tug) + \
                        f_a0*d_0*np.sin(beta_tug)

                    # Eq 8
                    f_ai1 = (f_l0*np.sin(beta_tug) -
                             f_a0*d_0*np.cos(beta_tug)) / d_pi

                # Reboques depois do primeiro...
                else:

                    # Reboque anterior é o i (lembre que o atual
                    # é i+1)
                    c_bef = self.train[i]

                    # Ângulo do reboque anterior
                    beta_2 = c_bef.angle

                    # Ângulo entre o reboque da frente e
                    # o cambão do atual.
                    beta_tug = beta_2 - beta_1

                    # Distância do centro do reboque até o ponto
                    # de conexão na parte de trás
                    d_tug = c_bef.back_drawbar + c_bef.wheelbase/2.0

                    # Eq 9
                    f_li1 = f_li2 * np.cos(beta_tug) + \
                        f_ai2 * d_tug * np.sin(beta_tug)

                    # Eq 10
                    f_ai1 = (f_li2 * np.sin(beta_tug) -
                             f_ai2 * d_tug * np.cos(beta_tug)) / d_pi

                # Atenção, essas variáveis abaixo são calculadas
                # ao final do laço para serem usadas na próxima
                # iteração.

                # Eq 11
                f_li2 = f_li1 * np.cos(beta_steer)

                # Eq 12
                f_ai2 = 2.0 * f_li1 * np.sin(beta_steer) / h_i

                # Adiciona duas linhas à matriz Jacobiano
                J += [[f_ai1, 0],
                      [f_ai2, 0]]

        # Transforma em array do NumPy
        J = np.array(J)

        return J
