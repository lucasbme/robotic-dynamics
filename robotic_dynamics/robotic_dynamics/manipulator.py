# Autores: Lucas Bosso de Mello
# Descrição: Classe com atributos e métodos atrelados ao manipulador
# Data: 20/08/2025
# Última modificação por Lucas Bosso em 05/09/2025

# ========== Inclusão de Bibliotecas ==========

import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH
from robotic_dynamics.joy import Joystick

# ========== Implementação com o ToolBox ==========

class Manipulator(DHRobot):

    # ========== Cinemática Direta (DH) ==========
    def __init__(self, params):
        links = [
            RevoluteDH(a = 0, alpha = -np.pi / 2, d = 0),
            RevoluteDH(a = 0, alpha =  np.pi / 2, d = params["d2"]),
            PrismaticDH(a = 0, alpha =  0, theta = 0, qlim = [0, 0.5]),
            RevoluteDH(a = 0, alpha = -np.pi / 2, d = params["d4"]),
            RevoluteDH(a = 0, alpha =  np.pi / 2, d = 0),
        ]

        super().__init__(links, name = "5DOF")

        # ========== Posição Inicial das Juntas ==========
        self.q = np.zeros(self.n)

    # ========== Posição das Juntas (Joystick)  ==========
    def joystick(self, joyCmd, gainRot = 0.02, gainLin = 0.01):
        steps = np.zeros(self.n)
        steps[0] = joyCmd.angular.z * gainRot   # Rotacional
        steps[1] = joyCmd.linear.x  * gainRot   # Rotacional
        steps[2] = joyCmd.linear.z  * gainLin   # Prismático
        steps[3] = joyCmd.linear.y  * gainRot   # Rotacional

        # ========== Incrementa juntas ========== 
        self.q += steps

        # ========== Limita prismático ========== 
        self.q[2] = np.clip(self.q[2], 0, 0.5)

        return self.q

    # ========== Atualiza Posição das Juntas ========== 
    def jointPosition(self, q, steps):
        qNew = q + steps
        qNew[2] = np.clip(qNew[2], 0, 0.5)

        return qNew

# ========== Implementação direta ==========

# class Manipulator:
#     def dh_notation(self, a, alpha, d, theta):
#         ca = np.cos(alpha)
#         ct = np.cos(theta)
#         sa = np.sin(alpha)
#         st = np.sin(theta)

#         T = np.array([
#             [ct, -st * ca,  st * sa, a * ct],
#             [st,  ct * ca, -ct * sa, a * st],
#             [0.0,     sa,      ca,     d],
#             [0.0,    0.0,     0.0,   1.0]
#         ])

#         return T
    
#     def fk(self, q, params):
#         q1, q2, q3, q4, q5 = q

#         alpha1, alpha2, alpha3 = -np.pi/2, np.pi/2, 0.0
#         alpha4, alpha5         = -np.pi/2, np.pi/2, 0.0

#         A1 = self.dh_notation(0, alpha1, 0,     q1)
#         A2 = self.dh_notation(0, alpha2, params["d2"], q2)
#         A3 = self.dh_notation(0, alpha3, q3,    0)
#         A4 = self.dh_notation(0, alpha4, params["d4"], q4)
#         A5 = self.dh_notation(0, alpha5, 0,     q5)

#         Ts = [None]*5
#         Ts[0] = A1
#         Ts[1] = Ts[0] @ A2
#         Ts[2] = Ts[1] @ A3
#         Ts[3] = Ts[2] @ A4
#         Ts[4] = Ts[3] @ A5

#         Tf = Ts[4]
#         return Tf, Ts