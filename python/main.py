# Autores: Lucas Bosso de Mello, Lucas Maroun de Almeida
# Descrição: Código principal de controle de fluxo
# Data: 05/09/2025
# Última modificação por Lucas Maroun em 23/09/2025

# ========== Inclusão de Bibliotecas ==========

import numpy as np                                      # Matemática
import roboticstoolbox as rtb                           # Toolbox Peter Corke

# from .dynamic import Dynamics
from handler import (
    Kinematics, 
    Jacobian,
    Trajectory,
    Dynamics,
)
    

class Manipulador:
    def __init__(self):
        self.init_manipulator()
        self.q = [0,   # q1
                  0,   # q2
                  0.2,             # q3 (prismática)
                  0,  # q4
                  0]   # q5
        
        self.kinematics = Kinematics(self.robot)
        self.jacobian = Jacobian(self.robot)
        self.trajectory = Trajectory(self.robot)
        self.dynamic = Dynamics(self.robot)
    
    def init_manipulator(self) -> None:
        '''
        
        '''
        self.robot = rtb.DHRobot([
        rtb.RevoluteDH( d = 0.2, a = 0, alpha = -np.pi / 2),
        rtb.RevoluteDH( d = 0.1, a = 0, alpha = np.pi / 2),
        rtb.PrismaticDH(theta = -np.pi/2, a = 0, alpha = 0, offset=0.2, qlim=[0.0, 0.5]),
        rtb.RevoluteDH( d = 0,            a = 0, alpha = np.pi / 2),
        rtb.RevoluteDH( d = 0,            a = 0, alpha = np.pi / 2)
    ])
    
    def main(self):
        '''
        
        '''
        
        # ==== Cinemática Direta ==== #
        T = self.kinematics.calc_forward_kinematics(self.q)
        
        # ==== Cinemática Inversa ==== #
        # Definir posição final (end-effector) desejada
        q_end_effector = [
            -0.3, # X
             0.1, # Y
             0.6, # Z
        ]
        ik_solution =self.kinematics.calc_inverse_kinematics(q_end_effector)
        
        # ==== Jacobiana ==== #
        J = self.jacobian.calc_jacobian(self.q, T)
        
        # ==== Trajetória ==== #
        # self.trajectory.trajectory_joint_space(ik_solution)
        self.trajectory.trajectory_cartesian_space(q_end_effector)
        
        # ==== Dinâmica ==== #
        # self.dynamic.main()
        
        
        ...

        
    
if __name__ == '__main__':
    manipulador = Manipulador()
    manipulador.main()
