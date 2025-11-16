# Autores: Lucas Bosso de Mello, Lucas Maroun de Almeida
# Descrição: Código principal de controle de fluxo
# Data: 05/09/2025
# Última modificação por Lucas Maroun em 23/09/2025

# ========== Inclusão de Bibliotecas ==========

import numpy as np                                      # Matemática
import roboticstoolbox as rtb                           # Toolbox Peter Corke

# from .dynamic import Dynamics
from utils import (
    Kinematics, 
    Jacobian,
    Trajectory,
    Dynamics,
)


# ========== Trajetoria ==========
def trajectory(robot, qStart, ikSolution):
    qEnd = ikSolution.q          # Posição das juntas via IK
    
    # Tempo de simulação 
    tSimulation = 10                
    dt = 0.05                       
    steps = int(tSimulation / dt)    
    t = np.linspace(0, tSimulation, steps)

    # Aplica trajetória polinomial
    traj = rtb.jtraj(qStart, qEnd, t)
    robot.plot(traj.q, backend="pyplot", dt=dt, block=True)
    


# ========== Função Principal ==========
def main():
    # ========== Inicializa Manipulador ==========
    params = {"d1": 0.2, "d2": 0.1} # Parâmetros do Manipulador

    robot = rtb.DHRobot([
        rtb.RevoluteDH( d = params['d1'], a = 0, alpha = -np.pi / 2),
        rtb.RevoluteDH( d = params['d2'], a = 0, alpha = np.pi / 2),
        rtb.PrismaticDH(theta = -np.pi/2, a = 0, alpha = 0, offset=0.2, qlim=[0.0, 0.5]),
        rtb.RevoluteDH( d = 0,            a = 0, alpha = np.pi / 2),
        rtb.RevoluteDH( d = 0,            a = 0, alpha = np.pi / 2)
    ])

    
    # Vetor de Estados das Juntas
    q = [0,   # q1
         0,   # q2
         0.2,             # q3 (prismática)
         0,  # q4
         0]   # q5
    
    qf = [-0.3, 0.1, 0.6]


    kinematics = Kinematics(robot)
    T = kinematics.calc_forward_kinematics(q)
    
    ik_solution = kinematics.calc_inverse_kinematics(qf)

    jacob = Jacobian(robot)
    jacob.calc_jacobian(q, T)

    # ========== Plot ==========
    robot.plot(robot.q, block=True)       # Plot do manipulador na notação DH
    # robot.plot(ikSolution.q, block=True)  # Posição das juntas após IK


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
        ...

        
    
if __name__ == '__main__':
    # manipulador = Manipulador()
    # manipulador.main()
    main()
