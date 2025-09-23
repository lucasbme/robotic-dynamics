# Autores: Lucas Bosso de Mello, Lucas Maroun de Almeida
# Descrição: Código principal de controle de fluxo
# Data: 05/09/2025
# Última modificação por Lucas Maroun em 23/09/2025

# ========== Inclusão de Bibliotecas ==========

import numpy as np                                      # Matemática
from spatialmath import SE3                             # Representação Espacial
import roboticstoolbox as rtb                           # Toolbox Peter Corke

# ========== Função Principal ==========

def main():
    # ========== Inicializa Manipulador ==========
    params = {"d1": 0.2, "d2": 0.2} # Parâmetros do Manipulador

    robot = rtb.DHRobot([
        rtb.RevoluteDH( d = params["d1"], a = 0, alpha = -np.pi / 2),
        rtb.RevoluteDH( d = params["d2"], a = 0, alpha = np.pi / 2),
        rtb.PrismaticDH(theta = -np.pi/2, a = 0, alpha = 0, offset=0.2, qlim=[0.0, 0.2]),
        rtb.RevoluteDH( d = 0,            a = 0, alpha = -np.pi / 2),
        rtb.RevoluteDH( d = 0,            a = 0, alpha = np.pi / 2)
    ])

    # Vetor de Estados das Juntas
    q = [np.deg2rad(0),   # q1
         np.deg2rad(0),   # q2
         0.2,             # q3 (prismática)
         np.deg2rad(0),   # q4
         np.deg2rad(0)]   # q5

    
    # ========== Aplica Cinemática Direta ==========
    T = robot.fkine(q)

    print("Matriz de transformação homogênea (FK):")
    print(T)

    print("\nPosição (x, y, z):", T.t)

    # ========== Aplica Cinemática Inversa ==========
    # ===== Posição Desejada =====
    qf = {
        "x": 0.2,
        "y": 0.0,
        "z": 0.3
    }                                  

    # ===== Gera Matriz Homogênea =====
    Tdes = SE3(qf["x"], qf["y"], qf["z"])                   

    # ===== Encontra a Solução =====
    ikSolution = robot.ikine_LM(Tdes, q0=[0, 0, 0.0, 0, 0])  

    # ===== Verifica se a Solução é Viável =====
    if ikSolution.success:
        print("\nConfiguração de juntas encontrada (IK):")
        print(ikSolution.q)
    else:
        print("\nA configuração não possui solução (IK).")

    # ========== Jacobiano ==========
    # print('\nJacobiano: ')
    # J = robot.jacob0(q, T)

    # print(J)

    # ========== Plot ==========
    # robot.plot(robot.q, block=True)       # Plot do manipulador na notação DH
    # robot.plot(ikSolution.q, block=True)  # Posição das juntas após IK

    # ========== Trajetoria ========== 
    qStart = np.array([0, 0, 0, 0, 0])
    qEnd = ikSolution.q          # Posição das juntas via IK
    
    # Tempo de simulação 
    tSimulation = 10                
    dt = 0.05                       
    steps = int(tSimulation / dt)    
    t = np.linspace(0, tSimulation, steps)

    # Aplica trajetória polinomial
    traj = rtb.jtraj(qStart, qEnd, t)
    robot.plot(traj.q, backend="pyplot", block=True, dt=dt)

if _name_ == '_main_':
    main()
