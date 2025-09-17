# Autores: Lucas Bosso de Mello
# Descrição: Código principal de controle de fluxo
# Data: 05/09/2025
# Última modificação por Lucas Bosso em 17/09/2025

# ========== Inclusão de Bibliotecas ==========

import numpy as np
from spatialmath import SE3
from robotic_dynamics.manipulator import Manipulator
from robotic_dynamics.joy import Joystick

# ========== Função Principal ==========

def main():
    # ========== Inicializa Manipulador ==========
    params = {"d2": 0.2, "d4": 0.2}
    q = [np.deg2rad(0),   # q1
         np.deg2rad(0),   # q2
         0.2,             # q3 (prismática)
         np.deg2rad(0),   # q4
         np.deg2rad(0)]   # q5

    # ========== Cria a Instância ==========
    robot = Manipulator(params)
    
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

    # ========== Plot ==========
    robot.plot(ikSolution.q, block=True)


if __name__ == '__main__':
    main()
