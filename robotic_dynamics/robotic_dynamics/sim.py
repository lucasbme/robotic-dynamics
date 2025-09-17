# Autores: Lucas Bosso de Mello
# Descrição: Código principal de controle de fluxo
# Data: 05/09/2025
# Última modificação por Lucas Bosso em 17/09/2025

# ========== Inclusão de Bibliotecas ==========

import numpy as np
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
    
    # ========== Aplica Cinemática Inversa ==========
    T = robot.fkine(q)

    robot.plot(q, block=True)

    print("Matriz de transformação homogênea (FK):")
    print(T)

    print("\nPosição (x, y, z):", T.t)

if __name__ == '__main__':
    main()
