# Autores: Lucas Bosso de Mello
# Descrição: Código principal de controle de fluxo
# Data: 05/09/2025
# Última modificação por Lucas Bosso em 05/09/2025

# ========== Inclusão de Bibliotecas ==========

import rclpy
import time
import os
from robotic_dynamics.manipulator import Manipulator
from robotic_dynamics.joy import Joystick

# ========== Função Principal ==========

def main():
    rclpy.init()

    # ========== Inicializa Joystick ==========
    currentDir = os.path.expanduser('~/ros2_ws')
    yamlPath = os.path.join(currentDir, "src/robotic_dynamics/config/joy_remap.yaml")

    joyNode = Joystick(yamlPath)

    # ========== Inicializa Manipulador ==========
    params = {"d2": 0.3, "d4": 0.4, "d6": 0.1}
    robot = Manipulator(params)

    # ========== Plot3D (Toolbox) ==========
    robot.plot(robot.q, block=False)

    # ========== ROS 2 Node ==========
    try:
        while rclpy.ok():
            rclpy.spin_once(joyNode, timeout_sec=0.01)

            # ========== Posição das Juntas (Joystick) ==========
            robot.joystick(joyNode.joyVel)

            # ========== Visualização (em Tempo Real) ==========
            robot.plot(robot.q, block=False)

            time.sleep(0.05)

    # ========== Destrutor ==========   
    except KeyboardInterrupt:
        pass
    finally:
        joyNode.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
