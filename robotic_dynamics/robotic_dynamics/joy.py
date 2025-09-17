# Autores: Lucas Bosso de Mello
# Descrição: Código para remapear o joystick
# Data: 29/08/2025
# Última modificação por Lucas Bosso em 05/09/2025

# ========== Inclusão de Bibliotecas ==========

import yaml
from pathlib import Path
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

# ========== Classe Auxiliar ==========
# Atualiza estado dos botões e faz detecção de bordas

class Button:
    def __init__(self):
        self.state = False
        self.prevState = False

    def updateState(self, newState: int):
        self.prevState = self.state
        self.state = bool(newState)

    def risingEdge(self):
        return self.state and not self.prevState

# ========== Classe Principal de Remapeamento ==========

class Joystick(Node):
    def __init__(self, yamlPath, joyName = 'xbox', joySub = 'joy', joyPub = 'cmd_vel'):
        super().__init__('joystick_remap')

        # ========== Inicialização do joystick pelo YAML ==========

        yamlFile = Path(yamlPath)
        if not yamlFile.exists():
            raise FileNotFoundError(f"Arquivo YAML não encontrado: {yamlPath}")
        
        with open(yamlFile, 'r') as file:
            allJoys = yaml.safe_load(file)

        if joyName not in allJoys:
            raise ValueError(f"Joystick {joyName} não encontrado no YAML")
        
        joyConfig = allJoys[joyName]
        self.axesMap = joyConfig['axes']
        self.buttonsMap = joyConfig['buttons']

        # ========== Inicialização de atributos ==========

        self.threshold = 0.1
        self.joyVel = Twist()
        self.buttons = {name: Button() for name in self.buttonsMap}

        # ==================== ROS 2 ====================

        self.subscription = self.create_subscription(
            Joy,
            joySub,
            self.joyCallback,
            10
        )

        self.publisher = self.create_publisher(
            Twist,
            joyPub,
            10
        )

    # ==================== Remapeamento ====================

    # ========== Remap dos Eixos ==========
    def remapAxes(self, msg: Joy):
        self.joyVel.linear.y  = msg.axes[self.axesMap['LY']]
        self.joyVel.linear.x  = msg.axes[self.axesMap['LX']]
        self.joyVel.angular.z = msg.axes[self.axesMap['RX']]
        self.joyVel.linear.z  = msg.axes[self.axesMap['RY']]

    # ========== Remap dos Botões ==========
    def remapButtons(self, msg: Joy):
        for name, idx in self.buttonsMap.items():
            self.buttons[name].updateState(msg.buttons[idx])

    # ========== Aplica Deadzone no Joystick ==========
    def applyDeadzone(self):
        for attr in ['x','y','z']:
            val = getattr(self.joyVel.linear, attr)
            if abs(val) < self.threshold:
                setattr(self.joyVel.linear, attr, 0.0)
        if abs(self.joyVel.angular.z) < self.threshold:
            self.joyVel.angular.z = 0.0

    # ========== Publica cmd_vel ==========
    def publishCmdVel(self):
        self.applyDeadzone()
        self.publisher.publish(self.joyVel)

    # ========== Callback Geral ==========
    def joyCallback(self, msg: Joy):
        try:
            self.remapAxes(msg)
            self.remapButtons(msg)
            self.publishCmdVel()
        except Exception as e:
            self.get_logger().error(f'Erro no callback: {e}')