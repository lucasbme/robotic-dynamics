import roboticstoolbox as rtb
from roboticstoolbox import jtraj, ctraj
import numpy as np
from spatialmath import SE3

class Trajectory:
    def __init__(self, robot):
        self.robot = robot
    
    
    def trajectory_joint_space(self, ik_solution):
        if ik_solution.success:
            q_start = np.zeros(5)
            q_end = ik_solution.q          # Posição das juntas via IK
            
            # Tempo de simulação 
            t_simulation = 10                
            dt = 0.05                       
            steps = int(t_simulation / dt)    
            t = np.linspace(0, t_simulation, steps)

            # Aplica trajetória polinomial
            traj = jtraj(q_start, q_end, t)
            self.robot.plot(traj.q, backend="pyplot", dt=dt, block=True) 
            
            print('\nTrajetória gerada', 
                  f'para a configuração {ik_solution.q}')
        else: 
            print('\nNão é possível gerar uma', 
                  'trajetória com essa configuração')

    def trajectory_cartesian_space(self, pos_end):
        
        # Tempo de simulação 
        t_simulation = 10                
        dt = 0.05                       
        steps = int(t_simulation / dt)    
        t = np.linspace(0, t_simulation, steps)
        
        
        pos_start = [0, 0, 0] # Posição inicial
    
        T_start = SE3(pos_start) 
        T_end = SE3(pos_end) 
        
        cartesian_traj = ctraj(T_start, T_end, t=t) 
        
    
        q_cartesian_traj = self.robot.ikine_LM(cartesian_traj, q0= [0, 0, 0.2, 0, 0])
        self.robot.plot(q_cartesian_traj.q, backend="pyplot", dt=dt, block=True) 
        
            
        ... 
        