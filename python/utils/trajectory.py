from roboticstoolbox import jtraj
import numpy as np

class Trajectory:
    def __init__(self, robot):
        self.robot = robot
    
    
    def define_trajectory(self, ik_solution):
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
        