from spatialmath import SE3

class Kinematics:
    def __init__(self, robot=None):
        self.robot = robot
        
    
    def calc_forward_kinematics(self, q) -> None:
        self.T = self.robot.fkine(q)

        print("------ FORWARD KINEMATICS ------")
        print("\nMatriz de transformação homogênea (FK):")
        print(self.T)

        print("\nPosição (x, y, z):", self.T.t)
        
        return self.T
        
        
        
    def calc_inverse_kinematics(self, qf: list) -> None:
        x, y, z = qf
        
        Tdes = SE3(x, y, z)

        # ===== Encontra a Solução =====
        self.ik_solution = self.robot.ikine_LM(Tdes, q0=[0, 0, 0.0, 0, 0])  
        
        
        if self.ik_solution.success:
            print("\n------ INVERSE KINEMATICS ------")
            print(f'\nEnd-effector position: {qf}')
            print("\nConfiguração de juntas encontrada:")
            q_ik = self.ik_solution.q 
            for i in range(5):    
                print(f'q{i} = {q_ik[i]:.2f} ')
                
                
        else:
            print("\nA configuração não possui solução (IK).")

        return self.ik_solution