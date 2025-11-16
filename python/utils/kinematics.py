from spatialmath import SE3

class Kinematics:
    def __init__(self, robot=None):
        self.robot = robot
        
        # self.calc_forward_kinematics()
        # self.calc_inverse_kinematics()
        pass
    
    def calc_forward_kinematics(self, q):
        T = self.robot.fkine(q)

        print("------ FORWARD KINEMATICS ------")
        print("\nMatriz de transformação homogênea (FK):")
        print(T)

        print("\nPosição (x, y, z):", T.t)
        
        return T
        
        
        
    def calc_inverse_kinematics(self, qf: list):
        x, y, z = qf
        
        Tdes = SE3(x, y, z)

        # ===== Encontra a Solução =====
        ik_solution = self.robot.ikine_LM(Tdes, q0=[0, 0, 0.0, 0, 0])  
        
        
        if ik_solution.success:
            print("\n------ INVERSE KINEMATICS ------")
            print(f'\nEnd-effector position: {qf}')
            print("\nConfiguração de juntas encontrada:")
            q_ik = ik_solution.q 
            for i in range(5):    
                print(f'q{i} = {q_ik[i]:.3f} ')
                
        else:
            print("\nA configuração não possui solução (IK).")

        return ik_solution