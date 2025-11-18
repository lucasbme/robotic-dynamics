class Jacobian:
    def __init__(self, robot):
        self.robot = robot
        
    def calc_jacobian(self, q, T):
        print("\n------ JACOBIAN ------\n")
        jacobian = self.robot.jacob0(q, T)
        J = []
        for line in jacobian:
            jacobian_line = []
            for value in line:
                value = round(value, 2)   
                
                if value == -0.0:
                    value = 0.0
                    
                jacobian_line.append(value)   
            J.append(jacobian_line)
        
        for line in J:
            print(line)
        
        return J