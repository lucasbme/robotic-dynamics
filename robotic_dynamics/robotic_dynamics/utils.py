import sympy as sp

class Utils:
    def __init__(self):
        self.q1, self.q2, self.q3, self.q4, self.q5 = sp.symbols('q1 q2 q3 q4 q5')
        self.d2, self.d4 = sp.symbols('d2 d4') 
        self.alpha1, self.alpha2, self.alpha3, self.alpha4, self.alpha5 = sp.symbols('alpha1 alpha2 alpha3 alpha4 alpha5')

    def dh_notation(self, a, alpha, d, theta):
        ca = sp.cos(alpha)
        sa = sp.sin(alpha)
        ct = sp.cos(theta)
        st = sp.sin(theta)

        T = sp.Matrix([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0,      sa,     ca,    d],
            [0,       0,      0,    1]
        ])
        return T

    def fk(self):
        A1 = self.dh_notation(0, self.alpha1, 0, self.q1)
        A2 = self.dh_notation(0, self.alpha2, self.d2, self.q2)
        A3 = self.dh_notation(0, self.alpha3, self.q3, 0)
        A4 = self.dh_notation(0, self.alpha4, self.d4, self.q4)
        A5 = self.dh_notation(0, self.alpha5, 0, self.q5)

        Ts = [None]*5
        Ts[0] = A1
        Ts[1] = Ts[0] @ A2
        Ts[2] = Ts[1] @ A3
        Ts[3] = Ts[2] @ A4
        Ts[4] = Ts[3] @ A5

        Tf = Ts[4]
        return Tf, Ts

robot = Utils()
Tf, Ts = robot.fk()

sp.pprint(Tf)
