import numpy as np
import roboticstoolbox as rtb
import sympy as sp

class Dynamics:
    def __init__(self, robot=None):
        self.robot = robot
        self.define_symbols()
        self.init_dh_matrices()
        self.init_masses()

    # =====================================================
    # 1. Variáveis simbólicas
    # =====================================================
    def define_symbols(self):
        # Coordenadas generalizadas
        self.q   = sp.symbols('q1 q2 q3 q4 q5')           # Posições
        self.qd  = sp.symbols('qd1 qd2 qd3 qd4 qd5')      # Velocidades
        self.qdd = sp.symbols('qdd1 qdd2 qdd3 qdd4 qdd5') # AceleraçÕes
        self.g   = sp.Symbol('g')                         # Gravidade

        # Parâmetros do manipulador
        self.d1, self.d2 = sp.symbols('d1 d2')
        self.alpha = [-sp.pi/2, sp.pi/2, 0, sp.pi/2, sp.pi/2]

    # =====================================================
    # 2. Matriz DH simbólica
    # =====================================================
    @staticmethod
    def define_dh_matrix(theta, d, a, alpha):
        return sp.Matrix([
            [sp.cos(theta), -sp.sin(theta)*sp.cos(alpha),  sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
            [sp.sin(theta),  sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
            [0,              sp.sin(alpha),                sp.cos(alpha),               d],
            [0,              0,                            0,                           1]
        ])

    # =====================================================
    # 3. Transformações homogêneas acumuladas
    # =====================================================
    def init_dh_matrices(self):
        q1, q2, q3, q4, q5 = self.q # Coordenadas configuradas
        d1, d2 = self.d1, self.d2   # Parametros
        a = 0                       # Parametros

        # Juntas: R, R, P, R, R
        
        A1 = self.define_dh_matrix(q1, d1, a, self.alpha[0])
        A2 = self.define_dh_matrix(q2, d2, a, self.alpha[1])
        A3 = self.define_dh_matrix(-sp.pi/2, q3, a, self.alpha[2])
        A4 = self.define_dh_matrix(q4, 0, a, self.alpha[3])
        A5 = self.define_dh_matrix(q5, 0, a, self.alpha[4])

        self.A = [A1, A2, A3, A4, A5]

        # Acumular transformações
        self.Tf = []                  # Transformação final
        Tp = sp.eye(4)                # Transformações parciais
        for Ai in self.A:
            Tp = sp.simplify(Tp * Ai)
            self.Tf.append(Tp)

        self.T = self.Tf[-1]          # Transformação final (atributo)

    # =====================================================
    # 4. Definição de massas, inércias e centros de massa
    # =====================================================
    def init_masses(self):
        self.m = sp.symbols('m1 m2 m3 m4 m5')
        self.l = sp.symbols('l1 l2 l3 l4 l5')

        # Centro de massa no meio do elo (em coords locais)
        self.r = [sp.Matrix([0, 0, self.l[i]/2, 1]) for i in range(5)]

        # Inércia simplificada (cilindro)
        self.I = []
        for i in range(5):
            Ixx, Iyy, Izz = sp.symbols(f'Ixx{i+1} Iyy{i+1} Izz{i+1}')
            self.I.append(sp.diag(Ixx, Iyy, Izz))

    # =====================================================
    # 5. Energia cinética (completa)
    # =====================================================
    def calc_kinetic_enercy(self):
        qd = sp.Matrix(self.qd)
        Tf = 0

        for i in range(5):
            Ti = self.Tf[i]
            Pi = Ti * self.r[i]  # posição do centro de massa
            pos = sp.Matrix(Pi[:3, 0])

            # Jacobianos linear e angular
            Jv = []
            Jw = []

            for j in range(5):
                # Transformação até a junta j
                if j == 0:
                    Tj = self.Tf[0]
                else:
                    Tj = self.Tf[j-1]
                zj = Tj[:3, 2]
                oj = Tj[:3, 3]

                if j > i:  # junta não afeta elo i
                    Jv.append(sp.zeros(3, 1))
                    Jw.append(sp.zeros(3, 1))
                elif j == 2:  # prismática
                    Jv.append(zj)
                    Jw.append(sp.zeros(3, 1))
                else:  # rotacional
                    Jv.append(zj.cross(pos - oj))
                    Jw.append(zj)

            Jv = sp.Matrix.hstack(*Jv)
            Jw = sp.Matrix.hstack(*Jw)

            vi = Jv * qd
            wi = Jw * qd

            # Energia cinética total do elo
            T_i = (1/2) * self.m[i] * (vi.T * vi)[0] + (1/2) * (wi.T * self.I[i] * wi)[0]
            Tf += T_i

        self.Texpr = sp.simplify(Tf)
        return self.Texpr

    # =====================================================
    # 6. Energia potencial
    # =====================================================
    def calc_potential_energy(self):
        g = self.g
        V = 0

        for i in range(5):
            Ti = self.Tf[i]
            Pi = Ti * self.r[i]
            z = Pi[2]
            vi = self.m[i] * g * z
            V += vi

        self.Vexpr = sp.simplify(V)
        return self.Vexpr

    # =====================================================
    # 7. Visualização
    # =====================================================
    def show_energies(self):
        print("\nEnergia Cinética T(q, q̇):")
        sp.pretty_print(self.T_expr)
        print("\nEnergia Potencial V(q):")
        sp.pretty_print(self.Vexpr)

    # =====================================================
    # 8. Aplica Euler-Lagrange
    # =====================================================
    def calc_lagrange_equations(self):
        """
        Calcula as equações de Euler-Lagrange e extrai M(q), C(q,qdot) e G(q).
        Retorna: (tauLagrange, M, C, G)
        """

        # vetores simbólicos como Matrix para facilitar operações
        q   = sp.Matrix(self.q)
        qd  = sp.Matrix(self.qd)
        qdd = sp.Matrix(self.qdd)

        # Lagrangiano
        L = self.Texpr - self.Vexpr

        # Tau via Lagrange direto
        tau = sp.zeros(5, 1)
        for i in range(5):
            dLdqi = sp.diff(L, q[i])
            dLdqdi = sp.diff(L, qd[i])
            ddt_dLdqdi = 0
            for j in range(5):
                ddt_dLdqdi += sp.diff(dLdqdi, q[j]) * qd[j] + sp.diff(dLdqdi, qd[j]) * qdd[j]
            tau[i, 0] = sp.simplify(ddt_dLdqdi - dLdqi)

        # Matriz de Inércia M(q)
        M = sp.zeros(5)
        for i in range(5):
            for j in range(5):
                M[i, j] = sp.simplify(sp.diff(sp.diff(self.T_expr, qd[i]), qd[j]))

        # Vetor gravitacional G(q)
        G = sp.Matrix([sp.simplify(sp.diff(self.Vexpr, qi)) for qi in q])

        # Matriz de Coriolis C(q, qd)
        # Usando fórmula de Christoffel: C_ij = 1/2 * sum_k ( dM_ij/dq_k + dM_ik/dq_j - dM_jk/dq_i ) * qd_k
        C = sp.zeros(5)
        for i in range(5):
            for j in range(5):
                Cij = 0
                for k in range(5):
                    c_ijk = (sp.diff(M[i, j], q[k]) + sp.diff(M[i, k], q[j]) - sp.diff(M[j, k], q[i])) / 2
                    Cij += c_ijk * qd[k]
                C[i, j] = sp.simplify(Cij)

        # Eq M*qdd + C*qd + G
        tauMCG = sp.simplify(M * qdd + C * qd + G)

        # Simplificação
        check = sp.simplify(tau - tauMCG)

        # Logs
        print("\n=== Equações de Euler-Lagrange (tau) ===")
        sp.pretty_print(sp.Matrix(tau))
        print("\n=== Matriz de Inércia M(q) ===")
        sp.pretty_print(M)
        print("\n=== Matriz de Coriolis C(q, q̇) ===")
        sp.pretty_print(C)
        print("\n=== Vetor gravitacional G(q) ===")
        sp.pretty_print(G)

        # Salvar variáveis
        self.M = M
        self.C = C
        self.G = G
        self.tauLagrange = tau
        self.tauMCG = tauMCG

        return tau, M, C, G

    # =========================================================
    # MAIN
    # =========================================================
    def main():
        params = {"d1": 0.2, "d2": 0.1}

        robot = rtb.DHRobot([
            rtb.RevoluteDH(d=params['d1'], a=0, alpha=-np.pi / 2),
            rtb.RevoluteDH(d=params['d2'], a=0, alpha=np.pi / 2),
            rtb.PrismaticDH(theta=-np.pi/2, a=0, alpha=0, offset=0.0, qlim=[0.0, 0.5]),
            rtb.RevoluteDH(d=0, a=0, alpha=np.pi / 2),
            rtb.RevoluteDH(d=0, a=0, alpha=np.pi / 2)
        ])

        dyn = Dynamics(robot)

        T = dyn.calc_kinetic_enercy()
        V = dyn.calc_potential_energy()

        # dyn.show_energy()

        tau, M, C, G = dyn.calc_lagrange_equations()


if __name__ == "__main__":
    Dynamics.main()
