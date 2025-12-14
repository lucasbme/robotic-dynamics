import roboticstoolbox as rtb
from roboticstoolbox import jtraj, ctraj
import numpy as np
import matplotlib.pyplot as plt

class Trajectory:
    def __init__(self, robot, jacobian_handler=None):
        self.robot = robot
        self.jacobian_handler = jacobian_handler
        
        for link in self.robot.links:
            if link.m == 0:
                link.m = 1.5  # 1.5 kg por elo
                link.r = [0.1, 0, 0] # Centro de massa deslocado no eixo X do elo
                link.I = [0.01, 0.01, 0.01]
    
    
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

    def trajectory_cartesian_space(self, p_final):
        T = 5.0
        dt = 0.02
        steps = int(T / dt)
        
        # Estados iniciais
        q = self.robot.q.copy()
        p_start = self.robot.fkine(q).t
        p_end = np.array(p_final)
        
        # Arrays para plotagem
        t_arr = np.arange(0, steps*dt, dt)
        dados = {'q': [], 'qd': [], 'qdd': [], 'tau': []}
        p_log = []
        dq_ant = np.zeros(5)

        print("Calculando trajetória (Polinômio de 5º grau)...")

        for i in range(steps):
            t = i * dt
            
            # Polinomio de 5º Grau - garante suavidade total 
            tau = t / T
            s = 10*(tau**3) - 15*(tau**4) + 6*(tau**5)
            
            p_ref = p_start + s * (p_end - p_start)
            
            #  Controle proporcional de posição 
            p_atual = self.robot.fkine(q).t
            err = p_ref - p_atual
            v = err * 2.0 
            
            # Cinemática Inversa Diferencial --- Linhas de posição do jacobiano 
            J = self.robot.jacob0(q)[:3, :]
            
            dq = np.linalg.pinv(J) @ v
            
            # Dinamica Inversa (RNE) 
            qdd = (dq - dq_ant) / dt
            tau = self.robot.rne(q, dq, qdd)
            
            # Salva dados
            dados['q'].append(q.copy())
            dados['qd'].append(dq.copy())
            dados['qdd'].append(qdd.copy())
            dados['tau'].append(tau.copy())
            p_log.append(p_atual.copy())

            # Integra para o proximo passo
            q = q + (dq * dt)
            dq_ant = dq
        
        self.robot.q = q
        print("Gerando gráficos...")
        
        # Plot 1: Animação
        q_arr = np.array(dados['q'])
        self.robot.plot(q_arr, backend="pyplot", dt=dt, block=False)
        
        # Plot 2: Análise dinâmica e trajetória
        self.plot_graphs(t_arr, q_arr, np.array(dados['qd']), 
                        np.array(dados['qdd']), np.array(dados['tau']), np.array(p_log))

    def plot_graphs(self, t, q, qd, qdd, tau, traj ):
        fig, ax = plt.subplots(4, 1, figsize=(8, 12), sharex=True)
        nomes = ["J1 - Base", "J2 - Ombro", "J3 - Prisma", "J4 - Cotovelo", "J5 - Efetuador"]
        
        for i in range(5):
            ax[0].plot(t, q[:, i], label=nomes[i])
            ax[1].plot(t, qd[:, i])
            ax[2].plot(t, qdd[:, i])
            ax[3].plot(t, tau[:, i])
            
        ax[0].set_title('Posição'); ax[0].set_ylabel('m'); ax[0].legend(loc='right')
        ax[1].set_title('Velocidade'); ax[1].set_ylabel('rad/s')
        ax[2].set_title('Aceleração'); ax[2].set_ylabel('rad/s²')
        ax[3].set_title('Torque'); ax[3].set_ylabel('N.m'); ax[3].set_xlabel('Tempo (s)')
        
        for a in ax: a.grid(True, alpha=0.5)
        plt.tight_layout()

        fig2, ax_c = plt.subplots(figsize=(8, 6))
        ax_c.plot(t, traj[:, 0], label='X ', color='r', linewidth=2)
        ax_c.plot(t, traj[:, 1], label='Y ', color='g', linewidth=2, linestyle='--')
        ax_c.plot(t, traj[:, 2], label='Z ', color='b', linewidth=2)
        
        ax_c.set_title('Trajetória cartesiana do efetuador (XYZ)')
        ax_c.set_xlabel('Tempo (s)')
        ax_c.set_ylabel('Posição (m)')
        ax_c.legend()
        ax_c.grid(True, alpha=0.5)
        fig2.tight_layout()
        plt.show(block=True)