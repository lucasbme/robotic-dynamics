# Robô Móvel com Câmera Acoplada ao Manipulador para Inspeção

Projeto de "Dinâmica de Sistemas Robóticos" desenvolvido por Leonardo Ortiz, Lucas Bosso de Mello (@lucasbme), Lucas Maroun de Almeida (@MarounLucas), Rafael Janowski Pozzer, Sophia de Souza Nobre Benevides (@benesophi) e Vítor Magalhães Dourado Soares. 

**Nota:** os referenciados com @ são autores do código.

O objetivo é o desenvolvimento de um manipulador robótico para robô de inspeção em plataformas offshore.



## Índice

* [Visão Geral do Projeto](#visão-geral-do-projeto)
* [Manipulador](#manipulador)
* [Manual de Uso](#manual-de-uso)


## 1. Visão Geral do Projeto

Este projeto consiste no desenvolvimento de um manipulador para um robô móvel destinado à inspeção e leitura de sensores em áreas de alta pressão.

### 1.1 O Problema
As principais dificuldades identificadas para este tipo de operação incluem:
* Locomoção em superfícies irregulares e com obstáculos.
* Garantia da localização e locomoção precisas do robô.
* Precisão na leitura de medidores e sensores no ambiente.

### 1.2 Solução Proposta
Para resolver esses desafios, a arquitetura do projeto combina:
* Plataforma Móvel: Um robô quadrúpede, escolhido por seu desempenho superior em terrenos irregulares.
* Manipulador: Um braço robótico com uma câmera acoplada na ponta para realizar a inspeção visual.
* Visão Computacional:
    * Leitura de medidores para inspeção.
    * Uso de SLAM para mapeamento do ambiente e localização do robô.

### 1.3 Implementação MATLAB/Python

O artifício principal do projeto é o Toolbox do Peter Corke implementado nos scripts em Python. A implementação no MATLAB serve como validação para as equações encontradas.

## 2. Manipulador

Para que as operações sejam corretamente realizadas foi proposto um manipulador RRPRR, 4 juntas rotacionais e 1 prismática, totalizando 5 GDL.

### 2.1 Especificações do Manipulador
O design do manipulador foi pensado para o ambiente offshore:
* Carenagem Fechada: proteção contra riscos de explosão;
* Material: alumínio, com o intuito de aliviar a massa e garantir a vedação;
* Câmera: compartimento vedado.

### 2.2 CAD 

O CAD do manipulador foi realizado através do software *Inventor*.

<p align="center">
  <img src="https://github.com/user-attachments/assets/ad309d2d-8996-4a39-a27b-e9d01af86f6b" alt="CAD do manipulador" width="50%">
</p>

### 2.3 Parâmetros de Denavit-Hartenberg

Seguindo o modelo proposto, com 5 GDL (RRPRR), a com os parâmetros de Denavit-Hartenberg do manipulador foram agrupados na tabela a seguir:

| Junta (j) | $\theta_j$ (theta) | $d_j$ (dist) | $a_j$ (dist) | $\alpha_j$ (alfa) |
|:---------:|:------------------:|:------------:|:------------:|:-----------------:|
| 1         | $q_1$              | $d_1$        | 0            | $-\pi/2$          |
| 2         | $q_2$              | $d_2$        | 0            | $\pi/2$           |
| 3         | $-\pi/2$           | $q_3$        | 0            | 0                 |
| 4         | $q_4$              | 0            | 0            | $\pi/2$           |
| 5         | $q_5$              | 0            | 0            | $\pi/2$           |

## 3. Manual de Uso 

O uso desse pacote pode ser divido em duas partes. A implementação em Python (Toolbox do Peter Corke) possui a simulação principal do projeto com o URDF. Além disso, a modelagem feita com o Toolbox foi tomada como referência para o implementação manual feita mo Matlab. Desssa forma, os valores retornados pelo Python foram utilizados como validação da implementação feita no Matlab.

A seguir, uma descrição mais de detalhada de como utilizar o pacote, tanto em Python, quanto Matlab.

### 3.1 Python (Toolbox Peter Corke)

A simulação principal em Python está nomeada como ```main.py```. Esta classa possui a chamada de todos os métodos para a simulação


### 3.2 Matlab (Validação do Modelo)

O código principal está nomeado como ```demo.m```. Este rodas as funções de notação DH (```dh_notation.m```), cinemática direta (```fk.m```), inversa (```ik.m```), jacobiano (```jacobian.m```) e plot 3d (```plot3d.m```).

Dentro de ```demo.m```, a variável ```params``` possui as características do manipulador, incluindo o tamanho dos elos e as rotações em torno das juntas.

```matlab
% Comprimento dos elos
params.d2 = 0.20;   % Distância entre as juntas 1 e 2 [m]
params.d3 = 0.20;   % Comprimento da prismática [m]
params.d4 = 0.20;   % Distância entre as juntas 3 e 4 [m]
params.d5 = 0.00;   % Comprimento do end-effector [m]

% Rotações das juntas
params.q1 = deg2rad(0); % Rotação junta 1
params.q2 = deg2rad(0); % Rotação junta 2
params.q4 = deg2rad(0); % Rotação junta 4
params.q5 = deg2rad(0); % Rotação junta 5
```

A implementação das funções ocorre no trecho a seguir. Aquela consiste em encontrar as transformações da cinemática direta e gerar um plot, encontrar a cinemática inversa a partir de um ponto solicitado (```q_des```) e gerar um plot, e encontrar o jacobiano.

```matlab
[Tf,Tp] = fk(q, params);
disp('Tf:'); disp(Tf);

plot3d(Tp, 1);

% ========== Cinemática Inversa ========== 
q_des = [0.2, 0.0, 0.3];
[q_sol, Tf_sol, Ts_sol] = ik(q_des, params);
disp('Solutions: '); disp(q_sol);

plot3d(Ts_sol, 2);

% ========== Jacobiano ========== 
J = jacobian(q, params);
disp('Jacobiano: '); disp(J);
```

Os valores retornados podem ser validados ao se comparar com os obtidos pelo Toolbox.
