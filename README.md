# Robô Móvel com Câmera Acoplada ao Manipulador para Inspeção

Projeto de "Dinâmica de Sistemas Robóticos" desenvolvido por ... O objetivo é o desenvolvimento de um robô de inspeção em plataformas offshore.



## Índice

* [Visão Geral do Projeto](#visão-geral-do-projeto)
* [Manipulador](#manipulador)


## Visão Geral do Projeto

Este projeto consiste no desenvolvimento de um robô móvel destinado à inspeção e leitura de sensores em áreas de alta pressão.

### O Problema
As principais dificuldades identificadas para este tipo de operação incluem:
* Locomoção em superfícies irregulares e com obstáculos.
* Garantia da localização e locomoção precisas do robô.
* Precisão na leitura de medidores e sensores no ambiente.

### Solução Proposta
Para resolver esses desafios, a arquitetura do projeto combina:
* Plataforma Móvel: Um robô quadrúpede, escolhido por seu desempenho superior em terrenos irregulares.
* Manipulador: Um braço robótico com uma câmera acoplada na ponta para realizar a inspeção visual.
* Visão Computacional:
    * Leitura de medidores para inspeção.
    * Uso de SLAM para mapeamento do ambiente e localização do robô.


## Manipulador

Para que as operações sejam corretamente realizadas foi proposto um manipulador RRPRR, 4 juntas rotacionais e 1 prismática, totalizando 5 GDL.

### Especificações do Manipulador
O design do manipulador foi pensado para o ambiente offshore:
* Carenagem Fechada: proteção contra riscos de explosão;
* Material: alumínio, com o intuito de aliviar a massa e garantir a vedação;
* Câmera: compartimento vedado.

### CAD 

O CAD do manipulador foi realizado através do software *Inventor*.

<p align="center">
  <img src="https://github.com/user-attachments/assets/ad309d2d-8996-4a39-a27b-e9d01af86f6b" alt="CAD do manipulador" width="50%">
</p>

### Parâmetros de Denavit-Hartenberg

Seguindo o modelo proposto, com 5 GDL (RRPRR), a com os parâmetros de Denavit-Hartenberg do manipulador foram agrupados na tabela a seguir:

| Junta (j) | $\theta_j$ (theta) | $d_j$ (dist) | $a_j$ (dist) | $\alpha_j$ (alfa) |
|:---------:|:------------------:|:------------:|:------------:|:-----------------:|
| 1         | $q_1$              | $d_1$        | 0            | $-\pi/2$          |
| 2         | $q_2$              | $d_2$        | 0            | $\pi/2$           |
| 3         | $-\pi/2$           | $q_3$        | 0            | 0                 |
| 4         | $q_4$              | 0            | 0            | $\pi/2$           |
| 5         | $q_5$              | 0            | 0            | $\pi/2$           |
