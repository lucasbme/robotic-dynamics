# Robô Móvel com Câmera Acoplada ao Manipulador para Inspeção

Projeto de "Dinâmica de Sistemas Robóticos" desenvolvido por ... O objetivo é o desenvolvimento de um robô de inspeção em plataformas offshore.



## Índice

* [Visão Geral do Projeto](#visão-geral-do-projeto)
* [Parâmetros de Denavit-Hatenberg](#-parametros-dh)
* [Cinemática Direta](#-direta)
* [Cinemática Inversa](#-inversa)
* [Jacobiana](#-jacob)
* [Dinâmica do Manipulador](#-dinamica)
* [Planjedor de Trajetória](#-trajetoria)
* [Resultados](#-resultados)
* [Autores](#-autores)

## Visão Geral do Projeto

Este projeto consiste no desenvolvimento de um robô móvel destinado à inspeção e leitura de sensores em áreas de alta pressão.

### O Problema
As principais dificuldades identificadas para este tipo de operação incluem:
* Locomoção em superfícies irregulares e com obstáculos.
* Garantia da localização e locomoção precisas do robô.
* Precisão na leitura de medidores e sensores no ambiente.

### Solução Proposta
Para resolver esses desafios, a arquitetura do projeto combina:
* **Plataforma Móvel:** Um **robô quadrúpede**, escolhido por seu desempenho superior em terrenos irregulares.
* **Manipulador:** Um braço robótico com uma **câmera acoplada** na ponta para realizar a inspeção visual.
* **Visão Computacional:**
    * Leitura de medidores para inspeção.
    * Uso de **SLAM** para mapeamento do ambiente e localização do robô.
