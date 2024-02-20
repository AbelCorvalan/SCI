%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Algoritmo de Mason %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; close all; clc
pkg load control
s= tf('s')

C1= 0.00001
C2= 0.00001
R1= 10000
R2= 10000
R3= 3300
R4= 5300
K_= R3/(R3+R4)
K= 1/(K_)

A= K/(R1*R2*C1*C2)
B= -(R1*C1+R2*C1+R1*C2*(1-K))/(R1*R2*C1*C2)
C= -1/(R1*R2*C1*C2)

%Caminos directos
M1= A*(1/s)*(1/s)

%Lazos cerrados
l1= (1/s)*B
l2= (1/s)*(1/s)*C

%Cofactor
D1= 1

%Determinante
D= 1-(l1+l2)

%Funcion Transferencia
FT= minreal((M1*D1)/D)
step(FT)

