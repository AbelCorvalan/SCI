%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Estabilidad Absoluta %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; close all; clc
pkg load symbolic
syms s C1 C2 R1 R2 real
syms R3 R4 K real

C1= 0.00001
C2= 0.00001
R1= 10000
R2= 10000
R3= 3300
R4= 5300
K_= R3/(R3+R4)
K= 1/(K_)
FT= simplify(K/((C1*C2*R1*R2*(s^2))+(R1*C1+R2*C2+R1*C2*(1-K))*s+1))

pkg load control
s= tf('s')
[N, D]= numden(FT)
G= minreal(tf(sym2poly(N), sym2poly(D)))
G= 260.6/((s^2)+4*s+100)
pole(G)
pzmap(G)
roots([1 4 100])
%step(G); grid minor

%Como las raíces de la ecuación característica de la función transferencia se
%encuentran en el semiplano izquierdo del eje imaginario tenemos que nuestro
%sistema es absoluto.

