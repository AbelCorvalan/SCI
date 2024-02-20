%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%  Respuesta del sistema   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
step(G); grid minor
pole(G)
%  Polos de la función de transferencia:
% -2.0 + 9.798i
% -2.0 - 9.798i

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Aproximación de la Función Transferencia %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Tp= 0.31
ymax= 3.97
yss= 2.61
MP= (ymax-yss)/yss
MP_= MP*100
psita= sqrt( ((log(MP))^2 / (pi^2 + (log(MP))^2 )) )
w= pi/Tp
wn= w/sqrt(1-(psita)^2)

FT1= yss*(wn)^2/(s^2+2*psita*wn*s+wn^2)
step(FT1); grid minor
pole(FT1)
%  Polos de la función de transferencia:
%  -2.103 + 10.134i
%  -2.103 - 10.134i
