%30/6
%clear all; close all; clc
%pkg load symbolic
%syms s C1 C2 R1 R2 real
%syms R3 R4 K real

%C1= 0.00001
%C2= 0.00001
%R1= 10000
%R2= 10000
%R3= 3300
%R4= 5000
%K_= R3/(R3+R4)
%K= 1/(K_)
%FT= simplify(K/((C1*C2*R1*R2*(s^2))+(R1*C1+R2*C2+R1*C2*(1-K))*s+1))

clear all; close all; clc
pkg load symbolic
syms s C1 C2 R1 R2 real
syms R3 R4 K real

C1= 0.00001
C2= 0.00001
R1= 150
R2= 150
R3= 1000
R4= 680
K_= R3/(R3+R4)
K= 1/(K_)
FT= simplify(K/((C1*C2*R1*R2*(s^2))+(R1*C1+R2*C2+R1*C2*(1-K))*s+1))

pkg load control
s= tf('s')
[N, D]= numden(FT)
G= minreal(tf(sym2poly(N), sym2poly(D)))
G= 260.6/((s^2)+4.848*s+100)
G= 260.6/((s^2)+5*s+100)
step(G); grid minor
pole(G)
rlocus(G)
% -2.424 + 9.7018i
% -2.424 - 9.7018i
%[z,p,k]= zpkdata(FT)

%PID
Td= 0.206
Ti= 0.0485
PID= Td*(((s^2)+(1/(Td))*s+(1/(Ti*Td)))/(s))

rlocus(PID*G)
sgrid(0.61, 1)
s1= -5
%s1= 4.5
invK= abs((Td*(((s1^2)+(1/(Td))*s1+(1/(Ti*Td)))/(s1)))*(260.6/((s1^2)+3.939*s1+100)))
K2= 1/invK %K= 0.0972
FTT= feedback(K2*PID*G, 1)
pole(FTT)
step(FTT)


%Sobrepasamiento
ymax= 1.089
yss= 1
MP= (ymax-yss)/yss
psita= sqrt( ((log(MP))^2 / (pi^2 + (log(MP))^2 )) )
