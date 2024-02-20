%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% ANALISIS DE ERROR DE PLANTA CON CONTROLADOR PID %%%%%%%%%%%%%%%%%%%%%%%%%%
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

roots([C1*C2*R1*R2 (R1*C1+R2*C2+R1*C2*(1-K))  1])

G1= 8600/(33*(s^2)+130*s+3300)
G1= 260/((s^2)+3.939*s+100)
G1= 260/((s^2)+4*s+100)
%FT1= 260/((s-(-1.9697+9.8041*i))*(s-(-1.9697-9.8041*i)))

Td= 0.25
Ti= 0.04
%PID= Td*(((s^2)+(1/(Td))*s+(1/(Ti*Td)))/(s))


FTLA= simplify((0.03)*(Td*(((s^2)+(1/(Td))*s+(1/(Ti*Td)))/(s)))*(260/((s^2)+3.939*s+100)))

%Kp= lim G(s)
%    s->0
Kp= FTLA
%%Kp= FT1

%Kv= lim s*G(s)
%    s->0
Kv= simplify(s*FTLA)
%%Kv= simplify(s*FT1)

%Ka= lim (s^2)*G(s)
%    s->0
Ka= simplify((s^2)*FTLA)
%%Ka= simplify((s^2)*FT1)

%evaluo s=0
s=0
eval(Kp)
eval(Kv)
eval(Ka)

%El error en estado estable para una entrada en escalon es ess=cte=1/(1+Kp)
%El error en estado estable para una entrada en escalon es ess=inf=1/Kv
%El error en estado estable para una entrada en escalon es ess=inf=1/Ka

ess_e= eval(1/(1+Kp))
ess_r= eval(1/Kv)
ess_p= eval(1/Ka)

