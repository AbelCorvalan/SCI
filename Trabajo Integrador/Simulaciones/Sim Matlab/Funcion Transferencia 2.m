%%%% Funcion Transferencia 2 %%%%
clear all; close all; clc
pkg load control
s= tf('s')

K= 192
K_98= K*0.98 %K98= 188.16
ts= 40 %seg
T= ts/4
G= K/(T*s+1)

%pkg load symbolic
%syms s real
G1= 19.2/(s+0.1)
step(G, 100)
rlocus(G1)
%H= 1/(60*(s^2)+38*s+40)
H= 1/(0.08*s+1)
%F= factor(G1/(1+G1*H), s)
F= feedback(G1, H)
step(F); grid minor
rlocus(G1*H)

%PID
PI= (s+0.1)/(s)
rlocus(PI*G1*H)
rlocus(G1*H)
s1= -1
invK= abs(((s1+0.1)/(s1))*(19.2/(s1+0.1))*(1/(60*(s1^2)+38*s1+1)))
K2= 1/invK %K= 1.1979
FT= feedback(K2*PI*G1, H)
step(FT)
%ts= 17.4 seg NUEVO TIEMPO DE ESTABLECIMIENTO
FTLA= minreal(K2*PI*G1)

FTLA= 0.013021*(19.2/s)

% ANALISIS DE ERROR DE PLANTA SIN PI %
%Las constantes de error para las distintas entradas son:
clear all; close all; clc
pkg load symbolic
syms s real

G1= 19.2/(s+0.1)
%%FT1= (0.013021)*((s+0.08)/(s))*(15.36/(s+0.08))

%Kp= lim G(s)
%    s->0
Kp= G1
%%Kp= FT1

%Kv= lim s*G(s)
%    s->0
Kv= simplify(s*G1)
%%Kv= simplify(s*FT1)

%Ka= lim (s^2)*G(s)
%    s->0
Ka= simplify((s^2)*G1)
%%Ka= simplify((s^2)*FT1)

%evaluo s=0
s=0
eval(Kp)
eval(Kv)
eval(Ka)

%El error en estado estable para una entrada en escalon es ess=cte=50=1/(1+Kp)
%El error en estado estable para una entrada en escalon es ess=inf=1/Kv
%El error en estado estable para una entrada en escalon es ess=inf=1/Ka

ess_e= eval(1/(1+Kp))
ess_r= eval(1/Kv)
ess_p= eval(1/Ka)

% ANALISIS DE ERROR DE PLANTA CON CONTROLADOR PI %
%Las constantes de error para las distintas entradas son:
clear all; close all; clc
pkg load symbolic
syms s real

FTLA= 0.013021*(19.2/s)

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

%El error en estado estable para una entrada en escalon es ess=cte=50=1/(1+Kp)
%El error en estado estable para una entrada en escalon es ess=inf=1/Kv
%El error en estado estable para una entrada en escalon es ess=inf=1/Ka

ess_e= eval(1/(1+Kp))
ess_r= eval(1/Kv)
ess_p= eval(1/Ka)
