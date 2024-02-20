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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Diseño del Controlador PID %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Td= 0.25
Ti= 0.04
PID= Td*(((s^2)+(1/(Td))*s+(1/(Ti*Td)))/(s))

rlocus(PID*G)
sgrid(0.707, 1), grid minor
s1= -2
invK= abs((Td*(((s1^2)+(1/(Td))*s1+(1/(Ti*Td)))/(s1)))*(260.6/((s1^2)+3.939*s1+100)))
K2= 1/invK %K= 0.0307
FTT= feedback(K2*PID*G, 1)
pole(FTT)
step(FTT)  % Respuesta del sistema con control PID
%ts= 17.4 seg, nuevo tiempo de establecimiento cumple con especificaciones
FTLA= minreal(K2*PID*G)

%FTLA= 2.97*(((s^2)+(1/(Ti*Td))*s+(1/(Ti*Td)))/(s))*(260.6/((s^2)+3.939*s+100))

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% ANALISIS DE ERROR DE PLANTA SIN PID %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

G1= 260.6/((s^2)+4*s+100)

% Las constantes de error para las distintas entradas son:

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

%El error en estado estable para una entrada en escalon es ess=cte=1/(1+Kp)
%El error en estado estable para una entrada en escalon es ess=inf=1/Kv
%El error en estado estable para una entrada en escalon es ess=inf=1/Ka

ess_e= eval(1/(1+Kp))
ess_r= eval(1/Kv)
ess_p= eval(1/Ka)

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

