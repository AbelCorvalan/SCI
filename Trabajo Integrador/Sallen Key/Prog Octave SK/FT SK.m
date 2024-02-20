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
%  Raíces:
% -1.9697 + 9.8041i
% -1.9697 - 9.8041i

%O= 260.6/((s^2)+4*s+100)
%pole(O)
%  -2.0000 + 9.7980i
%  -2.0000 - 9.7980i


roots([1 4 100])
% -2 + 9.8041i
% -2 - 9.8041i

%[z,p,k]= zpkdata(FT)
rlocus(G); grid minor %Lugar de Raíces de la planta

%Aprox de FT
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
%  -2.103 + 10.134i
%  -2.103 - 10.134i

%PID
Td= 0.25
Ti= 0.04
PID= Td*(((s^2)+(1/(Td))*s+(1/(Ti*Td)))/(s))

rlocus(PID*G)
sgrid(0.707, 1), grid minor
%s1= -5   %K= 0.0769  aprox 0.077
%s1= -10  %K= 0.1541
s1= -2    %K= 0.030737
invK= abs((Td*(((s1^2)+(1/(Td))*s1+(1/(Ti*Td)))/(s1)))*(260.6/((s1^2)+3.939*s1+100)))
K2= 1/invK %K= 0.021551
FTT= feedback(K2*PID*G, 1)
pole(FTT)
step(FTT)
%ts= 17.4 seg NUEVO TIEMPO DE ESTABLECIMIENTO
FTLA= minreal(K2*PID*G)

%FTLA= 2.97*(((s^2)+(1/(Ti*Td))*s+(1/(Ti*Td)))/(s))*(260.6/((s^2)+3.939*s+100))


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ANALISIS DE ERROR DE PLANTA SIN PID %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Las constantes de error para las distintas entradas son:
roots([C1*C2*R1*R2 (R1*C1+R2*C2+R1*C2*(1-K))  1])

G1= 8600/(33*(s^2)+130*s+3300)
G1= 260.6/((s^2)+4*s+100)

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
% ANALISIS DE ERROR DE PLANTA CON CONTROLADOR PID %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

