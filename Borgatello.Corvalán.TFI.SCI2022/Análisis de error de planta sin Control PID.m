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

