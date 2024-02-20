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
