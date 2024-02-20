clear all; close all; clc
pkg load symbolic
syms s C1 C2 R1 R2 real

C1= 0.00001
C2= 0.00002
R1= 375
R2= 375

G= 1/((C1*C2*R1*R2*(s^2))+((R1+R2)*C1*s)+1)
pkg load control
s= tf('s')
G= 1/((0.00001*0.00002*375*375*(s^2))+(375+375)*0.00001*s+1)
step(G)

%Sallen key ejemplo video
%https://www.youtube.com/watch?v=ZFrK0BHkejs
%G1= 1.586/((s^2)+1.4142*s+1)
G1= 1.586/((s^2)+1.243*s+1)
step(G1)
