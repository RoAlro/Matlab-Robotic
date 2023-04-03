%JACOBIANO SCARA 4GDL

clc,clear all
syms L1 L2 L3 L4 q1 q2 q3 q4 pi 
%Parámetros DH
theta=[q1+pi/2;q2;0;q4];
d=[L1;0;-q3;-L4];
a=[L2;L3;0;0];
alpha=[0;0;0;0];
PDH=[theta d a alpha]
%...................................................
%....................................................
T01=denavit2021(theta(1),d(1),a(1),alpha(1));
T12=denavit2021(theta(2),d(2),a(2),alpha(2));
T23=denavit2021(theta(3),d(3),a(3),alpha(3));
T34=denavit2021(theta(4),d(4),a(4),alpha(4));
T04=simplify(T01*T12*T23*T34)

%...................................................
%Posicion
P04=T04(1:3,4)

%Jacobiano de velocidades angulares
%Jw=el *Z0 + e2*Z1 +e3*Z2...enZn-1 donde ei=1(articulación rotacional)
%ei=0 (articulación prismática)
%Método 1
T03=T01*T12*T23
Z0=[0;0;1]
Z1=T01(1:3,3)
Z2=[0;0;0]
Z3=T03(1:3,3)
Jw=[Z0 Z1 Z2 Z3]

%---------------------------------
%Cálculo de Velocidades del Robot
%Cálculo del Jacobiano lineal
%Método diferencial.
Jvl=[diff(P04,q1) diff(P04,q2) diff(P04,q3) diff(P04,q4)]
%Jv1=jacobian(P04,[q1,q2,q3,q4])
%LA MATRIZ JACOBIANA
J=[Jvl;Jw]
%---------------------------------------------------------------------
%CACLULANDO LA VELOCIDAD LINEAL RESPECTO DEL REFERENCIAL BASE
qpunto=[pi/8;-2*pi/3;0.05;-pi/4];
Vl=Jvl*qpunto
%los valores se reemplazan manualmente  

%Singularidades

A=J(1:3,1:4);
B=[A;J(6,1:4)];
det(B)

%Determinando la aceleración:
q=[q1;q2;q3;q4]
qpunto=[q1_p;q2_p;q3_p;q4_p']
qdoblepunto=[q1_pp;q2_pp;q3_pp;q4_pp]
Jderiv=[diff(J,q1) diff(J,q2) diff(J,q3) diff(J,q4)]
a=Jderiv*qpunto + J*qdoblepunto

%Determinando qpunto para P=[2;-1;5] cm/s
%y para q=[pi/3;-pi/4;0.15;pi/10]

% P=[2;-1;5]
% Jvl_inv=(inv(transpose(Jvl)*Jvl))*transpose(Jvl)
% Jvl_inv2 = pinv(Jvl)
% qpunto=Jvl_inv2*P

% L1=350 % medida de mm
% L2=350
%  L3=200
%  L4=25
%  
%  q1=pi/3
%  q2=pi/6
%  q3=0.2
%  q4=-pi/8

% Vl=Jvl*qpunto