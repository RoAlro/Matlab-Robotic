%PARÁMETROS DH ROBOT HUMANOIDE
%MANO DERECHA

syms q1 q2 q3 q4 q5 q6 L2 L3
%L2=300;
%L3=320;
% parAmetros DH
theta = [q1-pi/2 q2+pi/2 q3+pi/2 q4 q5+pi/2 q6]
d = [0 0 0 0 0 0]
a = [0 0 L2 L3 0 0]
alfa = [pi/2 -pi/2 pi/2 0 -pi/2 0]
% Parte b)
T01_ = denavit(theta(1), d(1), a(1), alfa(1))
T12 = denavit(theta(2), d(2), a(2), alfa(2))
T23 = denavit(theta(3), d(3), a(3), alfa(3))
T34 = denavit(theta(4), d(4), a(4), alfa(4))
T45 = denavit(theta(5), d(5), a(5), alfa(5))
T56 = denavit(theta(6), d(6), a(6), alfa(6))
%Parte c)
T06=simplify(T01*T12*T23*T34*T45*56)
%...................................................
%Posicion
P06=T04(1:3,4)

%Calculando el Jacobiano....
T02=T01*T12
T03=T01*T12*T23
T04=T01*T12*T23*T34
T05=T01*T12*T23*T34*T45

Z0 = [0; 0; 1]
Z1 = T01(1:3, 3)
Z2 = T02(1:3, 3)
Z3 = T03(1:3, 3)
Z4 = T04(1:3, 3)
Z5 = T05(1:3, 3)
%e1=e2=e3=e4=s5=s6=e=1
e = 1;
% Jacobiano angular 
Jw = [Z0 Z1 Z2 Z3 Z4 Z5]
%Jacobiano de velocidad lineal
Jvl=jacobian(P06,[q1,q2,q3,q4,q5,q6])
J=[Jvl;Jw]





