%-----------------------------------------
clc,clear all

%Declaración de variables
syms q1 q2 q3 q4 q5 q6 pi m1 m2 m3 m6 L1 L2 L4 Fx Fy Fz g nx ny nz Fx Fy Fz g nx ny nz

%Parámetros de DH
theta = [q1;q2;0;q4;q5;q6];
d = [L1;L2;q3;0;0;L4];
a = [0;0;0;0;0;0];
alpha = [-pi/2;0;0;pi/2;-pi/2;0];
PDH=[theta d a alpha];

%Matriz de transformación homogénea entre sistemas de coordenadas
T01 = denavit(theta(1),d(1),a(1),alpha(1))
T12 = denavit(theta(2),d(2),a(2),alpha(2))
T23 = denavit(theta(3),d(3),a(3),alpha(3))
T34 = denavit(theta(4),d(4),a(4),alpha(4))
T45 = denavit(theta(5),d(5),a(5),alpha(5))
T56 = denavit(theta(6),d(6),a(6),alpha(6))
T02 = simplify(T01*T12)
T03 = simplify(T01*T12*T23)
T04 = simplify(T01*T12*T23*T34)
T05 = simplify(T01*T12*T23*T34*T45)
T06 = simplify(T01*T12*T23*T34*T45*T56)

%Posición
P06 = T06(1:3,4)


%Matrices de rotación
R01 = T01(1:3,1:3);
R10 = R01.';
R12 = T12(1:3,1:3); 
R23 = T23(1:3,1:3);
R34 = T34(1:3,1:3);
R45 = T45(1:3,1:3);
R56 = T56(1:3,1:3);

R02 = T02(1:3,1:3);
R20 = R02.';
R03 = T03(1:3,1:3);
R30 = R03.';
R04 = T04(1:3,1:3);
R40 = R04.';
R05 = T05(1:3,1:3);
R50 = R05.';
R06 = T06(1:3,1:3);

%%ESTÁTICA RECURSIVA
%Valores de entrada en efector final
% nx = 0;
% ny = 0;
% nz = 0;
% g=0;
% Fz=0;

F67 = [Fx;Fy;Fz];
M67 = [nx;ny;nz];  

%Gravedad
%g=9.81 m/s^2
G = [0;0;-g];

%Centros de masa en los puntos medios
r1_01=[0;0;L1]
r2_12=[0;0;L2]
r3_23=[0;0;q3]
r4_34=[0;0;0]
r5_45=[0;0;0]
r6_56=[0;0;L4]
%Cetnros de masa
r1_0c1=r1_01/2
r2_1c2=r1_01/2
r3_2c3=r1_01/2
r4_3c4=r1_01/2
r5_4c5=r1_01/2
r6_5c6=r1_01/2

%Cálculo de las fuerzas
m4=0;
m5=0;

F56 = R56*F67-m6*R50*G
F45 = R45*F56-m5*R40*G
F34 = R34*F45-m4*30*G
F23 = simplify(R23*F34-m3*R20*G)
F12 = simplify(R12*F23-m2*R10*G)
F01 = simplify(R01*F12-m1*G)

%Calculo de Pares
M56=simplify(R56*M67-cross(R56*r6_5c6,m6*R50*G)+cross(R56*r6_56,R56*F67))
M45=simplify(R45*M56-cross(R45*r5_4c5,m5*R50*G)+cross(R56*r5_45,R56*F67))
M34=simplify(R56*M67-cross(R56*r4_3c4,m4*R50*G)+cross(R56*r4_34,R56*F67))
M23=simplify(R23*M34-cross(R23*r3_2c3,m3*R20*G)+cross(R23*r3_23,R23*F34))
M12=simplify(R12*M23-cross(R12*r2_1c2,m2*R10*G)+cross(R12*r2_12,R12*F23))
M01=simplify(R01*M12-cross(R01*r1_0c1,m1*G)+cross(R01*r1_01,R01*F12))

%Tau final
tau = [M01;M12;M23;M34;M45;M56]

%%TRABAJO VIRTUAL

Jv = jacobian(P06,[q1,q2,q3,q4,q5,q6])
par = simplify(Jv.'*R06*F67)