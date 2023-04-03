clc,clear all
syms L1 L2 L3 q1 q2 q3 Lc1 Lc2 Lc3 pi
%DATOS
L1=250;%mm
L2=200;%mm
L3=300;%mm
Lc1=125;%mm
Lc2=100;%mm
Lc3=150;%mm

m1=1.6;%Kg
m2=1.5;%Kg
m3=1.7;%Kg
%-----------------------------------------
%Posición del centro de masa 1
%Parámetros DH 1
theta1=[q1;0;0];
d1=[0;0;0];
a1=[Lc1;0;0];
alpha1=[pi/2;0;0];

P0c1=P_DH_pata_mod(theta1,d1,a1,alpha1)

%Posición del centro de masa 2
%Parámetros DH 2
theta2=[q1;q2;0];
d2=[0;0;0];
a2=[L1;Lc2;0];
alpha2=[pi/2;0;0];

P0c2=P_DH_pata_mod(theta2,d2,a2,alpha2)

%Posición del centro de masa 3
%Parámetros DH 3
theta3=[q1;q2;q3-pi/2];
d3=[0;0;0];
a3=[L1;L2;Lc3];
alpha3=[pi/2;0;0];

P0c3=P_DH_pata_mod(theta3,d3,a3,alpha3)

%Parámetros DH para la pata

theta=[q1;q2;q3-pi/2];
d=[0;0;0];
a=[L1;L2;L3];
alpha=[pi/2;0;0];
PDH=[theta d a alpha];
%...................................................
%....................................................
T01=denavit(theta(1),d(1),a(1),alpha(1));
T12=denavit(theta(2),d(2),a(2),alpha(2));
T23=denavit(theta(3),d(3),a(3),alpha(3));

T03=simplify(T01*T12*T23);
T02=simplify(T01*T12);
%...................................................
%Posicion
P03=T03(1:3,4)

%Jacobiano de velocidad lineal del centro de masa
Jv1=[diff(P0c1(1),q1) 0 0; diff(P0c1(2),q1) 0 0; diff(P0c1(3),q1) 0 0]  %Eslabon 1
Jv2=[diff(P0c2(1),q1) diff(P0c2(1),q2) 0; diff(P0c2(2),q1) diff(P0c2(2),q2) 0; diff(P0c2(3),q1) diff(P0c2(2),q2) 0]  %Eslabon 2
Jv3=[diff(P0c3(1),q1) diff(P0c3(1),q2) diff(P0c3(1),q3); diff(P0c3(2),q1) diff(P0c3(2),q2) diff(P0c3(2),q3); diff(P0c3(3),q1) diff(P0c3(2),q2) diff(P0c3(3),q3)]  %Eslabon 3


Z0=[0;0;1];
ceros=[0;0;0];
Z1=T01(1:3,3);
Z2=T02(1:3,3);
Z3=T03(1:3,3);
%Jacobianos de velocidades angulares
%Como todos son revoluta e=1
Jw1=[Z1 ceros ceros]
Jw2=[Z1 Z2 ceros]
Jw3=[Z1 Z2 Z3]

%Matrices de Rotacion
R01=T01(1:3,1:3);

R12=T12(1:3,1:3);
R23=T23(1:3,1:3);

R02=simplify(R01*R12);
R03=simplify(R01*R12*R23);

%Se declara las matrices Inerciales

%Suposiciones de ancho(b) y altura (h), para todos los eslabones
b=20;%mm
h=20;%mm

Ic1=[(1/12)*m1*(b^2+h^2) 0 0; 0 (1/12)*m1*(b^2+L1^2) 0; 0 0 (1/12)*m1*(h^2+L1^2)]
Ic2=[(1/12)*m2*(b^2+h^2) 0 0; 0 (1/12)*m2*(b^2+L2^2) 0; 0 0 (1/12)*m2*(h^2+L2^2)]
Ic3=[(1/12)*m3*(b^2+h^2) 0 0; 0 (1/12)*m3*(b^2+L3^2) 0; 0 0 (1/12)*m3*(h^2+L3^2)]

%Referenciando con respecto al sistema 0:
%****Matrices de Rotación para los centros de masa
% R0c1=R01;
% R0c2=R02;
% R0c3=R03;
%******


%Paso 1
%Se calcula la matriz de Inercia del robot

D=simplify(m1*(Jv1.')*Jv1+(Jw1.')*R01*Ic1*(R01.')*Jw1+ m2*(Jv2.')*Jv2+(Jw2.')*R02*Ic2*(R02.')*Jw2+ m3*(Jv3.')*Jv3+(Jw3.')*R03*Ic3*(R03.')*Jw3)

%Paso 2
%Matrices de Coriolisis y Centrifuga
syms dq1 dq2 dq3 ddq1 ddq2 ddq3

C111 = (diff(D(1,1),q1)+diff(D(1,1),q1)-diff(D(1,1),q1))/2;
C112 = (diff(D(1,1),q2)+diff(D(1,2),q1)-diff(D(1,2),q1))/2;
C113 = (diff(D(1,1),q3)+diff(D(1,3),q1)-diff(D(1,3),q1))/2;
C121 = (diff(D(1,2),q1)+diff(D(1,1),q2)-diff(D(2,1),q1))/2;
C122 = (diff(D(1,2),q2)+diff(D(1,2),q2)-diff(D(2,2),q1))/2;
C123 = (diff(D(1,2),q3)+diff(D(1,3),q2)-diff(D(2,3),q1))/2;
C131 = (diff(D(1,3),q1)+diff(D(1,1),q3)-diff(D(3,1),q1))/2;
C132 = (diff(D(1,3),q2)+diff(D(1,2),q3)-diff(D(3,2),q1))/2;
C133 = (diff(D(1,3),q3)+diff(D(1,3),q3)-diff(D(3,3),q1))/2;

C211 = (diff(D(2,1),q1)+diff(D(2,1),q1)-diff(D(1,1),q2))/2;
C212 = (diff(D(2,1),q2)+diff(D(2,2),q1)-diff(D(1,2),q2))/2;
C213 = (diff(D(2,1),q3)+diff(D(2,3),q1)-diff(D(1,3),q2))/2;
C221 = (diff(D(2,2),q1)+diff(D(2,1),q2)-diff(D(2,1),q2))/2;
C222 = (diff(D(2,2),q2)+diff(D(2,2),q2)-diff(D(2,2),q2))/2;
C223 = (diff(D(2,2),q3)+diff(D(2,3),q2)-diff(D(2,3),q2))/2;
C231 = (diff(D(2,3),q1)+diff(D(2,1),q3)-diff(D(3,1),q2))/2;
C232 = (diff(D(2,3),q2)+diff(D(2,2),q3)-diff(D(3,2),q2))/2;
C233 = (diff(D(2,3),q3)+diff(D(2,3),q3)-diff(D(3,3),q2))/2;

C311 = (diff(D(3,1),q1)+diff(D(3,1),q1)-diff(D(1,1),q3))/2;
C312 = (diff(D(3,1),q2)+diff(D(3,2),q1)-diff(D(1,2),q3))/2;
C313 = (diff(D(3,1),q3)+diff(D(3,3),q1)-diff(D(1,3),q3))/2;
C321 = (diff(D(3,2),q1)+diff(D(3,1),q2)-diff(D(2,1),q3))/2;
C322 = (diff(D(3,2),q2)+diff(D(3,2),q2)-diff(D(2,2),q3))/2;
C323 = (diff(D(3,2),q3)+diff(D(3,3),q2)-diff(D(2,3),q3))/2;
C331 = (diff(D(3,3),q1)+diff(D(3,1),q3)-diff(D(3,1),q3))/2;
C332 = (diff(D(3,3),q2)+diff(D(3,2),q3)-diff(D(3,2),q3))/2;
C333 = (diff(D(3,3),q3)+diff(D(3,3),q3)-diff(D(3,3),q3))/2;

C11 = C111*dq1+C112*dq2+C113*dq3;
C12 = C121*dq1+C122*dq2+C123*dq3;
C13 = C131*dq1+C132*dq2+C133*dq3;
C21 = C211*dq1+C212*dq2+C213*dq3;
C22 = C221*dq1+C222*dq2+C223*dq3;
C23 = C231*dq1+C232*dq2+C233*dq3;
C31 = C311*dq1+C312*dq2+C313*dq3;
C32 = C321*dq1+C322*dq2+C323*dq3;
C33 = C331*dq1+C332*dq2+C333*dq3;

C = simplify([C11 C12 C13; C21 C22 C23; C31 C32 C33])


%Paso 3
%Vector Gravedad
g=9.81;%m/s^2
g0 = [0; 0; -g];
Jv1T = Jv1.';
Jv2T = Jv2.';
Jv3T = Jv3.';
G1 = simplify(-(Jv1T(1,1:3))*m1*g0-(Jv2T(1,1:3))*m2*g0-(Jv3T(1,1:3))*m3*g0);
G2 = simplify(-(Jv1T(2,1:3))*m1*g0-(Jv2T(2,1:3))*m2*g0-(Jv3T(2,1:3))*m3*g0);
G3 = simplify(-(Jv1T(3,1:3))*m1*g0-(Jv2T(3,1:3))*m2*g0-(Jv3T(3,1:3))*m3*g0);
G = [G1; G2; G3]


%Ecuación dinámica del robot
tau = D*[ddq1;ddq2;ddq3]+C*[dq1;dq2;dq3]+G

