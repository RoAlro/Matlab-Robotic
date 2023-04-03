
%Newton-Euler-5GDL
clc, clear all
syms pi q1 q2 q3 q4 q5 q1p q2p q3p q4p q5p q1pp q2pp q3pp q4pp q5pp m1 m2 m3 m4 m5 b1 b2 b3 b4 b5 I1 I2 I3 I4 I5 Iext g

%DATOS
d1 = 20%mm;
L1 = 16.4%mm;
L2 = 104;
L3 = 94.5;
d4 = 21.9;
L4 = 26;
L5 = 103.4;

%g = 9.81;
Iext = [0 0 0; 0 0 0; 0 0 0];

%PARÁMETROS DE DENAVIT - HARTENBERG
theta = [q1;q2+pi/2;q3;q4;q5];
d = [L1;0;0;-d4;L5];
a = [d1;L2;L3;L4;0];
alfa = [pi/2;0;0;pi/2;0];
MDH = [theta d a alfa]

%Masas : m1,m2,m3,m4,m5
%Coeficiente viscosos: b1,b2,b3,b4,b5
%Momentos de inercia de cada articulacion:I1,I2,I3,I4,I5
%Matrices de Inercia del robot:
r10I_r01 = I1;
r20I_r02 = I2;
r30I_r03 = I3;
r40I_r04 = I4;
r50I_r05 = I5;

%VECTORES ri0pi, ri0si
r10p1 = ri0pi(a(1),d(1),alfa(1))
r20p2 = ri0pi(a(2),d(2),alfa(2))
r30p3 = ri0pi(a(3),d(3),alfa(3))
r40p4 = ri0pi(a(4),d(4),alfa(4))
r50p5 = ri0pi(a(5),d(5),alfa(5))
r60p6 = zeros(3,1);
%POSICION DEL CENTRO DE MASA DE CADA ESLABON
r10s1 = [-d1/2;-L1/2;0]
r20s2 = [-L2/2;0;0]
r30s3 = [-L3/2;0;0]
r40s4 = [-L4/2;d4/2;0]
r50s5 = [0;0;-L5/2]
r60s6 = zeros(3,1);

%MATRICES DE TRANSFORMACIÓN
r01 = denavit(theta(1),d(1),a(1),alfa(1));
r01 = r01(1:3,1:3)
r10 = r01.'
r12 = denavit(theta(2),d(2),a(2),alfa(2));
r12 = r12(1:3,1:3)
r21 = r12.'
r23 = denavit(theta(3),d(3),a(3),alfa(3));
r23 = r23(1:3,1:3)
r32 = r23.'
r34 = denavit(theta(4),d(4),a(4),alfa(4));
r34 = r34(1:3,1:3)
r43 = r34.'
r45 = denavit(theta(5),d(5),a(5),alfa(5));
r45 = r45(1:3,1:3)
r54 = r45.'
r56 = eye(3);
r65 = r56.'

%VELOCIDAD ANGULAR
r00w0 = zeros(3,1); %Condición inicial de velocidad angular
r10w1 = ri0wi(r10,r00w0,q1p) %Vel angular del eslabón 1
r20w2 = ri0wi(r21,r10w1,q2p) %Velangular del eslabón 2
r30w3 = ri0wi(r32,r20w2,q3p) %Velangular del eslabón 3
r40w4 = ri0wi(r43,r30w3,q4p) %Velangular del eslabón 4
r50w5 = ri0wi(r54,r40w4,q5p) %Velangular del eslabón 5
r60w6 = ri0wi(r65,r50w5,0)

%ACELERACIÓN ANGULAR
r00wp0 = zeros(3,1); %Condición incial de la aceleración angular
r10wp1 = ri0wpi(r10,r00wp0,r00w0,q1p,q1pp)
r20wp2 = ri0wpi(r21,r10wp1,r10w1,q2p,q2pp)
r30wp3 = ri0wpi(r32,r20wp2,r20w2,q3p,q3pp)
r40wp4 = ri0wpi(r43,r30wp3,r30w3,q4p,q4pp)
r50wp5 = ri0wpi(r54,r40wp4,r40w4,q5p,q5pp)
r60wp6 = ri0wpi(r65,r50wp5,r50w5,0,0)

%ACELERACIÓN LINEAL ARTICULAR
r00vp0 = [0;0;-g] %Condición incial de la aceleración inicial
r10vp1 = simplify(ri0vpi_r(r10,r00vp0,r10wp1,r10w1,r10p1))
r20vp2 = simplify(ri0vpi_r(r21,r10vp1,r20wp2,r20w2,r20p2))
r30vp3 = simplify(ri0vpi_r(r32,r20vp2,r30wp3,r30w3,r30p3))
r40vp4 = simplify(ri0vpi_r(r43,r30vp3,r40wp4,r40w4,r40p4))
r50vp5 = simplify(ri0vpi_r(r54,r40vp4,r50wp5,r50w5,r50p5))
r60vp6 = simplify(ri0vpi_r(r65,r50vp5,r60wp6,r60w6,r60p6))

%ACELERACIÓN DEL CENTRO DE MASA DE CADA ELEMENTO
r10a1 = simplify(ri0ai(r10vp1,r10wp1,r10w1,r10s1))
r20a2 = simplify(ri0ai(r20vp2,r20wp2,r20w2,r20s2))
r30a3 = simplify(ri0ai(r30vp3,r30wp3,r30w3,r30s3))
r40a4 = simplify(ri0ai(r40vp4,r40wp4,r40w4,r40s4))
r50a5 = simplify(ri0ai(r50vp5,r50wp5,r50w5,r50s5))
r60a6 = simplify(ri0ai(r60vp6,r60wp6,r60w6,r60s6))

%FUERZA EN EL CENTRO DE MASA DE CADA ELEMENTO
r60f6 = ri0fi(r60a6,0)
r50f5 = ri0fi(r50a5,m5)
r40f4 = ri0fi(r40a4,m4)
r30f3 = ri0fi(r30a3,m3)
r20f2 = ri0fi(r20a2,m2)
r10f1 = ri0fi(r10a1,m1)

%PAR EN EL CENTRO DE MASA DE CADA ELEMENTO
r60n6 = ri0ni(r60wp6,r60w6,Iext)
r50n5 = ri0ni(r50wp5,r50w5,I5)
r40n4 = ri0ni(r40wp4,r40w4,I4)
r30n3 = ri0ni(r30wp3,r30w3,I3)
r20n2 = ri0ni(r20wp2,r20w2,I2)
r10n1 = ri0ni(r10wp1,r10w1,I1)

%FUERZAS ARTICULARES
r60f6a = r60f6
r50f5a = simplify(ri0fia(r56,r60f6a,r50f5))
r40f4a = simplify(ri0fia(r45,r50f5a,r40f4))
r30f3a = simplify(ri0fia(r34,r40f4a,r30f3))
r20f2a = simplify(ri0fia(r23,r30f3a,r20f2))
r10f1a = simplify(ri0fia(r12,r20f2a,r10f1))

%PARES ARTICULARES
r20p1 = r21*(r10p1);
r30p2 = r32*(r20p2);
r40p3 = r43*(r30p3);
r50p4 = r54*(r40p4);
r60p5 = r65*(r50p5);

r60n6a = r60n6
r50n5a = simplify(ri0nia(r56,r60n6a,r60f6a,r50n5,r50f5,r60p5,r50p5,r50s5))
r40n4a = simplify(ri0nia(r45,r50n5a,r50f5a,r40n4,r40f4,r50p4,r40p4,r40s4))
r30n3a = simplify(ri0nia(r34,r40n4a,r40f4a,r30n3,r30f3,r40p3,r30p3,r30s3))
r20n2a = simplify(ri0nia(r23,r30n3a,r30f3a,r20n2,r20f2,r30p2,r20p2,r20s2))
r10n1a = simplify(ri0nia(r12,r20n2a,r20f2a,r10n1,r10f1,r20p1,r10p1,r10s1))

%FUERZAS Y PARES DE ACCIONAMIENTO DE LOS ACTUADORES
t_1 = simplify(t_r(r10,r10n1a,q1p,b1))
t_2 = simplify(t_r(r21,r20n2a,q2p,b2))
t_3 = simplify(t_r(r32,r30n3a,q3p,b3))
t_4 = simplify(t_r(r43,r40n4a,q4p,b4))
t_5 = simplify(t_r(r54,r50n5a,q5p,b5))
tau = [t_1;t_2;t_3;t_4;t_5]