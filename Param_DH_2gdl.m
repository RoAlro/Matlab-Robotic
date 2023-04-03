%Parámetros D-H para 2GDL
clc,clear all
syms L1 L2 q1 q2

%Parámetros DH-Cinemática directa.
theta=[q1;q2];
d=[0;0];
a=[L1;L2];
alpha=[0;0];
PDH=[theta d a alpha];
%....................................................
%.....................................................
T01=denavit(theta(1),d(1),a(1),alpha(1))
T12=denavit(theta(2),d(2),a(2),alpha(2))
T02=simplify(T01*T12)
%....................................................
%Posición
P=T02(1:3,4)

