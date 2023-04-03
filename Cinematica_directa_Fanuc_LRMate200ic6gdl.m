%Cinemática direcata de robot INDUSTRIAL Fanuc LRMate200ic 6GDL
%UNT
%INGENIERÍA MECATRÓNICA
%ROBÓTICA
%FECHA
%ELABORADO POR
%-----------------------------------------
clc,clear all
syms L1 L2 L3 L4 L5 L6 L7 q1 q2 q3 q4 q5 q6 pi

%Parámetros DH
theta=[q1;q2+pi/2;q3;q4;q5;q6];
d=[L2;0;0;L6;0;L7];
a=[L5;L3;L4;0;0;0];
alpha=[pi/2;0;pi/2;-pi/2;pi/2;0];
PDH=[theta d a alpha]
%...................................................
%....................................................
T01=denavit2021(theta(1),d(1),a(1),alpha(1))
T12=denavit2021(theta(2),d(2),a(2),alpha(2))
T23=denavit2021(theta(3),d(3),a(3),alpha(3))
T34=denavit2021(theta(4),d(4),a(4),alpha(4))
T45=denavit2021(theta(5),d(5),a(5),alpha(5))
T56=denavit2021(theta(6),d(6),a(6),alpha(6))
T06=simplify(T01*T12*T23*T34*T45*56)

%...................................................
%Posicion
P=T06(1:3,4)