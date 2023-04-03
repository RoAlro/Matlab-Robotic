%Cinemática direcata de robot SCARA 4GDL
%UNT
%INGENIERÍA MECATRÓNICA
%ROBÓTICA
%FECHA
%ELABORADO POR
%-----------------------------------------
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
T01=denavit2021(theta(1),d(1),a(1),alpha(1))
T12=denavit2021(theta(2),d(2),a(2),alpha(2))
T23=denavit2021(theta(3),d(3),a(3),alpha(3))
T34=denavit2021(theta(4),d(4),a(4),alpha(4))
T04=simplify(T01*T12*T23*T34)

%...................................................
%Posicion
P=T04(1:3,4)