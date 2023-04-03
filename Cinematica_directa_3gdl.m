%Cinemática direcata de robot 3GDL
%Esférico 2revolutas 1prismático 
%UNT
%INGENIERÍA MECATRÓNICA
%ROBÓTICA
%FECHA
%ELABORADO POR
%-----------------------------------------
clc,clear all
syms L1 L2 q1 q2 q3 pi

%Parámetros DH
theta=[q1;q2+pi/2;0];
d=[L1;0;q3];
a=[0;0;0];
alpha=[pi/2;pi/2;0];
PDH=[theta d a alpha]
%...................................................
%....................................................
T01=denavit2021(theta(1),d(1),a(1),alpha(1))
T12=denavit2021(theta(2),d(2),a(2),alpha(2))
T23=denavit2021(theta(3),d(3),a(3),alpha(3))

T03=simplify(T01*T12*T23)

%...................................................
%Posicion
P=T03(1:3,4)