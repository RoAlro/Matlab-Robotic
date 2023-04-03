%UNT
%INGENIERÍA MECATRÓNICA
%ROBÓTICA
%FECHA
%ELABORADO POR Acosta Baltodano Roger Aldo

function R=P_DH_pata_mod_ROT(theta,d,a,alpha)
%Parámetros DH
%Los datos ingresados estarán modificados para encontrar la posición
%del centro de masa de cada eslabón con respecto a la base
T01=denavit(theta(1),d(1),a(1),alpha(1));
T12=denavit(theta(2),d(2),a(2),alpha(2));
T23=denavit(theta(3),d(3),a(3),alpha(3));
%Este algoritmo es parecedio al creado anteriormente, solo que en este
%vamos a calcular la matriz de rotación con repecto a la base
T03=simplify(T01*T12*T23);

R=T03(1:3,1:3);