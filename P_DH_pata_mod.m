

%UNT
%INGENIERÍA MECATRÓNICA
%ROBÓTICA
%FECHA
%ELABORADO POR Acosta Baltodano Roger Aldo

function P=P_DH_pata_mod(theta,d,a,alpha)
%Parámetros DH
%Los datos ingresados estarán modificados para encontrar la posición
%del centro de masa de cada eslabón con respecto a la base
T01=denavit(theta(1),d(1),a(1),alpha(1));
T12=denavit(theta(2),d(2),a(2),alpha(2));
T23=denavit(theta(3),d(3),a(3),alpha(3));
%Cuando se le pasa valores nulos a la función "denavit", esta arroja
%una matriz identidad, por tanto podemos utilizar esto para usar la posición
%P03 como P01 cuando (theta(2),d(2),a(2),alpha(2) y (theta(3),d(3),a(3),alpha(3))
%sean igual a cero, de manera similar para P02 cuando 
%(theta(3),d(3),a(3),alpha(3)) sea igual a 0.
T03=simplify(T01*T12*T23);

P=T03(1:3,4);