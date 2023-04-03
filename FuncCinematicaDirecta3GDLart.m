%Aplicacion particular
function T03=FuncCinematicaDirecta3GDLart(q)
%Longitudes del brazo robot
L1=0.2;%medida en metros
L2=0.4;
L3=0.3;
%Parámetros DH
theta=[q(1);q(2);q(3)];
d=[L1;0;0];
a=[0;L2;L3];
alpha=[pi/2;0;0];
PDH=[theta d a alpha]
%...................................................
%....................................................
T01=denavit2021(theta(1),d(1),a(1),alpha(1));
T12=denavit2021(theta(2),d(2),a(2),alpha(2));
T23=denavit2021(theta(3),d(3),a(3),alpha(3));

T03=(T01*T12*T23);
end