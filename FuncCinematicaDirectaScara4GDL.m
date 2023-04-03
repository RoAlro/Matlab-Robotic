%Aplicacion particular
function T04=FuncCinematicaDirectaScara4GDL(q)
%Longitudes del brazo robot
L1=35;%medida en centimetros
L2=35;
L3=20;
L4=2.5;
%Parámetros DH
theta=[q(1)+pi/2;q(2);0;q(4)];
d=[L1;0;-q(3);-L4];
a=[L2;L3;0;0];
alpha=[0;0;0;0];
PDH=[theta d a alpha]
%...................................................
%....................................................
T01=denavit2021(theta(1),d(1),a(1),alpha(1));
T12=denavit2021(theta(2),d(2),a(2),alpha(2));
T23=denavit2021(theta(3),d(3),a(3),alpha(3));
T34=denavit2021(theta(4),d(4),a(4),alpha(4));
T04=(T01*T12*T23*T34)

%...................................................
%Posicion

end