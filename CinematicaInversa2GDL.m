function Q=CinematicaInversa2GDL(T,codo)
%Codo: Indica que tupla de soluci�n se va a trabajar
%Longitudes del brazo robot
L1=0.6;%medida en metros
L2=0.4;
%Par�metros de Denavit Hartenberg
d= [0;0];
a=[L1;L2];
alfa=[0;0];
%Posici�n del Efector Final del Manipulador
P=T(1:3,4);
q1=0;
q2=0;
%Soluci�n de la 2�articulacion
cosq2=(P(1)^2+P(2)^2-L1^2-L2^2)/(2*L1*L2);
%codo abajo
q2_1=atan(sqrt(1-cosq2^2)/cosq2);
%codo arriba
q2_2=atan(-sqrt(1-cosq2^2)/cosq2);
%---------------------------------------
%Soluci�n de la 1� articulaci�n
psi=atan(P(2)/P(1));
%codo abajo
phi_1=atan((L2*sin(q2_1))/(cos(q2_1)*L2+L1));
%codo arriba.
phi_2=atan((L2*sin(q2_2))/(cos(q2_2)*L2+L1));

q1_1=psi-phi_1;
q1_2=psi-phi_2;

if codo==1
%Resultado codo abajo
disp('Codo Abajo')
Q=[q1_1;q2_1];
else
%Resultado codo Arriba
disp('Codo Arriba')
Q=[q1_2;q2_2];
end

