function Q=CinematicaInversa3GDLart(T,codo)
%Codo: Indica que tupla de solución se va a trabajar
%Longitudes del brazo robot
L1=0.2;%medida en metros
L2=0.4;
L3=0.3;
%Parámetros de Denavit Hartenberg
d= [L1;0;0];
a=[0;L2;L3];
alfa=[pi/2;0;0];
%
P=T(1:3,4);
q1=0;
q2=0;
q3=0;
%Solución de la 1°articulacion
q1=atan(P(2)/P(1))
r=sqrt(P(1)^2+P(2)^2);
%---------------------------------------
%Solución de la 3° articulación
cosq3=(P(1)^2+P(2)^2+(P(3)-L1)^2-L2^2-L3^2)/(2*L3*L2);
%codo abajo
q3_1=atan(sqrt(1-cosq3^2)/cosq3);
%codo arriba.
q3_2=atan(-sqrt(1-cosq3^2)/cosq3);
%---------------------------------------------------------
%Solución de la 2° articulación´
psi_1=atan((P(3)-L1)/r)
psi_2=atan((P(3)-L1)/r)%DEBERÍA SER -r creo
%CODO abajo
phi_1=atan((L3*sin(q3_1))/(cos(q3_1)*L3+L2));
phi_2=atan((L3*sin(q3_2))/(cos(q3_2)*L3+L2));

q2_1=psi_1-phi_1;
q2_2=psi_2-phi_1;
q2_3=psi_1-phi_2;
q2_4=psi_2-phi_2;

if codo==1
%Resultado codo abajo
disp('Codo Abajo')
Q=[q1;q2_1;q3_1];
else
%Resultado codo Arriba
disp('Codo Arriba')
Q=[q1;q2_4;q3_2];
end
