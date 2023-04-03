function Q=CinematicaInversaScara4GDL(T,codo)
%Codo: Indica que tupla de solución se va a trabajar
%Longitudes del brazo robot
L1=35;%medida en centimetros
L2=35;
L3=20;
L4=2.5;
%Parámetros DH

d=[L1;0;-q3;-L4];
a=[L2;L3;0;0];
alpha=[0;0;0;0];
PDH=[theta d a alpha]
%
P=T(1:3,4);
q1=0;
q2=0;
q3=0;
q4=0
%Solución de la 2°articulacion
q2_1=atan(sqrt(1-cosq2^2)/cosq2);
q2_2=atan(-sqrt(1-cosq2^2)/cosq2);
%---------------------------------------
%Solución de la 1° articulación
%codo abajo
q1_1=atan(P(2)/P(1))-atan((L2*sin(q2_1))/(L1+L2*cos(q2_1)))
%cpdo arriba
q1_2=atan(P(2)/P(1))-atan((L2*sin(q2_2))/(L1+L2*cos(q2_2)))

%Solución 3°articulación
q3=L1-L4-P(3)

% 
% q4_1=atan((sin(q1_1+q2_1+q4_1)))-q1_1-q2_1
% q4_2=atan((sin(q1_2+q2_2+q4)))

if codo==1
%Resultado codo abajo
disp('Codo Abajo')
q4_1=0;

while true
    
a=atan((sin(q1_1+q2_1+q4_1)))-q1_1-q2_1-q4_1;
q4_1=q4_1+0.001
if a<=0.0005
    break
end
end
Q=[q1_1;q2_1;q3;q4_1];
else
q4_2=0;

%Resultado codo Arriba
disp('Codo Arriba')
while true
    
a=atan((sin(q1_2+q2_2+q4_2)))-q1_2-q2_2-q4_2;
q4_2=q4_2+0.001
if a<=0.0005
    break
end
Q=[q1_2;q2_1;q3;q4_2];
end
end