clc,clear all
%ITEM O)
%----------------------------------------------
P=[16 22 9]
L1=35;%medida en centimetros
L2=35;
L3=20;
L4=2.5;
q1=0;
q2=0;
q3=0;
q4=0;
%Solución de la 2°articulacion
cosq2=(P(1)^2+P(2)^2-L1^2-L2^2)/(2*L1*L2);
q2_1=atan(sqrt(1-cosq2^2)/cosq2)
q2_2=atan(-sqrt(1-cosq2^2)/cosq2)
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

%Resultado codo abajo
disp('Codo Abajo')

q4_1=7;%2*pi =6.2831
a=0;
while true 
    
a=atan((sin(q1_1+q2_1+q4_1))/(cos(q1_1+q2_1+q4_1)))-q1_1-q2_1-q4_1;
q4_1=q4_1-0.0001;
if abs(a)<=0.0001
    break
end
end
Q=[q1_1;q2_1;q3;q4_1]

%Codo arriba
disp('Codo Arriba')

q4_2=7;%2*pi =6.2831
a=0;
while true 
    
a=atan((sin(q1_2+q2_2+q4_2))/(cos(q1_2+q2_2+q4_2)))-q1_2-q2_2-q4_2;
q4_2=q4_2-0.0001;
if abs(a)<=0.0001
    break
end
end
Q=[q1_2;q2_2;q3;q4_2]