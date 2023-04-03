function Jvl=Jv_Scara(theta,d,a,alpha)

PDH=[theta d a alpha]
%...................................................
%....................................................
T01=denavit2021(theta(1),d(1),a(1),alpha(1));
T12=denavit2021(theta(2),d(2),a(2),alpha(2));
T23=denavit2021(theta(3),d(3),a(3),alpha(3));
T34=denavit2021(theta(4),d(4),a(4),alpha(4));
T04=simplify(T01*T12*T23*T34);

%...................................................
%Posicion
P04=T04(1:3,4);

%Jacobiano de velocidades angulares
%Jw=el *Z0 + e2*Z1 +e3*Z2...enZn-1 donde ei=1(articulación rotacional)
%ei=0 (articulación prismática)
%Método 1
T03=T01*T12*T23;
Z0=[0;0;1];
Z1=T01(1:3,3);
Z2=[0;0;0];
Z3=T03(1:3,3);
Jw=[Z0 Z1 Z2 Z3];

%---------------------------------
%Cálculo de Velocidades del Robot
%Cálculo del Jacobiano lineal
%Método diferencial.
Jvl=[diff(P04,q1) diff(P04,q2) diff(P04,q3) diff(P04,q4)];

end