%Aplicacion particular
function T02=FuncCinematicaDirecta2GDL(q)
L1=0.6;%medida en metros
L2=0.4;
%Parámetros dDH
theta=[q(1);q(2)];
d=[0;0];
a=[L1;L2]
alfa=[0;0];

MDH=[theta d a alfa];
T01=denavit(theta(1),d(1),a(1),alfa(1));
T12=denavit(theta(2),d(2),a(2),alfa(2));
T02=(T01*T12);
end

