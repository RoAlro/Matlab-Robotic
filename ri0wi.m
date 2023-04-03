%VECTOR DE VELOCIDAD ANGULAR
%RI0WI Vector ri0wi
function y = ri0wi(iri_1,i_1r0wi_1,qpi)
z = [0;0;1];
y = iri_1*(i_1r0wi_1+z*qpi);