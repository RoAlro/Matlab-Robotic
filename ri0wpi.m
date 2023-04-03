%VECTOR DE ACELERACIÓN ANGULAR
%RI0WPI Vector ri0wpi
function y = ri0wpi(iri_1,i_1r0wpi_1,i_1r0wi_1,qpi,qppi)
z = [0;0;1];
a = cross(i_1r0wi_1,z*qpi);
y = iri_1*(i_1r0wpi_1+z*qppi+a);