%VECTOR ACELERACIÓN LINEAL
%RI0VPI_R Vector ri0vpi_r
function y = ri0vpi_r(iri_1,i_1r0vpi_1,ir0wpi,ir0wi,ir0pi)
z = [0;0;1];
a = cross(ir0wpi,ir0pi);
b = cross(ir0wi,ir0pi);
c = cross(ir0wi,b);
y = (a+c+iri_1*i_1r0vpi_1);