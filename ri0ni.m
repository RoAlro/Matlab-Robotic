%MOMENTOS EJERCIDOS SOBRE EL CENTRO DE MASA
%RI0NI Vector ri0ni
function y = ri0ni(ir0wpi,ir0wi,ir0I_0ri)
y = ir0I_0ri*ir0wpi+cross(ir0wi,(ir0I_0ri*ir0wi));