
%Vector ri0ai
function y = ri0ai(ir0vpi,ir0wpi,ir0wi,ir0si)
a = cross(ir0wi,ir0si);
b = cross(ir0wi,a);
y = cross(ir0wpi,ir0si)+b+ir0vpi;