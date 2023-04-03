function y = ri0nia(rii_1,ri_10ni_1a,ri_10fi_1a,ri0ni,ri0fi,ri_10pi,ri0pi,ri0si)
a = cross(ri_10pi,ri_10fi_1a);
b = cross((ri0pi+ri0si),ri0fi);
y = rii_1*(ri_10ni_1a + a) + b + ri0ni;