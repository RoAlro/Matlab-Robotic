function T=denavit2021(theta,d,a,alpha)
T=Rotzmod(theta)*Trasl_xyz([0;0;d])*Trasl_xyz([a;0;0])*Rotxmod(alpha);
end