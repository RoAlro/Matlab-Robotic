function Q=mat2quat1(R)
%R(4,4)=1;
new_R=R;
q0=(1/2)*sqrt(new_R(1,1)+new_R(2,2)+new_R(3,3)+1);
q1=(sign(new_R(3,2)-new_R(2,3)))*(1/2)*sqrt(new_R(1,1)-new_R(2,2)-new_R(3,3)+1);
q2=(sign(new_R(1,3)-new_R(3,1)))*(1/2)*sqrt(-new_R(1,1)+new_R(2,2)-new_R(3,3)+1);
q3=(sign(new_R(2,1)-new_R(1,2)))*(1/2)*sqrt(-new_R(1,1)-new_R(2,2)+new_R(3,3)+1);
Q=[q0 q1 q2 q3];

end