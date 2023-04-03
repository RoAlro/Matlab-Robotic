function Q=multQuat1(Q1,Q2)
Q=[Q1(1,1)*Q2(1,1)-(Q1(1,2)*Q2(1,2)+Q1(1,3)*Q2(1,3)+Q1(1,4)*Q2(1,4))
    Q1(1,1)*Q2(1,2)+Q1(1,2)*Q2(1,1)+Q1(1,3)*Q2(1,4)-Q1(1,4)*Q2(1,3)
    Q1(1,1)*Q2(1,3)+Q1(1,3)*Q2(1,1)+Q1(1,4)*Q2(1,2)-Q1(1,2)*Q2(1,4)
    Q1(1,1)*Q2(1,4)+Q1(1,4)*Q2(1,1)+Q1(1,2)*Q2(1,3)-Q1(1,3)*Q2(1,2)];
%Se usa una matriz de 4x1, solo con fines estéticos, ya que si el
%código se escribiera horizontalmente no se vería en pantalla
%Por eso se usará la matriz transpuesta de Q,para obtener una matriz de
%1x4
Q=Q.';

end
