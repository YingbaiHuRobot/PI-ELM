function w = directKinematic_2Link_2D(q)
 %ANGLE2CART translate the angular coordinate into Cartesian corrdinate
%   q �Ƕ�      
%   w ĩ������  
%   w0 �����ѿ�������
l1=8;
l2=8;
q1 = q(1);
q12 = q(1)+q(2);
w0 = [-5;12];
w = w0 + [l1*cos(q1); l1*sin(q1)] + [l2*cos(q12); l2*sin(q12)];
end

