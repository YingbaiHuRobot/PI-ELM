function q = inversetKinematic_2Link_2D(w)
%INVERSETKINEMATIC_2LINK_2D tranlate the Cartesian corrdinate into angular coordinate
%   ����е��ĩ�˵ĵѿ�������ת��Ϊ������
%   w  ��е��ĩ������
%   w0 ��������
l1 = 8;
l2 = 8;
w0 = [-5;12];
x = w(1) - w0(1);
y = w(2) - w0(2);
q2 = acos((x^2+y^2-l1^2-l2^2)/(2*l1*l2));
% if x = 0 && 
if x >= 0
    q1 = atan(y/x) - atan(l2*sin(q2)/(l1+l2*cos(q2)));
else
    q1 = atan(y/x) - atan(l2*sin(q2)/(l1+l2*cos(q2))) + pi;
end
q = [q1;q2];
end

