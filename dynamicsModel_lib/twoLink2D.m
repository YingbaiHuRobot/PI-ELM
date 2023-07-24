function [q,qd,qdd,w,wd,Tau] = twoLink2D(q,qd,w,wd,ref_w,ref_wd,strategy,dt,kp,kd)
% function [w,wd,wdd,Tau] = twoLink2D(w,wd,ref_w,ref_wd,kp,kd,dt)
% modelRod2D dynamics model of two 2D connecting-rods
%   二连杆机构

m1 = 2;
m2 = 1.85;
l1 = 8;
l2 = 8;

lc1=0.5*l1;
lc2=0.5*l2;
I1=1/4*m1*l1^2;
I2=1/4*m2*l2^2;
g=9.81;
p=zeros(1,5);
p(1)=m1*lc1^2+m2*l1^2+I1;
p(2)=m2*lc2^2+I2;
p(3)=m2*l1*lc2;
p(4)=m1*lc1+m2*l1;
p(5)=m2*lc2;

D = [p(1)+p(2)+2*p(3)*cos(q(2))     p(2)+p(3)*cos(q(2));
    p(2)+p(3)*cos(q(2))             p(2)];
C = [-p(3)*qd(2)*sin(q(2))  -p(3)*(qd(1)+qd(2))*sin(q(2));
    p(3)*qd(1)*sin(q(2))    0];
G = [p(4)*g*cos(q(1))+p(5)*g*cos(q(1)+q(2));
    p(5)*g*cos(q(1)+q(2))];
   
% F_end =  kp .* (ref_w - w) + kd .* (ref_wd - wd);

J = [-l1*sin(q(1))-l2*sin(q(1)+q(2)), -l2*sin(q(1)+q(2));
    l1*cos(q(1))+l2*cos(q(1)+q(2)),  l2*cos(q(1)+q(2))];

% Tau = J'*F_end;

e = ref_w - w;
de = ref_wd - wd;
if strcmp(strategy,'PD')
    F_end =  kp .* e + kd .* de;
    Tau = J'*F_end;
% elseif strcmp(strategy,'backstepping')
%     F_end =  kp .* e + kd .* de;
%     Tau = J'*F_end;
elseif strcmp(strategy,'SMC')
    K = kp;
    lambda = kd;
    S= de + lambda.*e;
    F_end = lambda.*de + K.*sgn(S);
    Tau = J'*F_end + C*qd + G;
else
    error("Only 'PD' or 'SMC' is allowed.");
end

qdd =D\(Tau-C*qd-G);
qd = qd + qdd*dt;
q = q + qd*dt;

w = directKinematic_2Link_2D(q);
J = [-l1*sin(q(1))-l2*sin(q(1)+q(2)), -l2*sin(q(1)+q(2));
    l1*cos(q(1))+l2*cos(q(1)+q(2)),  l2*cos(q(1)+q(2))];
wd = J * qd;


end




