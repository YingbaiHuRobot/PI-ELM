function [q,qd,qdd,command] = massPoint2D(q,qd,traj_y,traj_yd,kp,kd,Fieldforce,dt)
%MODELMASSPOINT2D dynamics model of a mass point
%   质点动力学方程
damp = 1;
mass = 1;
% traj_y-q
% traj_yd-qd
command = kp.*(traj_y-q) + kd.*(traj_yd-qd);
qdd = (command - qd * damp + Fieldforce)/mass;
qd  = qdd * dt + qd;
q   = qd * dt + q;
end

