function force = computeForceField_1(position, refTraj, gain)

if position(1) >= -5
    force = [0;(position(2)-refTraj(position(1)))*gain];
%     force = [0;gain];
else
    force = [0;0];
end

end