function force = computeForceField_2(position, gain)

if position(1) >= 0
    force = [0;gain];
else
    force = [0;0];
end

end