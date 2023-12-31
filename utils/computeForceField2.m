function force = computeForceField(position, refTraj, gain)
% This function computes the force exerced by a divergent force field
% surrounding a reference trajectory.
% 
%Inputs -----------------------------------------------
%
% position : position of the system, column vector
% ref_traj : reference trajectory which is the ridge of the divergent force
% field. The force is computed as a linear function of the distance to the
% reference trajectory. The trajectory is represented by a matrix wich each
% row representing a point of the trajectory.
% gain : multiplictive gain.
% 
%Outputs -----------------------------------------------
%
% force : column vector that represent the force computed for the given
% position
%
minDist = realmax;
minIndex = 0;

for cnt = 1:size(refTraj,2)
    temp = norm(position-refTraj(:,cnt));
    if temp<minDist
        minDist = temp;
        minIndex = cnt;
    end
end

if minIndex ~= size(refTraj,2)
    tang = refTraj(:,minIndex+1)-refTraj(:,minIndex);
else
    tang = refTraj(:,minIndex)-refTraj(:,minIndex-1);
end
    
force = gain*norm(minDist)*[0 1; -1 0]*tang/norm(tang);
if force' * (position-refTraj(:,minIndex)) < 0
   force = -force; 
end
% force

end