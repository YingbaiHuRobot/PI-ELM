function [totCostSteps, transCostSteps, viapointCostSteps, accelerationCostSteps, stiffnessCostSteps] = costFunc_MLP_massPoint_2D(D,ELM,p)
%totCostSteps corresponds to the S before the summation of timesteps, userCostSteps is
%the cost comming from the user defined cost function (before summation)

% implements a simple squared acceleration cost function with a penalty on
% the length of the parameter vector. Additionally, a via point has to be traversed at
% certain moment of time
goal = p.goal;
viapoints = p.viapoints;
constraints = p.constraints;

n_rep = size(D,1);
n_subtrials = size(D,2);
n_steps = p.duration/p.dt;  % the duration of the core trajectory in time steps

goalCostWeight = 1;

% the "transition" cost
transCostSteps = zeros(n_steps,n_rep,n_subtrials);
for k = 1:n_rep
    for m = 1:n_subtrials
        for n = 1:n_steps

            if n == 1
                transCostSteps(n,k,m) = 0;
            else
                transCostSteps(n,k,m) = norm((D(k,m).w(:,n) - D(k,m).w(:,n-1)),2);
            end   
        end
    end
end

%--------------------------------------------------------------------------

viapointCostSteps = zeros(n_steps, n_rep, n_subtrials);
accelerationCostSteps = zeros(n_steps, n_rep, n_subtrials);
stiffnessCostSteps = zeros(n_steps, n_rep, n_subtrials);
for k=1:n_rep
    for m= 1:n_subtrials
        for n=1:n_steps
        % cost during trajectory
            accelerationCostSteps(n,k,m)  =  norm(D(k,m).wdd(:,n),2);
            stiffnessCostSteps(n,k,m) = norm(D(k,m).kp(:,n),2);
            % avoid the constrained zone
%             isConstrained = 0;
%                 for i = 1:length(constraints)
%                     if strcmp(constraints(i).type, 'rectangle')
%                         isConstrained = isConstrained || (constraints(i).min_w1-0.5<= D(k,m).w(1,n) && D(k,m).w(1,n) <= constraints(i).max_w1+0.5 && constraints(i).min_w2-0.5 <=D(k,m).w(2,n) && D(k,m).w(2,n)<=constraints(i).max_w2+0.5);
%                     elseif strcmp(constraints(i).type, 'circle')
%                         isConstrained = isConstrained || norm(D(k,m).w(1,n)-[constraints(i).center1,constraints(i).center2] ) < constraints(i).radius + 1 ;
%                     end
%                 end
%                 
%                 
%             if isConstrained
%                 viapointCostSteps(n,k,m) = viapointCostSteps(n,k,m)+ 0.1;
%             end
        end
        

      % pass through the viapoints
        for i = 1:size(viapoints,1)
            [minVal, Index] =  min(sqrt((D(k,m).w(1,1:n_steps) - viapoints(i,1)).^2 + (D(k,m).w(2,1:n_steps) - viapoints(i,2)).^2));
%             [minVal, Index] =  min((D(k,m).w(1,1:n_steps) - viapoints(i,1)).^2 + (D(k,m).w(2,1:n_steps) - viapoints(i,2)).^2);
            viapointCostSteps(Index,k,m) = viapointCostSteps(Index,k,m)+ minVal;
        end
        % reach goal
        d_final  = sqrt((D(k,m).w(1,n_steps)-goal(1)).^2+(D(k,m).w(2,n_steps) - goal(2)).^2);
%         d_final  = (D(k,m).w(1,n_steps)-goal(1)).^2+(D(k,m).w(2,n_steps) - goal(2)).^2;
%         d_final = d_final ^ (1/3);
        viapointCostSteps(end,k,m) = viapointCostSteps(end,k,m) + goalCostWeight * d_final;
    
    end
end

%--------------------------------------------------------------------------
% mean of the subtrials
viapointCostWeight = 1;
stiffnessCostWeight = 0.00005;
accelerationCostWeight = 0.0002;
transCostWeight = 0.005;
% viapointCostSteps
% viapointCostSteps
viapointCostSteps = mean(viapointCostSteps,3) * viapointCostWeight;   
stiffnessCostSteps = mean(stiffnessCostSteps,3) * stiffnessCostWeight;
accelerationCostSteps = mean(accelerationCostSteps,3) * accelerationCostWeight;
transCostSteps = mean(transCostSteps,3)*transCostWeight;
%--------------------------------------------------------------------------
totCostSteps = viapointCostSteps + accelerationCostSteps + stiffnessCostSteps+ transCostSteps;
