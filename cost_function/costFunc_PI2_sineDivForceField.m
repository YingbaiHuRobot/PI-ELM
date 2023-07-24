function [totCostSteps, transCostSteps, viapointCostSteps, accelerationCostSteps, stiffnessCostSteps] = costFunc_PI2_sineDivForceField(D,model,p)
% genCostSteps corresponds to S tilda before the summation of timesteps, totCostSteps corresponds to the
% S before the summation of timesteps, userCostSteps is the cost comming from
% the user defined cost function (before summation)

% implements a simple squared acceleration cost function with a penalty on
% the length of the parameter vector. Additionally, a via point has to be traversed at
% certain moment of time

goal = p.goal;
viapoints = p.viapoints;
constraints = p.constraints;

n_rep = size(D,1);
n_subtrials = size(D,2);

n_steps_real = p.duration/p.dt;  % the duration of the core trajectory in time steps --
n_steps_convergence = p.duration_convergence/p.dt;     % everything beyond this time belongs to the terminal cost

% compute cost
% RR = D.RctrlCost; % disregard the lambda factor, to allow the transition
% cost to be scaled down;
QQ = 0.0001;
R = 0.000001*eye(length(model.theta));

viapointCostWeight = 1;

% the "transition" cost

transCostSteps1 = zeros(n_steps_real,n_rep,n_subtrials);
transCostSteps2 = zeros(n_steps_real,n_rep,n_subtrials);

for k = 1:n_rep
    for m = 1:n_subtrials
        for n = 1:n_steps_real
            G = D(k,m).G(:,:,n);
            H = G/R*G'; % H can be 0 if h are zero because too far from Gaussian. Then M is NaN and so is cost
            B = G*G';            
            M = R\G'/H*G;
            transCostSteps1(n,k,m) = 0.5 * (model.theta - model.theta0 + M*D(k,m).eps(:,n))'*R*(model.theta - model.theta0 + M*D(k,m).eps(:,n));
            transCostSteps2(n,k,m) = 0.5 * log(det(B));
                
        end
    end
end

%--------------------------------------------------------------------------
% "user defined" cost
viapointCostSteps = zeros(n_steps_real, n_rep, n_subtrials);
accelerationCostSteps = zeros(n_steps_real, n_rep, n_subtrials);
stiffnessCostSteps = zeros(n_steps_real, n_rep, n_subtrials);
for k=1:n_rep
    for m= 1:n_subtrials

        for n=1:n_steps_real
            % cost during trajectory
            accelerationCostSteps(n,k,m)  =  QQ * norm(D(k,m).wdd(:,n))^2;
            if size(D(k,m).kp,1) == 1
                stiffnessCostSteps(n,k,m) = 0.0005 * norm(repmat(D(k,m).kp(:,n),2,1),1);
            else
                stiffnessCostSteps(n,k,m) = 0.0005 * norm(D(k,m).kp(:,n),1); % the one-norm of the stiffness. Does the 2-norm make more sens?
            end
            isConstrained = 0;
            for i = 1:length(constraints)
    %             isConstrained = isConstrained || (constrains(i).min_q1 <= D(k,m).w(1,n) && D(k,m).w(1,n) <= constrains(i).max_q1 && constrains(i).min_q2 <=D(k,m).w(2,n) && D(k,m).w(2,n)<=constrains(i).max_q2);
                isConstrained = isConstrained || (constraints(i).min_w1-0.01 <= D(k,m).w(1,n) && D(k,m).w(1,n) <= constraints(i).max_w1+0.01 && constraints(i).min_w2-0.01 <=D(k,m).w(2,n) && D(k,m).w(2,n)<=constraints(i).max_w2+0.01 );
            end
            if isConstrained
                viapointCostSteps(n,k,m) = viapointCostSteps(n,k,m)+ 1;
            end
        end


      % pass through the viapoints
        for i = 1:size(viapoints,1)
            [minVal, Index] =  min((D(k,m).w(1,1:n_steps_real)- viapoints(i,1)).^2+(D(k,m).w(2,1:n_steps_real)- viapoints(i,2)).^2);
    %             [minVal, Index] =  min((D(k,m).w(1,1:n_steps) - viapoints(i,1)).^2 + (D(k,m).w(2,1:n_steps) - viapoints(i,2)).^2);
            viapointCostSteps(Index,k,m) = viapointCostSteps(Index,k,m)+ viapointCostWeight* minVal;
        end
      
%       [minVal, Index] =  min((D(k,m).w(1,1:n_steps_real)+7.5).^2+(D(k,m).w(2,1:n_steps_real)+1).^2);
%       viapointCostSteps(Index,k,m) = viapointCostSteps(Index,k,m)+ viapointCostWeight* minVal;
%       [minVal, Index] =  min((D(k,m).w(1,1:n_steps_real)+5).^2+(D(k,m).w(2,1:n_steps_real)+0).^2);
%       viapointCostSteps(Index,k,m) = viapointCostSteps(Index,k,m)+ viapointCostWeight* minVal;
%       [minVal, Index] =  min((D(k,m).w(1,1:n_steps_real)+2.5).^2+(D(k,m).w(2,1:n_steps_real)-1).^2);
%       viapointCostSteps(Index,k,m) = viapointCostSteps(Index,k,m)+ viapointCostWeight* minVal;
%       [minVal, Index] =  min((D(k,m).w(1,1:n_steps_real)+2.5).^2+(D(k,m).w(2,1:n_steps_real)-1).^2);
%       viapointCostSteps(Index,k,m) = viapointCostSteps(Index,k,m)+ viapointCostWeight* minVal;
%       [minVal, Index] =  min((D(k,m).w(1,1:n_steps_real)+1.25).^2+(D(k,m).w(2,1:n_steps_real)-0.5).^2);
%       viapointCostSteps(Index,k,m) = viapointCostSteps(Index,k,m)+ viapointCostWeight* minVal;


      
      % reach goal
      viapointCostSteps(end,k,m) = viapointCostSteps(end,k,m)+ viapointCostWeight*((D(k,m).w(1,n_steps_real)-goal(1)).^2+(D(k,m).w(2,n_steps_real)-goal(2)).^2);

    end
end

%--------------------------------------------------------------------------
% mean of the subtrials
viapointCostSteps = mean(viapointCostSteps,3);    
stiffnessCostSteps = mean(stiffnessCostSteps,3);
% stiffnessCostSteps =0 ;
accelerationCostSteps = mean(accelerationCostSteps,3);
transCostSteps = mean(transCostSteps1,3)*1;
%--------------------------------------------------------------------------
totCostSteps = transCostSteps + viapointCostSteps + accelerationCostSteps + stiffnessCostSteps;
