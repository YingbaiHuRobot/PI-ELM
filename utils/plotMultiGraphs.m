function  plotMultiGraphs(D,totCostSteps,p,D_eval,totCostSteps_eval, figID)

n_dim = double(p.n_dim);
n_Gauss = double(p.n_Gauss);
n_dim_kp = n_dim;
n_in = n_dim;
n_out = n_dim + n_dim_kp; % &&
n_param = n_Gauss*(n_in*n_out+n_out); 

gray = [0.5 0.5 0.5];
%dt = p.dt;
%T  = (1:length(D(1).q))'*dt;
T_real = 0.1:p.dt:p.duration;
T_tot = 0.1:p.dt:p.duration+p.duration_convergence;
TT  = zeros(length(D(1).q),p.rep);
TTT = zeros(length(D(1).q),p.rep*n_param);

%figure(figID);
figure
clf;

% pos, vel, acc
for j=1:n_dim
  
  % desired traj position
  subplot(n_dim+1,5,(j-1)*5+1);
  
  for k=1:p.rep
      TT(:,k)=D(k).traj_y(j,:);
  end
  plot(T_tot,D_eval(1).traj_y(j,:),'Color',gray,'LineWidth',2);
  hold on;
  plot(T_tot,TT);
  hold off;
  ylabel(sprintf('y_%d',j));
  title(sprintf('Dimension_%d',j));
  
  % desired traj velocity
  subplot(n_dim+1,5,(j-1)*5+2);
    
  for k=1:p.rep
      TT(:,k)=D(k).traj_yd(j,:);
  end
  plot(T_tot,D_eval(1).traj_yd(j,:),'Color',gray,'LineWidth',2);
  hold on;
  plot(T_tot,TT);
  hold off;
  ylabel(sprintf('yd_%d',j));
  
  % desired traj acceleration    !!! replace by point mass acceleration?
  % (and reorder)
%   subplot(2*n_dim,5,(j-1)*10+3);
%   
%   for k=1:p.rep,
%       TT(:,k)=D(k).dmp(j).ydd;
%   end
%   plot(T,D_eval(1).dmp(j).ydd,'Color',gray,'LineWidth',2);
%   hold on;
%   plot(T,TT);
%   hold off;
%   ylabel(sprintf('ydd_%d',j));
  
  % point mass position
  subplot(n_dim+1,5,(j-1)*5+3);
  
  for k=1:p.rep
      TT(:,k)=D(k).q(j,:);
  end
  plot(T_tot,D_eval(1).q(j,:),'Color',gray,'LineWidth',2);
  hold on;
  plot(T_tot,TT);
  hold off;
  ylabel(sprintf('q_%d',j));

  % point mass velocity
  subplot(n_dim+1,5,(j-1)*5+4);
    
  for k=1:p.rep
      TT(:,k)=D(k).qd(j,:);
  end
  plot(T_tot,D_eval(1).qd(j,:),'Color',gray,'LineWidth',2);
  hold on;
  plot(T_tot,TT);
  hold off;
  ylabel(sprintf('qd_%d',j));

  % kp
  subplot(n_dim+1,5, (j-1)*5+5);
  if n_dim_kp == 1
      dim_kp = 1;
  else
      dim_kp = j;
  end
  for k=1:p.rep
      TT(:,k)=D(k).kp(dim_kp,:);
  end
  plot(T_tot,D_eval(1).kp(dim_kp,:),'Color',gray,'LineWidth',2);
  hold on;
  plot(T_tot,TT);
  hold off;
  ylabel(sprintf('kp_%d',j));
  
end
  
% the cost
subplot(n_dim+1,5, n_dim*5+2);
plot(T_real,totCostSteps_eval,'Color',gray,'LineWidth',2);
hold on;
plot(T_real,totCostSteps);
ylabel(sprintf('step cost r'));

% the cumulative cost
subplot(n_dim+1,5, n_dim*4+3);
S_eval = rot90(rot90(cumsum(rot90(rot90(totCostSteps_eval)))));
S = rot90(rot90(cumsum(rot90(rot90(totCostSteps)))));
plot(T_real,S_eval,'Color',gray,'LineWidth',2);
hold on;
plot(T_real,S);
ylabel(sprintf('R=sum(r)'));
hold off

f = gcf;
end

