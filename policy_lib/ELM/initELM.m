function ELM = initELM(trainingData,n_dim,n_hidden,activationFnc)
% Entrance function for sufficient or weak conditions to ensure the
% globally asymptotical stability of the dynamical system. 
% Inputs:
%   TraniningData:  A 2d x N_Total matrix containing all demonstration data points.
%   n_hidden:    The number of the hidden neural layer
%   ActivationFunction: The active function in the neuron


inputData = trainingData(1:n_dim,:)';
targetOutput=trainingData(n_dim+1:end,:)';

% n_out = size(targetOutput,2);

w1 = 2*rand(n_dim,n_hidden)-1;
% [~,ps] = mapminmax(inputData');
% gain = eye(n_dim) .* ps.gain;
% offset = -gain * ps.xoffset + ps.ymin .* ones(n_dim,1);
% b1 = offset' * w1;
% w1 = gain' * w1;
adjust = max(abs(inputData),[],1);
w1 = w1./repmat(adjust',1,n_hidden);
b1= 2*rand(1,n_hidden)-1; 

tempH = inputData * w1 + b1;

ELM.w1 = w1;
ELM.b1 = b1;
ELM.actFuc = activationFnc;
eval(sprintf('h = %s(tempH);',activationFnc));
% % Calculate hidden neuron output matrix H
% switch lower(ActivationFunction)
%     case {'logsig','sigmoid'}
%         % Sigmoid 
% %         H = 2 ./ (1 + exp(-tempH))-1;
%         H = tansig(tempH);
%         ELM.actFuc='logsig';
%     case {'sin','sine'}
%         % Sine
%         H = sin(tempH);  
%         ELM.actFuc='sin';
%     case {'tanh'}
%         % tanh
%         H=tanh(tempH);
%         ELM.actFuc='tanh';
%     case {'sinh'}
%         H=sinh(tempH);
%         ELM.actFuc='sinh';
%     case {'hardlim'}
%         % Hard Limit
%         H = hardlim(tempH); 
%         ELM.actFuc='hardlim';
%         % More activation functions can be added here 
%     otherwise
%         error('wrong active function!');
% end
clear tempH;                                        %   Release the temparary array for calculation of hidden neuron output matrix H

w2 = pinv(h) * targetOutput;
ELM.w2 = w2;
ELM.theta0 = reshape(w2, [], 1);


