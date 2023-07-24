function outputs = forward_ELM(model,w,ref_w,p,u)

n_hidden = p.n_hiddens;
n_dim = double(p.n_dim);
if p.stiffness_learning == 1
    n_dim_kp = p.n_dim_kp;
else
    n_dim_kp = 0;
end
n_out = n_dim + n_dim_kp;
n_param = n_hidden * n_out;

if p.feedback == 0 
    h = semiforward_ELM(ref_w,model);  % !!! first step            
elseif p.feedback == 1
    h = semiforward_ELM(w,model); 
end

G = zeros(n_out, n_param);           
for i = 1:n_out
    G(i,:) = [zeros(1,(i-1)*(n_hidden)) h zeros(1,((n_out)-i)*(n_hidden))];
end
% if strcmp(p.learning_method)
outputs = G * u;
end

function h = semiforward_ELM(input,ELM)

w1 = ELM.w1;
b1 = ELM.b1;
activationFnc = ELM.actFuc;
tempH = input' * w1 + b1;

eval(sprintf('h = %s(tempH);',activationFnc));

end

