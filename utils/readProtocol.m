function protocol=readProtocol(protocol_name)
% parses the protocol file <protocol_name> 

fp = fopen(protocol_name,'r');
if fp == -1
  error('Cannot open protocol file');
end

% read all lines, discard lines that start with comment sign
protocol = [];
count    = 0;
while 1
    line = fgetl(fp);
    if ~ischar(line)
        break;
    end
    if numel(line) == 0 || line(1) == '%'
        continue;
    end
    d = textscan(line,'%s %s %s %s %d %s %d %d %f %f %f %d %d %d %d %d %s %s %s %s %d %d %d %d %d %d %s %s %f %d %d %d %s %d %d %d %d');
    count = count+1;
    % the <protocol> structure stores all important parameters to run PI2
    % scenario 
    n_dim = d{7};
    model = char(d{20});
    
    protocol(count).n_dim = n_dim;
    protocol(count).model = model;
    
    start_str = strsplit(char(d{1}),',');
    if (length(start_str) ~= n_dim && ~strcmp(model,'robot')) || (length(start_str) ~= 7 && strcmp(model,'robot'))
        error('Please check the dimension of the start position and make sure it equals to n_dim.');
    end
    start = zeros(length(start_str),1);
    for i= 1:length(start_str)
        start(i) = str2double(start_str{i}); 
    end
    protocol(count).start = start;
    
    goal_str = strsplit(char(d{2}),',');
    if length(goal_str) ~= n_dim
        error('Please check the dimension of the goal position and make sure it equals to n_dim.');
    end
    goal = zeros(length(goal_str),1);
    for i= 1:length(goal_str)
        goal(i) = str2double(goal_str{i}); 
    end
    protocol(count).goal = goal;
    
    if strcmp(char(d{3}),'none')
        protocol(count).viapoints = [];
    else
%         exp = sprintf("((([-\\d\\.]*){%d}\\,([-\\d\\.]*){1})\\;)*(([-\\d\\.]*){%d}\\,([-\\d\\.]*){1})",n_dim-1,n_dim-1);
%         matchStr = regexp(char(d{3}),exp,'match');
%         if isempty(matchStr) || ~strcmp(matchStr{1},char(d{3}))
%             error('Please check the format of the viapoints.')
%         end
        viapointsList_str = strsplit(char(d{3}),';');
        viapoints = zeros(length(viapointsList_str),n_dim);
        for i = 1:length(viapointsList_str)
            viapoint_str = strsplit(viapointsList_str{i},',');
            if length(viapoint_str) ~= n_dim
                error('Please check the dimension of the viapoints and make sure it equals to n_dim.');
            end
            for j= 1:n_dim
                viapoints(i,j) = str2double(viapoint_str{j}); 
            end
        end
        protocol(count).viapoints = viapoints;
    end
    
    if strcmp(char(d{4}),'none')
        protocol(count).constraints = [];
    else
        constrainsList_str = strsplit(char(d{4}),';');
        constraints = [];
        for i = 1:length(constrainsList_str)
            constrain_str = strsplit(constrainsList_str{i},',');
            
            if strcmp(constrain_str{1}, 'rectangle') || strcmp(constrain_str{1}, 'r')
                if length(constrain_str) ~= n_dim*2+1
                    error('Please check the dimension of the constrains and make sure it equals to n_dim * 2.');
                end
                constraint.type = 'rectangle';
                constraint.min_w1 = str2double(constrain_str{2});
                constraint.max_w1 = str2double(constrain_str{3});
                constraint.min_w2 = str2double(constrain_str{4});
                constraint.max_w2 = str2double(constrain_str{5});
                if n_dim == 3
                    constraint.min_w3 = str2double(constrain_str{6});
                    constraint.max_w3 = str2double(constrain_str{7});
                end
            elseif strcmp(constrain_str{1}, 'circle') || strcmp(constrain_str{1}, 'c')
                if length(constrain_str) ~= n_dim+2
                    error('Please check the dimension of the constrain and make sure it equals to n_dim * 2.');
                end
                constraint.type = 'circle';
                constraint.center1 = str2double(constrain_str{2});  
                constraint.center2 = str2double(constrain_str{3});
                constraint.radius = str2double(constrain_str{4});
                if n_dim == 3
                    constraint.center1 = str2double(constrain_str{2}); 
                    constraint.center2 = str2double(constrain_str{3}); 
                    constraint.center3 = str2double(constrain_str{4});
                    constraint.radius = str2double(constrain_str{5});
                end
            else
                error('Please check the type of the constrain and make sure it is rectangle or circle.');
            end
            constraints = [constraints constraint];
        end
        protocol(count).constraints = constraints;
    end
    
    protocol(count).augment_data= d{5};
    
    hiddenLayerList_str = strsplit(char(d{6}),',');
    n_hiddens = zeros(1,length(hiddenLayerList_str));
    for i= 1:length(hiddenLayerList_str)
        n_hiddens(i) = str2double(hiddenLayerList_str{i}); 
    end
%     n_hiddens
    protocol(count).n_hiddens = n_hiddens;
    
%     protocol(count).n_hidden    = d{6};
    
    protocol(count).n_dim_kp    = d{8};
    protocol(count).duration    = d{9};
    protocol(count).dt          = d{10};
    protocol(count).std         = d{11};
    protocol(count).n_runs      = d{12};
    protocol(count).updates     = d{13};
    protocol(count).rep         = d{14};
    protocol(count).n_subtrials = d{15};
    protocol(count).n_reuse     = d{16};
    protocol(count).cost        = char(d{17});
    protocol(count).learning_method = char(d{18});
    protocol(count).policy      = char(d{19});
    protocol(count).forceField  = d{21};
    protocol(count).feedback    = d{22};
    protocol(count).SEDS_constr = d{23};
    protocol(count).SEDS_init   = d{24};
    protocol(count).relearnGMM  = d{25};
    protocol(count).n_Gauss     = d{26};
    
    parameter_str = strsplit(char(d{27}),',');
    parameter = zeros(length(parameter_str),1);
    for i= 1:length(parameter_str)
        parameter(i) = str2double(parameter_str{i}); 
    end
    
    protocol(count).parameter         = parameter;
    protocol(count).demo_set    = char(d{28});
    protocol(count).duration_convergence  = d{29};
    protocol(count).disable_plotting = d{30};
    protocol(count).noisyStartPos = d{31};
    protocol(count).plotIntermediate = d{32};
    protocol(count).PI2_type      = d{33};
    protocol(count).fixed_noise = d{34};
    protocol(count).dataset_type = d{35};
    protocol(count).disable_saving = d{36}; 
    protocol(count).stiffness_learning = d{37};
  
end
fclose(fp);
end