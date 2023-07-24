function trainingData = defineDataset(demos,p)
%AUGMENTDATA 

n_dim = p.n_dim;
dt = p.dt;
n_dim_kp = p.n_dim_kp;
useSEDS = p.SEDS_init;
policy = p.policy;
stiffness_learning = p.stiffness_learning;
parameter = p.parameter;
duration = p.duration;
n_demos = size(demos,2);


n_steps = duration/dt; % Length of each trajectory
wMax = round(n_steps/40); % Warping time window
% s = cell(1,n_dim);
sample = zeros(n_dim,n_steps,n_demos);
for n = 1:n_demos % linspace(1,size(datass{n}.pos,2),nbData) 范围内，步长采样，插值
    sample(:,:,n) = spline(1:size(demos{n},2), demos{n}(1:n_dim,:), linspace(1,size(demos{n},2),n_steps)); %Resampling
end                    

tmpSample = cell(1,n_demos);
tmpSample{1}.Data = sample(:,:,1);
for n=2:n_demos
    [tmpSample{1}.Data, tmpSample{n}.Data, tmpSample{n-1}.wPath] = DTW(tmpSample{1}.Data, sample(:,:,n), wMax);
    pp = tmpSample{n-1}.wPath(1,:);
    for m=2:n
        DataTmp = tmpSample{m}.Data(:,pp);
        tmpSample{m}.Data = spline(1:size(DataTmp,2), DataTmp, linspace(1,size(DataTmp,2),n_steps)); %Resampling
    end
end

rSample = zeros(n_dim,n_steps,n_demos);
% rSample = cell(1,n_demos);
for n=1:n_demos
    rSample(:,:,n) = tmpSample{1,n}.Data;
end

if strcmp(policy, 'RNN')
    inputData = cell(1,n_steps-1);
    targetData = cell(1,n_steps-1);
    wd = cell(1,n_demos);
    for n2 = 1:n_demos    
        wd{n2} = diff(rSample(1:n_dim,:,n2)')'/dt;
    end
    for n1 = 1:n_steps-1    
    %     wd = diff(demos{n1}(1:n_dim,:)')'/dt;
        iInput = zeros(n_dim,n_demos);
        iTarget = zeros(n_dim,n_demos);
        for n2 = 1:n_demos
            iInput(:,n2) = rSample(:,n1,n2);
            iTarget(:,n2) = wd{n2}(:,n1);
        end
        
        if stiffness_learning == 1
            if length(parameter) == 1
                kp = parameter*ones(n_dim_kp,size(iTarget,2));
                iTarget = cat(1,iTarget,kp);
            elseif length(parameter) == n_dim_kp
                kp = parameter.*ones(n_dim_kp,size(iTarget,2));
                iTarget = cat(1,iTarget,kp);
            else
                error('Please check the dimension of kp!');
            end
        end
    
        inputData{1,n1} = iInput;
        targetData{1,n1} = iTarget;
    end
    
    trainingData.inputData = inputData;
    trainingData.targetData = targetData;
else
    sum_steps = 0;
     for n1 = 1:n_demos
        sum_steps = sum_steps + n_steps-1;   
        if n_dim == 2
            trainingData(:,sum_steps - n_steps+1 +1:sum_steps) = [rSample(1,1:n_steps-1,n1); rSample(2,1:n_steps-1,n1); diff(rSample(1,:,n1))/dt; diff(rSample(2,:,n1)/dt)];
        elseif n_dim == 3
            trainingData(:,sum_steps - n_steps+1 +1:sum_steps) = [rSample(1,1:n_steps-1,n1); rSample(2,1:n_steps-1,n1); rSample(3,1:n_steps-1,n1); diff(rSample(1,:,n1))/dt; diff(rSample(2,:,n1)/dt); diff(rSample(3,:,n1)/dt)];
        end
     end
end


if p.augment_data == 1 && ~strcmp(policy, 'RNN')
    max_x = max(trainingData(1,:));
    min_x = min(trainingData(1,:));
    max_y = max(trainingData(2,:));
    min_y = min(trainingData(2,:));
    d1 = max_x - min_x;
    d2 = max_y - min_y;

    min_x = min_x - 0.3*d1;
    max_x = max_x + 0.3*d1;
    min_y = min_y - 0.3*d2;
    max_y = max_y + 0.3*d2;

    refDemo = mean(rSample,3);
    
    if n_dim == 2
        nx = round(sqrt(n_steps*n_demos/10));
        ny = nx;
    %     ax_x = linspace(min_x,max_x,nx);
    %     ax_y = linspace(min_y,max_y,ny);
        ax_x = cat(2,linspace(min_x,min_x+0.3*d1,nx/2),linspace(max_x-0.3*d1,max_x,nx/2));
        ax_y = cat(2,linspace(min_y,min_y+0.3*d2,ny/2),linspace(max_y-0.3*d2,max_y,ny/2));
        [x_tmp, y_tmp] = meshgrid(ax_x,ax_y); %meshing the input domain
        w_aug = [x_tmp(:) y_tmp(:)]';
        training_wd = sqrt(trainingData(3,:).*trainingData(3,:) + trainingData(4,:).*trainingData(4,:));
    elseif n_dim == 3
        nx = round(nthroot(n_steps*n_demos/10));
        ny = nx;
        nz = nx;

        max_z = max(trainingData(3,:));
        min_z = min(trainingData(3,:));
        d3 = max_z - min_z;
        min_z = min_z - 0.3*d3;
        max_z = max_z + 0.3*d3;

    %     ax_x = linspace(min_x,max_x,nx);
    %     ax_y = linspace(min_y,max_y,ny);
    %     az_z = linspace(min_z,max_z,nz);
        ax_x = cat(2,linspace(min_x,min_x+0.3*d1,nx/2),linspace(max_x-0.3*d1,max_x,nx/2));
        ax_y = cat(2,linspace(min_y,min_y+0.3*d2,ny/2),linspace(max_y-0.3*d2,max_y,ny/2));
        az_z = cat(2,linspace(min_z,min_z+0.3*d3,nz/3),linspace(max_z-0.3*d3,max_z,nz/2));
        [x_tmp, y_tmp, z_tmp] = meshgrid(ax_x,ax_y,az_z); %meshing the input domain
        w_aug = [x_tmp(:) y_tmp(:) z_tmp(:)]';
        training_wd = sqrt(trainingData(4,:).^2 + trainingData(5,:).^2 + trainingData(6,:).^2);
    else
        error('n_dim = 2 or 3.');
    end
    % max_wd = max(training_wd);
    mean_wd = mean(training_wd);
    wd_aug = zeros(n_dim,length(w_aug));


    for i=1:length(w_aug)
        minDist = realmax;
        minIndex = 0;
        for cnt = 1:size(refDemo,2)
    %         size(w_aug(:,i))
    %         size(refDemo(:,cnt))
            temp = norm(w_aug(:,i)-refDemo(:,cnt));
            if temp<minDist
                minDist = temp;
                minIndex = cnt;
            end
        end
        wd_aug_i = (refDemo(:,minIndex)-w_aug(:,i)) + (refDemo(:,end)-w_aug(:,i));
        wd_aug(:,i) = mean_wd * wd_aug_i/norm(wd_aug_i);
    end

    augData = cat(1,w_aug,wd_aug);
    % size(trainingData)
    % size(augData)
    trainingData = cat(2,trainingData,augData);
end

if ~(useSEDS && strcmp(policy, 'GMR')) && ~strcmp(policy, 'RNN') && stiffness_learning == 1
    if length(parameter) == 1
        kp = parameter*ones(n_dim_kp,size(trainingData,2));
        trainingData = cat(1,trainingData,kp);
    elseif length(parameter) == n_dim_kp
        kp = parameter.*ones(n_dim_kp,size(trainingData,2));
        trainingData = cat(1,trainingData,kp);
    else
        error('Please check the dimension of kp!');
    end       
end
end

