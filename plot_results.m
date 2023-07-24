addpath ./utils
addpath ./configuration
addpath ./2Dletters
addpath ./khansari-lasahandwritingdataset/Dataset

%p_name = 'Spoon_massPoint_2D';
%p_name = 'Leaf1_massPoint_2D';
p_name = 'Spoon_forceField_stiffness_massPoint_2D';

p = readProtocol(strcat(p_name, '.txt'));

class_name = sprintf('%s_%dViapoint_%dConstraint',p(1).demo_set, size(p(1).viapoints,1), size(p(1).constraints,2));
if p(1).stiffness_learning
    class_name = strcat(class_name, '_stiffness');
end
if p(1).forceField
    class_name = strcat(class_name, '_forceField');
end

data_path = strcat('./results/', class_name, '/data.mat');

results = load(data_path).results_protocol;

demos = load(char(p(1).demo_set)).demos;
plotActualDesiredPath_r(p(1), results.Dfin, demos)
plotTotCost_NumRollouts_r(p(1), results(1, 1))