function plotActualDesiredPath(p,D_eval,model,demos,folder_name, class_name)

n_dim = p.n_dim;
n_subtrials = p.n_subtrials;
forceField = p.forceField;

constraints = p.constraints;
start = p.start;
goal = p.goal;
viapoints = p.viapoints;
%figure(figID+1);
 
if n_dim == 3
    figure
    hold on
    
    for m = 1:n_subtrials
        plot(D_eval(1,m).w(1,1:p.duration/p.dt),D_eval(1,m).w(2,1:p.duration/p.dt),'b.');
        plot(D_eval(1,m).ref_w(1,1:p.duration/p.dt),D_eval(1,m).ref_w(2,1:p.duration/p.dt),'r.');
        plot(D_eval(1,m).w(1,p.duration/p.dt+1:end),D_eval(1,m).w(2,p.duration/p.dt+1:end),'c.');
        plot(D_eval(1,m).ref_w(1,p.duration/p.dt+1:end),D_eval(1,m).ref_w(2,p.duration/p.dt+1:end),'y.');
        for i = 1:length(constraints)
             x = constraints(i).min_w1; 
             y = constraints(i).min_w2;
             w = constraints(i).max_w1 - constraints(i).min_w1;
             h = constraints(i).max_w2 - constraints(i).min_w2;
            rectangle('Position',[x y w h],'FaceColor','k')
        end
    end
    if size(viapoints,1) ~= 0
        plot(viapoints(:,1), viapoints(:,2), '.', 'Color', '#D95319', 'MarkerSize', 20);
    end
    plot(goal(1), goal(2), '.', 'Color', '#D95319', 'MarkerSize',30);
%     axis equal
    xlabel('x')
    ylabel('y');
    hold off
end

figure
hold on

n_demos = size(demos,2);
if n_dim == 2
    for n1 = 1:n_demos
        
        if p.dataset_type == 0
            a = plot(demos{n1}(1,:), demos{n1}(2,:),'Linewidth',0.02);
            a.Color(4) = 0.4;
        elseif p.dataset_type == 1 || p.dataset_type == 2
            a = plot(demos{n1}.pos(1,:), demos{n1}.pos(2,:),'Linewidth',0.02);
            a.Color(4) = 0.4;
        else 
            error('Please the dataset type in the protocol!!!');
        end
    
    end 

elseif n_dim == 3
    for n1 = 1:n_demos
        plot(demos{n1}(1,:), demos{n1}(2,:), demos{n1}(3,:));
    end
end

if forceField
    if forceField == 1
        forceFieldTraj = @(x) -sin(2/10*pi*x); 
        nPoints = 100;
        X = linspace(-5, 1 , nPoints);
        Y = forceFieldTraj(X);
        plot(X, Y, 'c:', 'LineWidth', 2) 
    end
    if p.dataset_type == 1
        xSample = 35;
        ySample = 10;
        for x = linspace(-13, 13, xSample)
            for y = linspace(-12, 12, ySample)
                if forceField == 1
                    force = computeForceField_1([x;y], forceFieldTraj, 0.1);
                elseif forceField == 2
                    force = computeForceField_2([x;y], 1);
                else
                    force = 0;
                end
                quiver(x,y,force(1),force(2),'c-','MaxHeadSize', 15, 'Linewidth', 1)
            end
        end
    elseif p.dataset_type == 2
        if strcmp(p.demo_set, 'JShape')
            xSample = 61;
            ySample = 25;
            xmin = -40;
            xmax = 60;
            ymin = -40;
            ymax = 50;
        elseif strcmp(p.demo_set, 'Khamesh')
            xSample = 11;
            ySample = 15;
            xmin = -10;
            xmax = 10;
            ymin = -45;
            ymax = 5;
        elseif strcmp(p.demo_set, 'Leaf_1')
            xSample = 41;
            ySample = 15;
            xmin = -35;
            xmax = 35;
            ymin = -5;
            ymax = 55;
        elseif strcmp(p.demo_set, 'Spoon')
            xSample = 13;
            ySample = 12;
            xmin = -10;
            xmax = 10;
            ymin = -35;
            ymax = 15;
        end
        for x = linspace(xmin, xmax, xSample)
            for y = linspace(ymin, ymax, ySample)
                if forceField == 1
                    force = computeForceField_1([x;y], forceFieldTraj, 0.1);
                elseif forceField == 2
                    force = computeForceField_2([x;y], 1);
                else
                    force = 0;
                end
                quiver(x,y,force(1),force(2),'c-','MaxHeadSize', 15, 'Linewidth', 1)
            end
        end
    elseif p.dataset_type == 0
        xSample = 25;
        ySample = 20;
        for x = linspace(-5, 2,xSample)
    %         x = cnt2 * 14/xSample - 12;
            for y = linspace(-2.5, 2.5, ySample)
                if forceField == 1
                    force = computeForceField_1([x;y], forceFieldTraj, 0.1);
                elseif forceField == 2
                    force = computeForceField_2([x;y], 1);
                else
                    force = 0;
                end
%                 force = computeForceField([x;y], forceFieldTraj, 0.1);
                quiver(x,y,force(1),force(2),'c-','MaxHeadSize', 15, 'Linewidth', 1)
            end
        end
    
    end
end

if n_dim == 2
    for m = 1:n_subtrials
        if p.stiffness_learning == 0
            plot([p.start(1) D_eval(1,m).ref_w(1,1:p.duration/p.dt)],[p.start(2) D_eval(1,m).ref_w(2,1:p.duration/p.dt)],'r','LineWidth',1.5);
        end
        if p.stiffness_learning == 1
            plot([p.start(1) D_eval(1,m).w(1,1:p.duration/p.dt)],[p.start(2) D_eval(1,m).w(2,1:p.duration/p.dt)],'m','LineWidth',1.5);
        end
        plot(D_eval(1,m).w(1,p.duration/p.dt+1:end),D_eval(1,m).w(2,p.duration/p.dt+1:end),'c.');
        plot(D_eval(1,m).ref_w(1,p.duration/p.dt+1:end),D_eval(1,m).ref_w(2,p.duration/p.dt+1:end),'y.');
    end 
    xlabel('x')
    ylabel('y');
elseif n_dim == 3
%     axis equal
    view([1 1 1]);
%     axis equal
    for m = 1:n_subtrials
        plot3(D_eval(1,m).w(1,1:p.duration/p.dt),D_eval(1,m).w(2,1:p.duration/p.dt),D_eval(1,m).w(3,1:p.duration/p.dt),'b.');
        plot3(D_eval(1,m).ref_w(1,1:p.duration/p.dt),D_eval(1,m).ref_w(2,1:p.duration/p.dt),D_eval(1,m).ref_w(3,1:p.duration/p.dt),'r.');
        plot3(D_eval(1,m).w(1,p.duration/p.dt+1:end),D_eval(1,m).w(2,p.duration/p.dt+1:end),D_eval(1,m).w(3,p.duration/p.dt+1:end),'c.');
        plot3(D_eval(1,m).ref_w(1,p.duration/p.dt+1:end),D_eval(1,m).ref_w(2,p.duration/p.dt+1:end),D_eval(1,m).ref_w(3,p.duration/p.dt+1:end),'y.');
    end
    xlabel('x')
    ylabel('y');
    zlabel('z');
    set(gca,'XDir','normal');  
    set(gca,'YDir','normal');  
    set(gca,'ZDir','normal');  

end


if strcmp(p.policy, 'GMR')
    if n_dim ==2
        my_plotGMM(model.muInput, model.sigmaInput, 'r', 1);
        scatter(model.muInput(1,:),model.muInput(2,:),[],'r')
    elseif n_dim ==3
        scatter3(model.muInput(1,:),model.muInput(2,:),model.muInput(3,:),[],'r')
    end
end

if n_dim == 2
    for i = 1:length(constraints)
        if strcmp(constraints(i).type, 'rectangle') 
            x = constraints(i).min_w1; 
            y = constraints(i).min_w2;
            w = constraints(i).max_w1 - constraints(i).min_w1;
            h = constraints(i).max_w2 - constraints(i).min_w2;
            rectangle('Position',[x y w h],'FaceColor','k')
        elseif strcmp(constraints(i).type, 'circle') 
            x = constraints(i).center1;
            y = constraints(i).center2;
            r = constraints(i).radius;
            rectangle('Position',[x-r,y-r,2*r,2*r],'Curvature',[1,1],'FaceColor','k')
        end
    end
else
    for i = 1:length(constraints)
         x = constraints(i).min_w1; 
         y = constraints(i).min_w2;
         z = constraints(i).min_w3;
         w = constraints(i).max_w1 - constraints(i).min_w1;
         l = constraints(i).max_w2 - constraints(i).min_w2;
         h = constraints(i).max_w3 - constraints(i).min_w3;
         PlotCuboid([x,y,z],[w,l,h])
    end
end

if n_dim == 2
    plot(goal(1), goal(2), 'h', 'Color', 'g', 'MarkerSize',15,'markerfacecolor','g');
    plot(start(1), start(2), '.', 'Color', 'y', 'MarkerSize',40);
    if size(viapoints,1) ~= 0
        plot(viapoints(:,1), viapoints(:,2), '.', 'Color', 'b', 'MarkerSize', 25);
    end
else
    plot3(goal(1), goal(2), goal(3), 'h', 'Color', '#D95319', 'MarkerSize',12,'markerfacecolor','#D95319');
    plot3(start(1), start(2), start(3), '.', 'Color', '#D95319', 'MarkerSize',20);
    if size(viapoints,1) ~= 0
        plot3(viapoints(:,1), viapoints(:,2), viapoints(:,3), '.', 'Color', '#D95319', 'MarkerSize', 15);
    end
end
% zlim([-0.09 -0.06]);
hold off

set(gca,'Fontsize',16,'Fontname','Time New Roman')


if p.dataset_type == 0
    set(gca,'XLim',[-11 1])
    set(gca,'YLim',[-2 2])
elseif p.dataset_type == 1
    set(gca,'XLim',[-11 11])
    set(gca,'YLim',[-10 10])
    axis equal
elseif p.dataset_type == 2
    if strcmp(p.demo_set, 'JShape')
        set(gca,'XLim',[-15 45])
        set(gca,'YLim',[-25 30])
        
    elseif strcmp(p.demo_set, 'Khamesh')
        set(gca,'XLim',[-55 5])
        set(gca,'YLim',[-40 5])
        
    elseif strcmp(p.demo_set, 'Leaf_1')
        set(gca,'XLim',[-20 15])
        set(gca,'YLim',[-3 52])
    elseif strcmp(p.demo_set, 'Spoon')
        set(gca,'XLim',[-52 8])
        set(gca,'YLim',[-25 10])
    end
    axis equal
else 
    error('Please the dataset type in the protocol!!!');
end
% drawnow;

% f = gcf;
if ~p.disable_saving
    saveas(gcf, strcat(folder_name, '/', class_name, '_path'), 'epsc')
    saveas(gcf, strcat(folder_name, '/', class_name, '_path.fig'))
end
end

function PlotCuboid(originPoint,cuboidSize)

vertexIndex=[0 0 0;0 0 1;0 1 0;0 1 1;1 0 0;1 0 1;1 1 0;1 1 1];
vertex=originPoint+vertexIndex.*cuboidSize;

facet=[1 2 4 3;1 2 6 5;1 3 7 5;2 4 8 6;3 4 8 7;5 6 8 7];

color=[1;2;3;4;5;6;7;8];
% color=['k';'k';'k';'k';'k';'k';'k';'k'];

patch('Vertices',vertex,'Faces',facet,'FaceVertexCData',color,'FaceColor','interp','FaceAlpha',0.5);
% axis equal
end

