function plotkp(p,kp_matrix, folder_name, class_name)

figure
plot(kp_matrix(1,:),'b','LineWidth',2);
xlabel('Time Step');
ylabel('s_1');
set(gca,'Fontsize',16,'Fontname','Time New Roman')

if ~p.disable_saving
    saveas(gcf, strcat(folder_name, '/', class_name, '_kp1'), 'epsc')
    saveas(gcf, strcat(folder_name, '/', class_name, '_kp1.fig'))
end

figure
plot(kp_matrix(2,:),'b','LineWidth',2);
xlabel('Time Step');
ylabel('s_2');
set(gca,'Fontsize',16,'Fontname','Time New Roman')

if ~p.disable_saving
    saveas(gcf, strcat(folder_name, '/', class_name, '_kp2'), 'epsc')
    saveas(gcf, strcat(folder_name, '/', class_name, '_kp2.fig'))
end
end

