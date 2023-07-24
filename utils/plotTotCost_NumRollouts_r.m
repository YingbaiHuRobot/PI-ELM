function  plotTotCost_NumRollouts_r(p, result)

figure;
hold on
plot(result.cost(:,1),result.cost(:,3),'r','LineWidth',2); % Path length
plot(result.cost(:,1),result.cost(:,4),'c','LineWidth',2); % viapoint cost
plot(result.cost(:,1),result.cost(:,5),'g','LineWidth',2); % acceleration cost
if p.stiffness_learning == 1
plot(result.cost(:,1),result.cost(:,6),'m','LineWidth',2); % stiffness cost
end
plot(result.cost(:,1),result.cost(:,2),'b','LineWidth',2);   % total cost
if p.stiffness_learning == 1
    legend({'Path length', 'Via-points', 'Acceleration', 'Stiffness','Total cost'});
else
    legend({'Path length', 'Via-points', 'Acceleration','Total cost'});
end
legend('position',[0.42,0.58,0.45,0.3],'Fontsize',16,'Fontname','Time New Roman','Box','off')
xlabel('Roll-outs');
ylabel('Cost');
hold off
set(gca,'Fontsize',16,'Fontname','Time New Roman')
set(gca, 'XLim', [0 result.cost(end,1)])

end

