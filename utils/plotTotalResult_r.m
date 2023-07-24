function plotTotalResult_r(results)
%PLOTTOTALRESULT Plot the mean and std of cost evolution for all runs

n_p = size(results,1);
n_runs = size(results,2);
for cnt1 = 1:n_p
    for cnt2 = 1:n_runs
        costEvoAllRuns(:,cnt1,cnt2) = results(cnt1,cnt2).cost(:,2);
    end
end

for cnt1 = 1:n_p
    costEvoRolloutIndex = results(cnt1,1).cost(:,1);
    costEvoMean = mean(costEvoAllRuns(:,cnt1,:),3);
    
    costEvoStd = std(costEvoAllRuns(:,cnt1,:),0,3);
%     size(costEvoStd)
    figure
    hold on
    plot(costEvoRolloutIndex, costEvoMean,'LineWidth',1);
    if n_runs ~=1
        plot(costEvoRolloutIndex, costEvoMean+costEvoStd);
        plot(costEvoRolloutIndex, costEvoMean-costEvoStd);
        s = strcat('Mean and std of cost evolution for', int2str(n_runs),' runs');
    else
        plot(costEvoRolloutIndex, costEvoMean);
        s = strcat('Mean and std of cost evolution for one run');
    end
    title(s)
    hold off

    figure
    hold on
    for cnt3=1:n_runs
        scatter(1,results(cnt1,cnt3).cost(end,2))
    end

end
end

