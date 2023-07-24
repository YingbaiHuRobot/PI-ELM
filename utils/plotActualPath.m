function  plotActualPath(D_eval)
%%% plot 2D graph of point mass
figure;
hold on
plot(D_eval.q(1,:),D_eval.q(2,:));
xlabel('q_1')
ylabel('q_2');
title('2D path of point mass');
hold off

end

