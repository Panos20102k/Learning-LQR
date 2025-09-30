%% Load 
load('data/noisy_2d-vehicle/opt.mat')
load('data/noisy_2d-vehicle/q.mat')
%% Plot Q-learning
% Plot confidence intervals of Q-learning
costs_q_learning_max = max(q.costs,[],2);
costs_q_learning_mean = mean(q.costs,2);
costs_q_learning_min = min(q.costs,[],2);
% Set index of first non NaN entry of costs.q_learning
% (if all are NaNs set to final row number)
index_q = 1;
x_q = q.budgets(index_q:end);                     
xconf_q = [x_q x_q(end:-1:1)];         
yconf_q = [costs_q_learning_max(index_q:end)' costs_q_learning_min(end:-1:index_q)'];
figure
p_q = fill(xconf_q,yconf_q,'blue', 'DisplayName','Q-learning');
p_q.FaceColor = [0.8 0.8 1];      
p_q.EdgeColor = 'none';       
hold on
% Plot Q-learning mean costs
plot(x_q,costs_q_learning_mean(index_q:end),'blue', 'HandleVisibility', 'off')
% Plot Optimal cost
plot(q.budgets, opt.cost*ones(1,length(q.budgets)), 'DisplayName','Optimal', 'Color', 'k')
set(gca, 'XScale', 'log'); % make x-axis logarithmic
set(gca, 'YScale', 'log'); % make y-axis logarithmic
% xlim([0 200000])
%ylim([1 5])
legend
xlabel("Samples")
ylabel("Cost")
%title("Costs on LQR")
hold off