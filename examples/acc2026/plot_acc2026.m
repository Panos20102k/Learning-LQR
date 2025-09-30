%% Load data
load('data/acc2026/opt.mat')
load('data/acc2026/algo.mat')
load('data/acc2026/pg.mat')
load('data/acc2026/rs.mat')
load('data/acc2026/q.mat')
%% Plot
figure
hold on
plot(1:algo.max_iter+1,algo.beta1_traj,'Color','r','LineWidth',2.5,...
    'DisplayName','$\beta_1$','Marker','o','LineStyle','--')
plot(1:algo.max_iter+1,-1.5*ones(algo.max_iter+1,1),'Color','r',...
    'LineWidth',2.5,'DisplayName','$\beta_1^{*}$','LineStyle','-')
hold off
xlabel('Iterations'); ylabel('\beta_1'); legend('Interpreter','latex')
axis tight
box on
ylim([-1.6 2])
%% clc complexity
clc = struct;
conv_idx = length(algo.Jrc);
clc.complexity = zeros(conv_idx,1);
clc.complexity(1) = 1; % initial cost has no complexity
for i = 2:conv_idx
    clc.complexity(i) = clc.complexity(i-1) + algo.complexity(i);
end
%% pg complexity
pg.complexity = numel(pg.costs); % make sure pg has converged
pg.trials = size(pg.costs,2);
%% rs complexity
rs.complexity = numel(rs.costs); % make sure rs has converged 
rs.trials = size(rs.costs,2);
%% q-learning complexity
q.complexity = numel(q.costs); % make sure q-learning has converged
q.trials = size(q.costs, 2); 
q.costs_plot = q.mean_costs;
q.complexity_plot = 1:q.trials:q.trials*length(q.costs_plot);
%% compare complexities
figure
hold on
plot(1:pg.trials:pg.complexity,pg.mean_costs,'Color','m',...
    'LineWidth',2.5,'DisplayName','PG','LineStyle','-')
plot(1:rs.trials:rs.complexity,rs.mean_costs,'Color','b',...
    'LineWidth',2.5,'DisplayName','RS','LineStyle',':')
plot(q.complexity_plot,q.costs_plot,'Color','g',...
    'LineWidth',2.5,'DisplayName','Q','LineStyle','-.')
plot(clc.complexity,algo.Jrc(1:conv_idx,1),'Color','r','LineWidth',2.5,...
    'LineStyle','--','Marker','o','DisplayName','CLC')
max_complexity = max(pg.complexity,max(clc.complexity));
plot(1:max_complexity,opt.cost*ones(max_complexity,1),'Color','k',...
    'LineWidth',2.5,'DisplayName','Optimal')
hold off
xlabel('Complexity'); ylabel('Cost'); legend('Interpreter','latex')
box on
axis tight
ylim([0.7 5])
