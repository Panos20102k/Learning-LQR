%% Load 
load('data/2d-vehicle/opt.mat')
load('data/2d-vehicle/pg.mat')
load('data/2d-vehicle/rs.mat')
load('data/2d-vehicle/q.mat')
%% Plot pg
% Plot confidence intervals of PG
costs_pg_max = max(pg.costs,[],2);
costs_pg_mean = mean(pg.costs,2);
costs_pg_min = min(pg.costs,[],2);
% Set index of first non NaN entry of pg.costs
% (if all are NaNs set to final row number)
index_pg = 1;
x_pg = pg.budgets(index_pg:end);                     
xconf_pg = [x_pg x_pg(end:-1:1)] ;         
yconf_pg = [costs_pg_max(index_pg:end)' costs_pg_min(end:-1:index_pg)'];
figure
p_pg = fill(xconf_pg,yconf_pg,'red', 'DisplayName','Policy Gradient');
p_pg.FaceColor = [1 0.8 0.8];      
p_pg.EdgeColor = 'none';       
hold on
% Plot Policy Gradient mean costs
plot(x_pg,costs_pg_mean(index_pg:end),'r', 'HandleVisibility', 'off')
plot(pg.budgets, opt.cost*ones(1,length(pg.budgets)), 'DisplayName','Optimal', 'Color', 'k')
set(gca, 'XScale', 'log'); % make x-axis logarithmic
set(gca, 'YScale', 'log'); % make y-axis logarithmic
legend
xlabel("Complexity")
ylabel("Cost")
hold off
%% Plot rs
% Plot confidence intervals of rs
costs_rs_max = max(rs.costs,[],2);
costs_rs_mean = mean(rs.costs,2);
costs_rs_min = min(rs.costs,[],2);
% Set index of first non NaN entry of rs.costs
% (if all are NaNs set to final row number)
index_rs = 2;
x_rs = rs.budgets(index_rs:end);                     
xconf_rs = [x_rs x_rs(end:-1:1)];         
yconf_rs = [costs_rs_max(index_rs:end)' costs_rs_min(end:-1:index_rs)'];
figure
p_rs = fill(xconf_rs,yconf_rs,'green', 'DisplayName','Random Search');
p_rs.FaceColor = [0.8 1 0.8];        
p_rs.EdgeColor = 'none';       
hold on
% Plot Random Search mean costs
plot(x_rs,costs_rs_mean(index_rs:end),'green', 'HandleVisibility', 'off')
plot(rs.budgets, opt.cost*ones(1,length(rs.budgets)), 'DisplayName','Optimal', 'Color', 'k')
set(gca, 'XScale', 'log'); % make x-axis logarithmic
set(gca, 'YScale', 'log'); % make y-axis logarithmic
legend
xlabel("Complexity")
ylabel("Cost")
hold off
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
%% Plot all
figure
p_pg = fill(xconf_pg,yconf_pg,'red', 'DisplayName','Policy Gradient');
p_pg.FaceColor = [1 0.8 0.8];      
p_pg.EdgeColor = 'none';       
hold on
% Plot Policy Gradient mean costs
plot(x_pg,costs_pg_mean(index_pg:end),'r', 'HandleVisibility', 'off')
plot(pg.budgets, opt.cost*ones(1,length(pg.budgets)), 'DisplayName','Optimal', 'Color', 'k')
p_rs = fill(xconf_rs,yconf_rs,'green', 'DisplayName','Random Search');
p_rs.FaceColor = [0.8 1 0.8];        
p_rs.EdgeColor = 'none';       
hold on
% Plot Random Search mean costs
plot(x_rs,costs_rs_mean(index_rs:end),'green', 'HandleVisibility', 'off')
p_q = fill(xconf_q,yconf_q,'blue', 'DisplayName','Q-learning');
p_q.FaceColor = [0.8 0.8 1];      
p_q.EdgeColor = 'none';       
hold on
% Plot Q-learning mean costs
plot(x_q,costs_q_learning_mean(index_q:end),'blue', 'HandleVisibility', 'off')
set(gca, 'XScale', 'log'); % make x-axis logarithmic
set(gca, 'YScale', 'log'); % make y-axis logarithmic
legend
xlabel("Complexity")
ylabel("Cost")
hold off
