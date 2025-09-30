%% Add path
addpath(genpath(pwd));
%% Set global params
% System params
real = struct;
model = struct;
T = 3; % decision stages
x0 = 0.5; % initial state
real.A = 2; % system dynamics 
real.B = 1; % system dynamics 
model.A = 1; % model dynamics 
model.B = 1; % model dynamics 
Q = 1; % cost function
R = 1; % cost function
% Load parameters in combatible objects for functions
A_real = real.A*ones(1,1,T-1);
B_real = real.B*ones(1,1,T-1);
A_model = model.A*ones(1,1,T-1);
B_model = model.B*ones(1,1,T-1);
Q_ = Q*ones(1,1,T);
Q_(1,1,1) = 0;
R_ = R*ones(1,1,T-1);
W = zeros(1,1,T-1); % process noise
%% Optimal control
opt = struct;
% Get law
[opt.law, ~] = get_law_with_DP(A_real, B_real, Q_, R_, T);
% Get cost
opt.cost = get_cost_with_law(A_real, B_real, Q_, R_, x0, T, W, opt.law);
%% Save
save("data/acc2026/opt.mat","opt")
%% Algorithm, learning beta (Q=R=1)
algo = struct;
algo.max_iter = 20; algo.heta1 = 1;algo.heta2 = 1; algo.delta = 0.5; 
algo.beta2 = -Q; algo.beta1 = 2; algo.beta1_traj = zeros(algo.max_iter+1,1);
algo.beta2_traj = zeros(algo.max_iter+1,1);
algo.beta1_traj(1) = algo.beta1; algo.beta2_traj(1) = algo.beta2;
algo.Jrc = zeros(algo.max_iter,3);
algo.complexity = zeros(algo.max_iter,1);
f = waitbar(0, 'Starting');
for iter = 1:algo.max_iter
    algo.beta1_old = algo.beta1;
    algo.beta2_old = algo.beta2;
    for s = 1:2
        if s == 2
            algo.beta1 = algo.beta1 - algo.delta;
        elseif s == 3
            algo.beta2 = algo.beta2 + algo.delta;
            algo.beta1 = algo.beta1_old;
        end
        algo.x1_real_p_grid = (-0.25:0.025:1)''; algo.x2_real_p_grid = (-0.25:0.025:1)'';
        algo.x0 = x0; algo.x1_grid = (-1:0.025:0.8)'; algo.x2_grid = (-1:0.025:0.8)';
        algo.u0_grid = (-1:0.025:0.05)'; algo.u1_grid = (-1:0.025:0.05)';
        algo.x1_real_p_grid_len = length(algo.x1_real_p_grid);
        algo.x2_real_p_grid_len = length(algo.x2_real_p_grid);
        algo.x1_len = length(algo.x1_grid); algo.x2_len = length(algo.x2_grid);
        algo.u0_len = length(algo.u0_grid); algo.u1_len = length(algo.u1_grid);
        % initialize 
        algo.u0 = zeros(algo.x1_real_p_grid_len,algo.x2_real_p_grid_len);
        algo.u1 = zeros(algo.x1_len,algo.x1_real_p_grid_len,algo.x2_real_p_grid_len);
        % DP
        for x2_real_p_idx = 1:algo.x2_real_p_grid_len
            x2_real_p = algo.x2_real_p_grid(x2_real_p_idx);
            for x1_real_p_idx = 1:algo.x1_real_p_grid_len
                x1_real_p = algo.x1_real_p_grid(x1_real_p_idx);
                % initialize
                V2 = zeros(algo.x2_len,1);
                V1 = zeros(algo.x1_len,1);
                for x2_idx = 1:algo.x2_len
                    x2 = algo.x2_grid(x2_idx);
                    V2(x2_idx) = x2^2 + algo.beta2*(x2-x2_real_p)^2;
                end
                for x1_idx = 1:algo.x1_len
                    x1 = algo.x1_grid(x1_idx);
                    J1 = zeros(algo.u1_len,1);
                    for u1_idx = 1:algo.u1_len
                        u1 = algo.u1_grid(u1_idx);
                        [~,x2_idx] = min(abs(algo.x2_grid-model.A*x1-model.B*u1));
                        J1(u1_idx) = x1^2+algo.beta1*(x1-x1_real_p)^2+...
                            u1^2+V2(x2_idx);
                    end
                    [V1(x1_idx), u1_idx] = min(J1);
                    algo.u1(x1_idx,x1_real_p_idx,x2_real_p_idx) = algo.u1_grid(u1_idx);
                end
                J0 = zeros(algo.u0_len,1);
                for u0_idx = 1:algo.u0_len
                    u0 = algo.u0_grid(u0_idx);
                    [~,x1_idx] = min(abs(algo.x1_grid-model.A*x0-model.B*u0));
                    J0(u0_idx) = u0^2+V1(x1_idx);
                end
                [V0, u0_idx] = min(J0);
                algo.u0(x1_real_p_idx,x2_real_p_idx) = algo.u0_grid(u0_idx);
            end
        end
        % Finding correct real states
        tol = 0.06;
        converged = false;
        for u0_idx = 1:algo.u0_len
            if converged == true
                break
            end
            u0 = algo.u0_grid(u0_idx);
            for u1_idx = 1:algo.u1_len
                u1 = algo.u1_grid(u1_idx);
                x1_real = real.A*algo.x0+real.B*u0;
                x2_real = real.A*x1_real+real.B*u1;
                algo.complexity(iter) = algo.complexity(iter)+1;
                [~,x1_real_p_idx] = min(abs(algo.x1_real_p_grid-x1_real));
                [~,x2_real_p_idx] = min(abs(algo.x2_real_p_grid-x2_real));
                u0_temp = algo.u0(x1_real_p_idx,x2_real_p_idx);
                x1 = model.A*algo.x0+model.B*u0_temp;
                [~,x1_idx] = min(abs(algo.x1_grid-x1));
                u1_temp = algo.u1(x1_idx,x1_real_p_idx,x2_real_p_idx);
                if abs(u0-u0_temp) <= tol && abs(u1-u1_temp) <= tol
                    algo.u0_opt = u0_temp;
                    algo.u1_opt = u1_temp;
                    converged = true;
                    break
                end
            end
        end
        % apply opt controls to real
        x1_real_opt = real.A*x0 + real.B*algo.u0_opt;
        x2_real_opt = real.A*x1_real_opt + real.B*algo.u1_opt;
        algo.complexity(iter) = algo.complexity(iter)+1;
        algo.Jrc(iter,s) = algo.u0_opt^2+x1_real_opt^2+algo.u1_opt^2+...
            x2_real_opt^2;
    end
    % get gradient estimate
    algo.grad_1 = (algo.Jrc(iter,1)-algo.Jrc(iter,2))/algo.delta;
    % algo.grad_2 = (algo.Jrc(iter,3)-algo.Jrc(iter,1))/algo.delta;
    % step gradient descent
    algo.beta1 = algo.beta1_old - algo.heta1*algo.grad_1;
    % algo.beta2 = algo.beta2_old - algo.heta2*algo.grad_2;
    algo.beta1_traj(iter+1) = algo.beta1;
    % algo.beta2_traj(iter+1) = algo.beta2;
    waitbar(iter/algo.max_iter, f, sprintf("Progress: %d %%", ...
            floor(iter/algo.max_iter*100)));
end
close(f)
%% Plot
figure
hold on
plot(1:algo.max_iter+1,algo.beta1_traj,'Color','r','LineWidth',2.5,...
    'DisplayName','$\beta_1$','Marker','o','LineStyle','--')
plot(1:algo.max_iter+1,-1.5*ones(algo.max_iter+1,1),'Color','r',...
    'LineWidth',2.5,'DisplayName','$\beta_1*$','LineStyle','-')
hold off
xlabel('iterations'); ylabel('\beta_1'); legend('Interpreter','latex')
title(sprintf('A_{hat} = %2.1f, B_{hat} = %2.1f, A = %2.1f, B = %2.1f',...
    real.A,real.B,model.A,model.B))
%% Plot
figure
plot(1:algo.max_iter,algo.complexity,'Color','b','LineWidth',2.5,...
    'DisplayName','$\beta_1$','Marker','o','LineStyle','--')
%% Save
save('data/acc2026/algo.mat','algo')
%% Load structures
pg = struct; q = struct; rs = struct;
%% Run PG
pg.budgets = 1:1:200; pg.budgets_len = length(pg.budgets);
pg.trials = 1:115; pg.trials_len = length(pg.trials);
pg.costs = zeros(pg.budgets_len+1,pg.trials_len);
pg.K_0 = -0.5; % Initial K (must be stabilizing!)
pg.costs(1,:) = get_cost_with_law(A_real,B_real,Q_,R_,x0,T,W,pg.K_0);
pg.alpha = 0.01;
pg.sigma = 0.1;
f = waitbar(0, 'Starting');
for budget_idx = 1:pg.budgets_len
    % Get current budget
    budget = pg.budgets(budget_idx);
    for trial_idx = 1:pg.trials_len
        law_pg = get_law_with_PG(A_real,B_real,Q_,R_,x0,T,W,pg.K_0,budget,1,...
                    pg.alpha,pg.sigma);
        cost_pg = get_cost_with_law(A_real,B_real,Q_,R_,x0,T,W,law_pg);
        pg.costs(budget_idx+1,trial_idx) = cost_pg;
    end
    waitbar(budget_idx/length(pg.budgets), f, sprintf("Progress: %d %%", ...
            floor(budget_idx/length(pg.budgets)*100)));
end
close(f)
pg.mean_costs = mean(pg.costs,2);
%% Plot
figure
hold on
plot(1:pg.budgets_len+1,pg.mean_costs,'Color','m','LineWidth',2.5,...
    'LineStyle','-','DisplayName','PG')
plot(1:pg.budgets_len+1,opt.cost*ones(pg.budgets_len+1,1),'Color','y','LineWidth',2.5,...
    'DisplayName','optimal')
hold off
xlabel('complexity'); ylabel('cost'); legend('Interpreter','latex')
%% Save 
save('data/acc2026/pg.mat','pg')
%% Run RS
rs.budgets = 1:1:200; rs.budgets_len = length(rs.budgets);
rs.trials = 1:115; rs.trials_len = length(rs.trials);
rs.costs = zeros(rs.budgets_len+1,rs.trials_len);
rs.K_0 = -0.5; % Initial K (must be stabilizing!)
rs.costs(1,:) = get_cost_with_law(A_real,B_real,Q_,R_,x0,T,W,rs.K_0);
rs.alpha = 0.01;
rs.sigma = 0.1;
rs.batch_size = 1;
f = waitbar(0, 'Starting');
for budget_idx = 1:rs.budgets_len
    % Get current budget
    budget = rs.budgets(budget_idx);
    for trial_idx = 1:rs.trials_len
        law_rs = get_law_with_RS(A_real,B_real,Q_,R_,x0,T,W,rs.K_0,...
            budget,rs.batch_size,rs.alpha,rs.sigma);
        cost_rs = get_cost_with_law(A_real,B_real,Q_,R_,x0,T,W,law_rs);
        rs.costs(budget_idx+1,trial_idx) = cost_rs;
    end
    waitbar(budget_idx/length(rs.budgets), f, sprintf("Progress: %d %%", ...
            floor(budget_idx/length(rs.budgets)*100)));
end
close(f)
rs.mean_costs = mean(rs.costs,2);
%% Plot
figure
hold on
plot(1:rs.budgets_len+1,rs.mean_costs,'Color','m','LineWidth',2.5,...
    'LineStyle','-','DisplayName','RS')
plot(1:rs.budgets_len+1,opt.cost*ones(rs.budgets_len+1,1),'Color','y','LineWidth',2.5,...
    'DisplayName','optimal')
hold off
xlabel('complexity'); ylabel('cost'); legend('Interpreter','latex')
%% Save 
save('data/acc2026/rs.mat','rs')
%% Run Q-learning
q.budgets = 1:200; q.budgets_len = length(q.budgets);
q.trials = 1:115; q.trials_len = length(q.trials);
q.costs = zeros(q.budgets_len+1,q.trials_len);
q.epsilon = 0.1;
% State and control space bounds
q.X_min = -0.25; q.X_max = 0.5;
q.U_min = -0.95; q.U_max = 0.05;
% Number of states and controls
q.n_X = 50; 
q.n_U = 10;
Q_table_0 = zeros(q.n_X, q.n_U); 
q.costs(1,:) = get_cost_with_Q_table_1d(A_real,B_real,Q_,R_,x0,T,W,...
    q.X_min,q.X_max,q.U_min,q.U_max,q.n_X,q.n_U,Q_table_0);
f = waitbar(0, 'Starting');
for budget_idx = 1:q.budgets_len
    % Get current budget
    budget = q.budgets(budget_idx);
    for trial_idx = 1:q.trials_len
        Q_table = get_Q_table_1d(A_real,B_real,Q_,R_,x0,T,W,budget,...
            q.epsilon,q.X_min,q.X_max,q.U_min,q.U_max,q.n_X,q.n_U);
        cost_q = get_cost_with_Q_table_1d(A_real,B_real,Q_,R_,x0,T,W,...
            q.X_min,q.X_max,q.U_min,q.U_max,q.n_X,q.n_U,Q_table);
        q.costs(budget_idx+1,trial_idx) = cost_q;
    end
    waitbar(budget_idx/length(q.budgets), f, sprintf("Progress: %d %%", ...
            floor(budget_idx/length(q.budgets)*100)));
end
close(f)
q.mean_costs = mean(q.costs,2);
%% Plot
figure
hold on
plot(1:length(q.mean_costs),q.mean_costs,'Color','y','LineWidth',2.5,...
    'LineStyle','-','DisplayName','Q-learning')
plot(1:length(q.mean_costs),opt.cost*ones(length(q.mean_costs),1),'Color','g','LineWidth',2.5,...
    'DisplayName','optimal')
hold off
xlabel('complexity'); ylabel('cost'); legend('Interpreter','latex')
%% Save q-learning
save('data/acc2026/q.mat','q')