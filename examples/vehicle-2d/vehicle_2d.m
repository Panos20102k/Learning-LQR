%% Add path
addpath(genpath(pwd));
%% Set global params
% System params
T = 10; % decision stages
X0 = [-1; 0]; % initial state
% Load parameters in compatible objects for functions
A = ones(2,2,T-1); % system dynamics
A(2,1,:) = 0;
B = zeros(2,1,T-1); % system dynamics 
B(2,1,:) = 1;
Q = zeros(2,2,T); % cost function
Q(1,1,2:end) = 1;
R = ones(1,1,T-1); % cost function 
W = zeros(1,1,T-1); % process noise
% Run params
batch_size = 16;
% budgets = [1 batch_size/8 batch_size/4 batch_size/2 batch_size 2*batch_size ...
%      4*batch_size 8*batch_size 16*batch_size 32*batch_size 64*batch_size ...
%      128*batch_size 256*batch_size 512*batch_size 1024*batch_size ...
%      2048*batch_size 4096*batch_size 8192*batch_size 16384*batch_size ...
%      32768*batch_size 65536*batch_size 2000000 5000000];
budgets = [1 batch_size/8 batch_size/4 batch_size/2 batch_size 2*batch_size ...
     4*batch_size 8*batch_size 16*batch_size 32*batch_size 64*batch_size ...
     128*batch_size 256*batch_size 512*batch_size 1024*batch_size ...
     2048*batch_size 4096*batch_size 8192*batch_size];
trials = 10;
%% Load structures
opt = struct; pg = struct; q = struct; rs = struct;
%% Optimal control
% Get optimal law
[opt.law, ~] = get_law_with_DP(A, B, Q, R, T);
% Get optimal cost
opt.cost = get_cost_with_law(A, B, Q, R, X0, T, W, opt.law);
%% Save 
save("data/2d-vehicle/opt.mat","opt")
%% Set PG params
pg.batch_size = batch_size;
pg.budgets=budgets;
pg.budgets_len = length(pg.budgets);
pg.trials = trials; 
pg.costs = zeros(pg.budgets_len,pg.trials);
pg.K0 = -[0.8 1.8]; % Initial K (must be stabilizing!)
pg.alpha = 0.01;
pg.sigma = 0.1;
%% Run PG
f = waitbar(0, 'Starting');
for budget_index = 1:pg.budgets_len
    % Get current budget
    budget = pg.budgets(budget_index);
    for trial = 1:pg.trials
        if budget == 1
            % Get law with just 1 trajectory. No batching.
            law_pg = get_law_with_PG(A,B,Q,R,X0,T,W,pg.K0,budget,1,...
                pg.alpha,pg.sigma);
            % Get costs with law
            pg.costs(budget_index,trial) = get_cost_with_law(A,B,Q,R,X0,...
                T,W,law_pg);
        end
        if budget == pg.batch_size/8
            % Get law with 1 update and batch_size/8 batch size
            law_pg = get_law_with_PG(A,B,Q,R,X0,T,W,pg.K0,1,pg.batch_size/8,...
                pg.alpha,pg.sigma);
            % Get costs with law
            pg.costs(budget_index,trial) = get_cost_with_law(A,B,Q,R,X0,...
                T,W,law_pg);
        end
        if budget == pg.batch_size/4
            % Get law with 1 update and batch_size/4 batch size
            law_pg = get_law_with_PG(A,B,Q,R,X0,T,W,pg.K0,1,pg.batch_size/4,...
                pg.alpha,pg.sigma);
            % Get costs with law
            pg.costs(budget_index,trial) = get_cost_with_law(A,B,Q,R,X0,...
                T,W,law_pg);
        end
        if budget == pg.batch_size/2
            % Get law with 1 update and batch_size/2 batch size
            law_pg = get_law_with_PG(A,B,Q,R,X0,T,W,pg.K0,1,pg.batch_size/2,...
                pg.alpha,pg.sigma);
            % Get costs with law
            pg.costs(budget_index,trial) = get_cost_with_law(A,B,Q,R,X0,...
                T,W,law_pg);
        end
        if budget >= pg.batch_size
            % Get law with budget/batch_size updates and batch_size batch size
            law_pg = get_law_with_PG(A,B,Q,R,X0,T,W,pg.K0,budget/pg.batch_size,...
                batch_size,pg.alpha,pg.sigma);
            % Get costs with law
            pg.costs(budget_index,trial) = get_cost_with_law(A,B,Q,R,X0,...
                T,W,law_pg);
        end
    end
    waitbar(budget_index/length(pg.budgets), f, sprintf("Progress: %d %%", ...
        floor(budget_index/length(pg.budgets)*100)));
end
close(f)
%% Save
save("data/2d-vehicle/pg.mat","pg")
%% Set RS params
rs.batch_size = batch_size;
rs.budgets=budgets;
rs.budgets_len = length(rs.budgets);
rs.trials = trials; 
rs.costs = zeros(rs.budgets_len,rs.trials);
rs.K0 = -[0.8 1.8]; % Initial K (must be stabilizing!)
rs.alpha = 0.01;
rs.sigma = 0.1;
%% Run RS
f = waitbar(0, 'Starting');
for budget_index = 1:length(rs.budgets)
    % Get current budget
    budget = rs.budgets(budget_index);
    for trial = 1:rs.trials
        if budget == 1
            % RS needs at least 2 budgets
            rs.costs(budget_index,trial) = NaN;
        end
        if budget == rs.batch_size/8
            % Get law with 1 update and batch_size/16 batch size
            law_rs = get_law_with_RS(A,B,Q,R,X0,T,W,rs.K0,1,rs.batch_size/16,...
                rs.alpha,rs.sigma);
            % Get costs with law
            rs.costs(budget_index,trial) = get_cost_with_law(A,B,Q,R,X0,...
                T,W,law_rs);
        end
        if budget == batch_size/4
            % Get law with 1 update and batch_size/8 batch size
            law_rs = get_law_with_RS(A,B,Q,R,X0,T,W,rs.K0,1,batch_size/8,...
                rs.alpha,rs.sigma);
            % Get costs with law
            rs.costs(budget_index,trial) = get_cost_with_law(A,B,Q,R,X0,...
                T,W,law_rs);
        end
        if budget == batch_size/2
            % Get law with 1 update and batch_size/4 batch size
            law_rs = get_law_with_RS(A,B,Q,R,X0,T,W,rs.K0,1,batch_size/4,...
                rs.alpha,rs.sigma);
            % Get costs with law
            rs.costs(budget_index,trial) = get_cost_with_law(A,B,Q,R,X0,...
                T,W,law_rs);
        end
        if budget >= batch_size
            % Get law with budget/batch_size update and batch_size/2 batch size
            law_rs = get_law_with_RS(A,B,Q,R,X0,T,W,rs.K0,budget/batch_size,...
                batch_size/2,rs.alpha,rs.sigma);
            % Get costs with law
            rs.costs(budget_index,trial) = get_cost_with_law(A,B,Q,R,X0,...
                T,W,law_rs);
        end
    end
    waitbar(budget_index/length(rs.budgets), f, sprintf("Progress: %d %%", ...
        floor(budget_index/length(rs.budgets)*100)));
end
close(f)
%% Save
save("data/2d-vehicle/rs.mat","rs")
%% Set Q-learning params
q.budgets = budgets; q.budgets_len = length(q.budgets);
q.trials = trials;
q.costs = zeros(q.budgets_len,q.trials);
q.epsilon = 0.1;
% State and control space bounds
q.X1_min = -4; q.X1_max = 4;
q.X2_min = -4; q.X2_max = 4;
q.U_min = -4; q.U_max = 5;
% Number of states and controls
q.n_X1 = 100; 
q.n_X2 = 100;
q.n_U = 100;
%% Run Q-learning
f = waitbar(0, 'Starting');
for budget_index = 1:length(q.budgets)
    % Get current budget
    budget = q.budgets(budget_index);
    for trial = 1:q.trials
        if budget == 1
            % Get law with just 1 trajectory. No batching.
            Q_table = get_Q_table_2d(A,B,Q,R,X0,T,W,budget,q.epsilon,q.X1_min,...
                q.X1_max,q.X2_min,q.X2_max,q.U_min,q.U_max,q.n_X1,q.n_X2,q.n_U);
            % Get costs with law
            q.costs(budget_index,trial) = get_cost_with_Q_table_2d(...
                A,B,Q,R,X0,T,W,q.X1_min,q.X1_max,q.X2_min,q.X2_max,q.U_min,q.U_max,...
                q.n_X1,q.n_X2,q.n_U,Q_table);
        end
        if budget == batch_size/8
            % Get Q-table with budget updates
            Q_table = get_Q_table_2d(A,B,Q,R,X0,T,W,budget,q.epsilon,q.X1_min,...
                q.X1_max,q.X2_min,q.X2_max,q.U_min,q.U_max,q.n_X1,q.n_X2,q.n_U);
            % Get costs with law
            q.costs(budget_index,trial) = get_cost_with_Q_table_2d(...
                A,B,Q,R,X0,T,W,q.X1_min,q.X1_max,q.X2_min,q.X2_max,q.U_min,q.U_max,...
                q.n_X1,q.n_X2,q.n_U,Q_table);
        end
        if budget == batch_size/4
            % Get Q-table with budget updates
            Q_table = get_Q_table_2d(A,B,Q,R,X0,T,W,budget,q.epsilon,q.X1_min,...
                q.X1_max,q.X2_min,q.X2_max,q.U_min,q.U_max,q.n_X1,q.n_X2,q.n_U);
            % Get costs with law
            q.costs(budget_index,trial) = get_cost_with_Q_table_2d(...
                A,B,Q,R,X0,T,W,q.X1_min,q.X1_max,q.X2_min,q.X2_max,q.U_min,q.U_max,...
                q.n_X1,q.n_X2,q.n_U,Q_table);
        end
        if budget == batch_size/2
            % Get Q-table with budget updates
            Q_table = get_Q_table_2d(A,B,Q,R,X0,T,W,budget,q.epsilon,q.X1_min,...
                q.X1_max,q.X2_min,q.X2_max,q.U_min,q.U_max,q.n_X1,q.n_X2,q.n_U);
            % Get costs with law
            q.costs(budget_index,trial) = get_cost_with_Q_table_2d(...
                A,B,Q,R,X0,T,W,q.X1_min,q.X1_max,q.X2_min,q.X2_max,q.U_min,q.U_max,...
                q.n_X1,q.n_X2,q.n_U,Q_table);
        end
        if budget >= batch_size
            % Get Q-table with budget updates
            Q_table = get_Q_table_2d(A,B,Q,R,X0,T,W,budget,q.epsilon,q.X1_min,...
                q.X1_max,q.X2_min,q.X2_max,q.U_min,q.U_max,q.n_X1,q.n_X2,q.n_U);
            % Get costs with law
            q.costs(budget_index,trial) = get_cost_with_Q_table_2d(...
                A,B,Q,R,X0,T,W,q.X1_min,q.X1_max,q.X2_min,q.X2_max,q.U_min,q.U_max,...
                q.n_X1,q.n_X2,q.n_U,Q_table);
        end
    end
    waitbar(budget_index/length(q.budgets), f, sprintf("Progress: %d %%", ...
        floor(budget_index/length(q.budgets)*100)));
end
close(f)
%% Save
save("data/2d-vehicle/q.mat","q")