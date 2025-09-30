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
W = randn(2,1,T-1); % process noise
% Run params
batch_size = 16;
% Set trajectory budgets
budgets = [1 batch_size/8 batch_size/4 batch_size/2 batch_size 2*batch_size ...
     4*batch_size 8*batch_size 16*batch_size 32*batch_size 64*batch_size ...
     128*batch_size 256*batch_size 512*batch_size 1024*batch_size ...
     2048*batch_size 4096*batch_size 8192*batch_size 16384*batch_size ...
     32768*batch_size 65536*batch_size 2000000 5000000];
% budgets = [1 batch_size/8 batch_size/4 batch_size/2 batch_size 2*batch_size ...
%      4*batch_size 8*batch_size 16*batch_size 32*batch_size 64*batch_size ...
%      128*batch_size 256*batch_size];
trials = 10;
%% Load structures
opt = struct; q = struct;
%% Optimal control
% Get optimal law
[opt.law, ~] = get_law_with_DP(A, B, Q, R, T);
% Sample optimal cost
opt.cost = get_cost_with_law(A, B, Q, R, X0, T, W, opt.law);
% Get optimal expected cost
samples = 10000;
costs = zeros(samples,1);
for s = 1:samples
    W = randn(2,1,T-1);
    costs(s) = get_cost_with_law(A,B,Q,R,X0,T,W,opt.law);
end
opt.exp_cost = mean(costs);
%% Save costs
save("data/noisy_2d-vehicle/opt.mat","opt")
%% Set Q-learning params
q.budgets = budgets; q.budgets_len = length(q.budgets);
q.trials = trials;
q.costs = zeros(q.budgets_len,q.trials);
q.epsilon = 0.01;
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
%% Save costs
save("data/noisy_2d-vehicle/q.mat","q")
