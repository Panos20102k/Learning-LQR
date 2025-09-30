function cost = get_cost_with_Q_table(A, B, Q, R, X_0, T, W, ...
    X_min, X_max, U_min, U_max, n_X, n_U, Q_table)

% Initialize optimal control strategy
opt_strag = zeros(n_X);
% Get optimal control strategy
for i = 1:n_X
    [~, opt_strag(i)] = min(Q_table(i,:));
end

% Discrete state and control values
X_vals = linspace(X_min, X_max, n_X);
U_vals = linspace(U_min, U_max, n_U);
% Initial state
% [~, X1_index_0] = min(abs(X1_vals + X_0(1))); 
% [~, X2_index_0] = min(abs(X2_vals + X_0(2))); 
% X_index_0 = [X1_index_0; X2_index_0];
[~, X_index_0] = min(abs(X_vals + X_0)); 
% Set initial state
X_index = X_index_0;
% Set real initial state 
X = X_0;
% Initialize total cost
cost = 0;
% Run real system with this strategy
for t = 1:T-1
    % Get control
    U = U_vals(opt_strag(X_index));
    % Get one-step cost
    cost = cost + X'*Q(:,:,t)*X + U'*R(:,:,t)*U;
    % Get system evolution
    X = A(:,:,t)*X + B(:,:,t)*U + W(:,:,t);
    % Discretize resulting state
    % [~, X1_index] = min(abs(X1_vals - X(1)));
    % [~, X2_index] = min(abs(X2_vals - X(2)));
    % X_index = [X1_index; X2_index];
    [~, X_index] = min(abs(X_vals - X));
end
% Get final cost
cost = cost + X'*Q(:,:,T)*X;
