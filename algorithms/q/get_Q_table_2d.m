function Q_table = get_Q_table(A, B, Q, R, X_0, T, W, episodes, epsilon, ...
    X1_min, X1_max, X2_min, X2_max, U_min, U_max, n_X1, n_X2, n_U)

% Discrete state and control values
X1_vals = linspace(X1_min, X1_max, n_X1);
X2_vals = linspace(X2_min, X2_max, n_X2);
U_vals = linspace(U_min, U_max, n_U);

% Initial state
[~, X1_index_0] = min(abs(X1_vals + X_0(1))); 
[~, X2_index_0] = min(abs(X2_vals + X_0(2))); 
X_index_0 = [X1_index_0; X2_index_0];

% Initialize Q-table
Q_table = zeros(n_X1, n_X2, n_U);

% Track # of visits for each (X,U)
m = zeros(n_X1, n_X2, n_U);

for i = 1:episodes
    % Set discrete initial state
    X_index = X_index_0;
    % Set real initial state 
    X = X_0;
    % Generate episode
    for t = 1:T-1
        % Get ε-greedy control
        if rand < epsilon
            U_index = randi(n_U);
        else
            [~, U_index] = min(Q_table(X_index(1), X_index(2), :));
        end
        % Get current control
        U = U_vals(U_index);
        % Get one-step cost 
        cost = X'*Q(:,:,t)*X + U'*R(:,:,t)*U;
        % Log # of visits to this (x,u)
        m(X_index(1),X_index(2),U_index) = m(X_index(1),X_index(2),U_index) + 1;
        % Get step size for upcoming update
        gamma = 2/(1 + m(X_index(1),X_index(2),U_index));
        % Get system evolution
        X_new = A(:,:,t)*X + B(:,:,t)*U + W(:,:,t);
        % Discretize resulting state
        [~, X1_new_index] = min(abs(X1_vals - X_new(1)));
        [~, X2_new_index] = min(abs(X2_vals - X_new(2)));
        X_new_index = [X1_new_index; X2_new_index];
        % Update Q-table
        Q_table(X_index(1),X_index(2), U_index) = (1 - gamma)*Q_table(X_index(1),X_index(2), U_index) + ...
            gamma*(cost + min(Q_table(X_new_index(1),X_new_index(2), :)));
        % Repeat
        X_index = X_new_index;
        X = X_new;
    end
    % Get ε-greedy control
    if rand < epsilon
        U_index = randi(n_U);
    else
        [~, U_index] = min(Q_table(X_index(1), X_index(2), :));
    end
    % Update Q-table with final cost
    Q_table(X_index(1),X_index(2), U_index) = (X'*Q(:,:,T)*X);
end
