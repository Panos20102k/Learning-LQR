function K = get_law_with_PG(A, B, Q, R, X_0, T, W, K_0, updates, ...
    batch_size, alpha, sigma)

% Set initial K (must be stabilizing)
K = K_0; 
[n,m] = size(K);
% Initialize baseline 
baseline = 0;
for i = 1:updates
    % Initialize the batch of stochastic gradients
    batch_sgrad = 0;
    % Initialize batch cost
    batch_cost = 0;
    for b = 1:batch_size
        % Initialize state
        X = X_0;
        % Initialize total cost of a trajectory
        cost = 0;
        % Initialize the sum of the expectation
        sum = zeros(n,m); 
        for t = 1:T-1
            % Sample a standard normal variable
            heta = randn;
            % Update the sum of the expectation
            sum = sum + heta*X';
            % Sample control from policy
            U = K*X + sigma*heta;
            % Get one-step cost 
            cost = cost + X'*Q(:,:,t)*X + U'*R(:,:,t)*U;
            % Get system evolution
            X = A(:,:,t)*X + B(:,:,t)*U + W(:,:,t);
        end
        % Get final cost
        cost = cost + X'*Q(:,:,T)*X;
        % Add total cost to batch cost
        batch_cost = batch_cost + cost;
        % Get stochastic gradient
        sgrad = (cost - baseline)*sum/sigma;
        % Add it to the batch of stochastic gradients
        batch_sgrad = batch_sgrad + sgrad;
    end
    % Update baseline (average cost of previous iteration)
    baseline = batch_cost/batch_size;
    % Do SGD on K
    K = K - alpha*batch_sgrad/batch_size;
end
end