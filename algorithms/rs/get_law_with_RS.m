function K = get_law_with_RS(A, B, Q, R, X_0, T, W, K_0, updates, ...
    batch_size, alpha, sigma)

% Set initial K (must be stabilizing)
K = K_0; 
[n,m] = size(K);
for i = 1:updates
    % Initialize batch cost
    batch_cost = 0;
    for b = 1:batch_size
        % Sample a standard normal vector
        ksi = randn(n,m);
        % Get a trajectory with + and - the perturbation each
        for sign = -1:2:1
            perturbation = sign*sigma*ksi;
            % Initialize state
            X = X_0;
            % Initialize total cost of a trajectory
            cost = 0;
            for t = 1:T-1
                % Get control 
                U = (K + perturbation)*X;
                % Get one-step cost 
                cost = cost + X'*Q(:,:,t)*X + U'*R(:,:,t)*U;
                % Get system evolution
                X = A(:,:,t)*X + B(:,:,t)*U + W(:,:,t);
            end
            % Get final cost
            cost = cost + X'*Q(:,:,T)*X;
            % Add total cost to batch cost
            batch_cost = batch_cost + sign*cost*ksi;
        end
    end
    % Do SGD on K
    K = K - alpha*batch_cost/(2*sigma)/batch_size;
end
end