function cost = get_cost_with_law(A, B, Q, R, X_0, T, W, K)
n = size(A,1);
m = size(B,2);
% Check if K is constant 
if size(K,3) == 1
    K_temp = K;
    K = zeros(m,n,T-1);
    for t = 1:T-1
        K(:,:,t) = K_temp;
    end
end

X = zeros(n,1,T);
X(:,1,1) = X_0;

U = zeros(m,1,T-1);

cost = 0;
% Forward run of system
for t = 1:T-1
    U(:,:,t) = K(:,:,t)*X(:,:,t);
    cost = cost + X(:,:,t)'*Q(:,:,t)*X(:,:,t) + U(:,:,t)'*R(:,:,t)*U(:,:,t);
    X(:,:,t+1) = A(:,:,t)*X(:,:,t) + B(:,:,t)*U(:,:,t) + W(:,:,t);
end
cost = cost + X(:,:,T)'*Q(:,:,T)*X(:,:,T);