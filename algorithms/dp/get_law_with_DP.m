function [K, L] = get_law_with_DP(A, B, Q, R, T)
n = size(A,1);
m = size(B,2);
K = zeros(m,n,T-1);
L = zeros(n,n,T);
L(:,:,end) = Q(:,:,end);
% Backward recursion
for t = T-1:-1:1
    TEMP = (B(:,:,t)'*L(:,:,t+1)*B(:,:,t) + R(:,:,t))^(-1);
    K(:,:,t) = -TEMP*B(:,:,t)'*L(:,:,t+1)*A(:,:,t);
    L(:,:,t) = A(:,:,t)'*(L(:,:,t+1) - L(:,:,t+1)*B(:,:,t)*TEMP* ...
        B(:,:,t)'*L(:,:,t+1))*A(:,:,t) + Q(:,:,t);
end
end