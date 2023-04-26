function X = projplane(x,K,R,t,n)
%PROJPLANE Project image points x onto the plane nᵗX+1 = 0.

% λx = K(RX+t) <=> X = Rᵗ(λK⁻¹x-t)
% nᵗX+1 = 0 => λ = (nᵗRᵗt-1)/(nᵗRᵗK⁻¹x)
Kinv = inv(K);
lambda = repmat((n'*R'*t-1)./(n'*R'*Kinv*x),[3 1]);
X = R'*(lambda.*(Kinv*x)-t);
end

