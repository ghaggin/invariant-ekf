function lieTocartesian(filter)
    f = @func;
    kappa = 2;
    X = logm(filter.mu);
    
    x = [X(3, 2); X(1, 3); X(2, 1); X(1:3,4); X(1:3, 5); filter.bias];
    ut = unscented_transform(x, filter.Sigma, f, kappa);
    
    ut.propagate();
    
    filter.sigma_cart = ut.Cov;
end

function y = func(x)
    rot = skew(x(7:9));
    X = expm([rot, x(1:3), x(4:6); zeros(2,3), eye(2)]);
    y = [X(1:3, 4); X(1:3, 5); unskew(X(1:3,1:3))];
end

function ux  = skew(u)
    ux = [0   -u(3)  u(2)
          u(3) 0    -u(1)
         -u(2) u(1)  0];
end

function u = unskew(ux)
    u(1,1) = -ux(2,3);
    u(2,1) = ux(1,3);
    u(3,1) = -ux(1,2);
end

