function [ N ] = rdcgain(F, G, Gr, C)

    Nx = size(F, 2);
    Nu = size(G, 2);
    Ny = size(C, 1);
    Ns = size(F, 3);
    
    N = sdpvar(Nu, Ny);
    gains = sdpvar(Ns, 1);
    gamma = sdpvar(1, 1);
    
    constraints = [];
    
    for i = 1:Nx
        gains(i) = - (C / (eye(Nx) - F(:, :, i))) * (Gr - G(:, :, i) * N);
        constraints = [constraints; gains(i)' * gains(i)' <= gamma];
    end
    
    opt = sdpsettings('solver', '+sedumi', 'verbose', 0);
    sol = optimize(constraints, gamma, opt);
    
    N = value(N);

end

