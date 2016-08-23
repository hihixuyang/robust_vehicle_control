function [K, P, e] = gcc(F, G, H, Ef, Eg, Q, R)
    % gcc Generates the infinite horizon Guaranteed Cost Control for a
    %     parametric uncertain linear system
    %    Inputs: F  - System matrix
    %            G  - Input matrix
    %            H  - Disturbance matrix (LHS)
    %            Ef - System disturbance matrix (RHS)
    %            Eg - Input disturbance matrix (RHS)
    %            Q  - State cost matrix
    %            R  - Input cost matrix
    %    
    %    Outputs: K - GCC gain matrix
    %             P - GCC cost matrix
    %             e - optimal epsilon
    %
    %    Author: Carlos M. Massera
    %    Instituition: University of São Paulo

    Nx = size(F,2);
    Nu = size(G,2);
    Nw = size(H,2);

    [~, s, v] = svd(Q);
    mask = (diag(s) >= 1e-10);

    Qv = s(mask, mask);
    Qp = v(:, mask)';

    Pinv = sdpvar(Nx, Nx);
    KPinv = sdpvar(Nu, Nx, 'full');
    S = sdpvar(Nx, Nx);
    e = sdpvar(1,1);

    M = blkvar;
    M(1,1) = - inv(Qv);
    M(1,5) = Qp * Pinv;
    M(2,2) = - inv(R);
    M(2,5) = KPinv;
    M(3,3) = - e * eye(Nw);
    M(3,5) = Ef * Pinv - Eg * KPinv;
    M(4,4) = - Pinv + e * (H * H');
    M(4,5) = F * Pinv - G * KPinv;
    M(5,5) = - Pinv;

    N = blkvar;
    N(1,1) = Pinv;
    N(1,2) = eye(Nx);
    N(2,2) = S;

    constraints = [Pinv >= 0;
                   M <= 0;
                   N >= 0;
                   e >= 0];

    opt = sdpsettings('solver', '+sedumi', 'verbose', 0);
    sol = optimize(constraints, trace(S), opt);

%     if sol.problem ~= 0
%         baseException = MException('gcc:solver_failed','SeDuMi did not converge');
%         throw(baseException);
%     end

    K = value(KPinv) / value(Pinv);
    P = inv(value(Pinv));
    e = value(e);

end