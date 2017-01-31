function x_hat = lateral_observer(y, x_hat_prev, u_prev, kappa, observer, has_data)
    % lateral_observer Guaranteed cost observer callback
    %    Inputs: y          - Measurement vector (ey, epsi, vx)
    %            x_hat_prev - Previous state estimate (s, ey, epsi, vx, vy, r)
    %            u_prev     - Previous control input
    %            kappa      - Path curvature
    %            observer - Observer gain structure
    %                observer.vx          - Reference velocity vector [m/s]
    %                observer.vx_min      - Minimum reference v`Nx + Nr);
    %                observer.L           - Observer gains w.r.t vx [-]
    %                observer.P           - Cost matrix w.r.t vx [-]
    %                observer.F           - State transition matrix w.r.t vx [-]
    %                observer.G           - Input matrix w.r.t vx [-]
    %                observer.Gr          - Reference matrix w.r.t vx [-]
    %                observer.C           - Output matrix w.r.t vx [-]
    %                observer.subsampling - Number of timesteps between measurements
    %            has_data   - Flag for new measurement arrived
    %
    %    Outputs: x_hat - Current state estimate (s, ey, epsi, vx, vy, r)
    %
    %    Author: Carlos M. Massera
    %    Instituition: University of São Paulo

    vx = y(3);
    y = [y(1:2)];

    % Build reduced state vector
    x_hat_prev = [x_hat_prev(2:3); x_hat_prev(5:6)];

    % Query matrices for prediction and correction
    F = permute(interp1(observer.vx, observer.F, ...
                        max(min(vx, observer.vx_max), observer.vx_min), 'spline'), [2 3 1]);
    G = permute(interp1(observer.vx, observer.G, ...
                        max(min(vx, observer.vx_max), observer.vx_min), 'spline'), [2 3 1]);
    Gr = permute(interp1(observer.vx, observer.Gr, ...
                        max(min(vx, observer.vx_max), observer.vx_min), 'spline'), [2 3 1]);

    L = permute(interp1(observer.vx, observer.L, ...
                        max(min(vx, observer.vx_max), observer.vx_min), 'spline'), [2 3 1]);
    C = observer.C;

    % Predict
    x_hat = F * x_hat_prev + G * u_prev + Gr * kappa;

    % Correct new data is available
    if has_data
        x_hat = (eye(size(F,2)) - L * C) * x_hat + L * y;
    end

    % Rebuild complete vector
    x_hat = [0; x_hat(1:2); vx; x_hat(3:4)];

end