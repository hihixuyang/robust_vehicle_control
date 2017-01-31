function [dx, ax] = nonliear_bicycle_model(x, u, kappa, mu, ax_prev, param)
    % nonliear_bicycle_model Nonlinear bicycle model dynamics with fiala tire models
    %    Inputs: x     - State vector (s, ey, epsi, vx, vy, r)
    %            u     - Input vector (delta_f, Fx_f, delta_r, Fx_r)
    %            kappa - Path curvature
    %            mu    - Tire friction coefficients (mu_f, mu_r)
    %            ax_prev - Previous longitudinal acceleration
    %            param - Parameter structure
    %                param.m  - Vehicle mass [Kg]
    %                param.l  - Wheelbase [m]
    %                param.a  - Distance from cg to front axle (x) [m]
    %                param.h  - Distance from cg to ground (z) [m]
    %                param.Iz - Yaw moment of inertia [Kgm^2/rad]
    %                param.g  - Gravity acceleration [m/s^2]
    %                param.front_tire
    %                    front_tire.C  - Cornering stiffness [N/rad]
    %                    front_tire.Ru - Radio between frictions [-]
    %                param.rear_tire
    %                    rear_tire.C  - Cornering stiffness [N/rad]
    %                    rear_tire.Ru - Radio between frictions [-]
    %                param.aero
    %                    aero.CdA - CdA coefficient [m^2]
    %                    aero.rho - Air density [kg/m^3]
    %
    %    Outputs: dx - State derivatives
    %             ax - Current longitudinal acceleration
    %
    %    Author: Carlos M. Massera
    %    Instituition: University of São Paulo

    % Name states for improved readibility
    s = x(1);
    ey = x(2);
    epsi = x(3);
    vx = x(4);
    vy = x(5);
    r = x(6);

    % name inputs for improved readibility
    delta_f = u(1);
    Fx_f = u(2);
    delta_r = u(3);
    Fx_r = u(4);

    % Load parameter for brevity
    m = param.body.m;
    l = param.body.l;
    a = param.body.a;
    b = l - a;
    h = param.body.h;
    Iz = param.body.Iz;
    dy = param.body.dy;
    g = param.world.g;

    CdA = param.aero.CdA;
    rho = param.aero.rho;

    % Load friction for brevity
    mu_f = mu(1);
    mu_r = mu(2);

    % Quantities required for spacial states derivatives
    cpsi = cos(epsi);
    spsi = sin(epsi);

    % Spacial states derivatives
    ds    = (vx * cpsi - (vy + dy * r) * spsi) / (1 - kappa * ey);
    dey   = vx * spsi + (vy + dy * r) * cpsi;
    depsi = r - kappa * ds;

    % Quantities required for dynamical states derivatives
    Fz_f = m * (b * g - ax_prev * h) / l;
    Fz_r = m * (a * g + ax_prev * h) / l;

    slip_f = atan2(vy + a * r, vx) - delta_f;
    slip_r = atan2(vy - b * r, vx) - delta_r;

    % Limit longitudinal force by normal load
    Fx_f = sign(Fx_f) * min(abs(Fx_f), 0.99 * Fz_f * mu_f);
    Fx_r = sign(Fx_r) * min(abs(Fx_r), 0.99 * Fz_r * mu_r);

    Fy_f = fiala_model(slip_f, sqrt(max(0, Fz_f^2 - (Fx_f / mu_f)^2)), param.front_tire);
    Fy_r = fiala_model(slip_r, sqrt(max(0, Fz_r^2 - (Fx_r / mu_r)^2)), param.rear_tire);

    Fx_aero = - sign(vx) * CdA * rho * vx ^ 2;

    cdf = cos(delta_f);
    sdf = sin(delta_f);
    cdr = cos(delta_r);
    sdr = sin(delta_r);

    Fx_body = Fx_f * cdf - Fy_f * sdf + Fx_r * cdr - Fy_r * sdr + Fx_aero;
    Fy_body = Fx_f * sdf + Fy_f * cdf + Fx_r * sdr + Fy_r * cdr;
    Mz_body = a * (Fx_f * sdf + Fy_f * cdf) - b * (Fx_r * sdr + Fy_r * cdr);

    % Dynamical states derivatives
    dvx = Fx_body / m + vy * r;
    dvy = Fy_body / m - vx * r;
    dr  = Mz_body / Iz;

    % Set the outputs
    dx = [ds; dey; depsi; dvx; dvy; dr];
    ax = Fx_body / m;

end