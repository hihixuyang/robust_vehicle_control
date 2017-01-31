function [F, G, H, Ef, Eg] = uc2d(A, B, Hc, Ea, Eb, Ts)
    % uc2d Discretizes a parametric uncertain linear system
    %    Inputs: A   - Continuous system matrix
    %            B   - Continuous input matrix
    %            Hc  - Continuous disturbance matrix (LHS)
    %            Ea  - Continuous system disturbance matrix (RHS)
    %            Eb  - Continuous input disturbance matrix (RHS)
    %
    %    Outputs: F  - Discrete system matrix
    %             G  - Discrete input matrix
    %             H  - Discrete disturbance matrix (LHS)
    %             Ef - Discrete system disturbance matrix (RHS)
    %             Eg - Discrete input disturbance matrix (RHS)
    %
    %    Author: Carlos M. Massera
    %    Instituition: University of São Paulo

    Nx = size(A, 2);
    Nu = size(B, 2);
    Nw = size(Hc, 2);

    Mc = [A, B,                     Hc;
          zeros(Nu + Nw, Nx + Nu + Nw)];
    Md = expm(Ts * Mc);

    F = Md(1:Nx, 1:Nx);
    G = Md(1:Nx, Nx + (1:Nu));
    H = Md(1:Nx, Nx + Nu + (1:Nw));
    Ef = Ea;
    Eg = Eb;