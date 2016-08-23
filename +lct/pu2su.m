function [ Fout, Gout ] = pu2su(F, G, H, Ef, Eg)

    Nx = size(F, 2);
    Nu = size(G, 2);
    Nw = size(H, 2);

    Fout = zeros(Nx, Nx, 2^Nw);
    Gout = zeros(Nx, Nu, 2^Nw);
    
    for i = 1:2^Nw
        disturbance = zeros(Nw,1);
        
        j = i - 1;
        j_idx = 1;
        while j ~= 0
            disturbance(j_idx) = mod(j, 2);
            j = floor(j / 2);
            j_idx = j_idx + 1;
        end
        
        Delta = diag(2 * disturbance - 1);
        
        Fout(:, :, i) = F + H * Delta * Ef;
        Gout(:, :, i) = G + H * Delta * Eg;
    end
end

