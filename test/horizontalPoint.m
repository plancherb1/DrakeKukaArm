%% Compute the horizontal (starting) point for a Kuka arm
function [xf,uf] = horizontalPoint(p,v)
    xf = zeros(p.num_positions + p.num_velocities,1);
    xf(2) = pi/2;
    uf = zeros(size(p.umin,1),1);
end